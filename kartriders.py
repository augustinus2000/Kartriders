import cv2
import time
from ultralytics import YOLO

import serial

pTime = 0
TURN_MIN_VALUE = 100
TURN_MAX_VALUE = 500

DISTANCE_MIN_VALUE = 10
DISTANCE_MAX_VALUE = 50

PWM_SCALE_TURN = [25, 50] # 회전 pwm 값은 거리 pwm 값에서 뺄 거니까 낮게 설정해도 괜찮을 거 같다.
PWM_SCALE_DISTANCE = [77, 127]

# Bluetooth 포트 설정
bluetooth_port = 'COM5'  # 실제 포트에 맞게 수정해야 함
baudrate = 115200 # 이거 아두이노에 설정한 값이랑 같아야 한다.

# 시리얼 연결 설정
ser = serial.Serial(bluetooth_port, baudrate) # 블루투스 장치와 연결

# 이동 평균 필터 안관련 변수 초기화
alpha = 0.5  # 이동 평균 필터 가중치, 0~1 사이 값을 줘야 한다. 
# 0에 가까울수록 정성이 올라가지만, 반응 속도는 느려진다.
previous_distance = 0  # 이전 거리값 저장 변수
previous_turn = 0

# 명령 전송
def send_command(char, num1=0, num2=0): # send_command는 블루투스를 통해 명령을 아두이노로 전송하는 함수.\
    # num의 기본 값은 0으로 설정해서 정지할 때는 pwm 값을 아두이노로 안 보내도 된다.
    signal = f"{char},{num1},{num2}\n" # 문자와 숫자를 쉼표로 구분해서 전송. 
    ser.write(signal.encode()) # command 문자열을 바이트 형식으로 변환하고 ser을 통해 전송.
    # print(f"명령 '{signal}' 전송됨")

def RangeCalc(In, in_max, in_min, out_max, out_min):
    x = min(max(In, in_min), in_max)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Initialize the webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("웹캠 초기화 실패. 종료합니다.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Initialize YOLOv8 object detector
model_path = "C:/vscode/HappyHappy/best.pt"
model = YOLO(model_path)

# Initialize OpenCV Tracker
tracker = cv2.TrackerCSRT_create()  # Use CSRT tracker
tracking = False  # Flag to check if we are tracking an object
bbox = None  # Bounding box for the object
frame_count = 0  # Frame counter for periodic YOLO updates

def calculate_iou(box1, box2):
    """두 박스의 IoU 계산."""
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    # 두 박스의 우측 하단 좌표 계산
    x1_end, y1_end = x1 + w1, y1 + h1
    x2_end, y2_end = x2 + w2, y2 + h2

    # 교차 영역의 좌상단과 우하단 좌표 계산
    x_intersect = max(x1, x2)
    y_intersect = max(y1, y2)
    x_intersect_end = min(x1_end, x2_end)
    y_intersect_end = min(y1_end, y2_end)

    # 교차 영역의 넓이 계산
    intersection_width = max(0, x_intersect_end - x_intersect)
    intersection_height = max(0, y_intersect_end - y_intersect)

    intersection_area = intersection_width * intersection_height

    # 두 박스의 넓이 계산
    area1 = w1 * h1
    area2 = w2 * h2

    # IOU 계산
    union_area = area1 + area2 - intersection_area
    iou = intersection_area / union_area if union_area > 0 else 0

    return iou

# 바운딩 박스 줄이기 shrink_factor = 0.2 -> 20퍼센트 줄어듬
def shrink_bbox(xmin, ymin, xmax, ymax, shrink_factor=0.2):
    width = xmax - xmin
    height = ymax - ymin

    # 중심으로부터 shrink_factor 비율로 축소
    new_xmin = xmin + int(width * shrink_factor / 2)
    new_ymin = ymin + int(height * shrink_factor / 2)
    new_xmax = xmax - int(width * shrink_factor / 2)
    new_ymax = ymax - int(height * shrink_factor / 2)

    return (new_xmin, new_ymin, new_xmax, new_ymax)

# yolo 바운딩 박스와 트래커 바운딩 박스 합치기 alpha=0이면 완전히 YOLO 박스를 사용
def calculate_fused_bbox(tracker_bbox, yolo_bbox, A=0.5):
    x1_t, y1_t, w_t, h_t = tracker_bbox
    x2_t, y2_t = x1_t + w_t, y1_t + h_t

    x1_y, y1_y, x2_y, y2_y = yolo_bbox

    # 융합된 바운딩 박스 계산 (가중 평균)
    x1_f = int(A * x1_t + (1 - A) * x1_y)
    y1_f = int(A * y1_t + (1 - A) * y1_y)
    x2_f = int(A * x2_t + (1 - A) * x2_y)
    y2_f = int(A * y2_t + (1 - A) * y2_y)

    return (x1_f, y1_f, x2_f - x1_f, y2_f - y1_f)  # (xmin, ymin, width, height)

# Function to handle object detection and tracking in parallel (for performance)
def detect_and_track(frame):
    global bbox, tracking, tracker, frame_count

    frame_count += 1

    if not tracking or frame_count % 15 == 0:  # YOLO는 15프레임마다 실행
        results = model.predict(source=frame, conf=0.7, iou=0.7, save=False, verbose=False)
        detections = results[0]

        if len(detections) > 0:
            boxes = detections.boxes.xyxy.cpu().numpy()
            class_ids = detections.boxes.cls.cpu().numpy()

            detected = False

            for box, class_id in zip(boxes, class_ids):
                if int(class_id) == 0:  # 휠체어 클래스만 탐지
                    xmin, ymin, xmax, ymax = map(int, box)

                    # YOLO 바운딩 박스 축소
                    xmin, ymin, xmax, ymax = shrink_bbox(xmin, ymin, xmax, ymax, shrink_factor=0.2)
                    yolo_bbox = (xmin, ymin, xmax - xmin, ymax - ymin)

                    if tracking:
                        # 트래커의 현재 박스와 YOLO 박스 융합
                        success, tracker_bbox = tracker.update(frame)
                        if success:
                            iou = calculate_iou(tracker_bbox, yolo_bbox)
                            if iou > 0.7:
                                bbox = calculate_fused_bbox(tracker_bbox, yolo_bbox)  # 융합된 박스
                            else:
                                # IoU가 낮으면 트래커를 YOLO 박스로 초기화
                                tracker.init(frame, yolo_bbox)
                                bbox = yolo_bbox
                        else:
                            # 트래커 실패 시 YOLO 박스로 초기화
                            tracker.init(frame, yolo_bbox)
                            bbox = yolo_bbox
                    else:
                        # 최초 탐지 시 YOLO 박스로 트래커 초기화
                        tracker.init(frame, yolo_bbox)
                        bbox = yolo_bbox
                        print("Tracking initialized.")

                    tracking = True
                    detected = True
                    break  # 첫 번째 휠체어만 추적

            if not detected:
                print("No wheelchair user detected.")
        else:
            print("No detections found.")

    if tracking:
        success, tracker_bbox = tracker.update(frame)
        if success:
            bbox = tracker_bbox
            xmin, ymin, w, h = map(int, bbox)
            xmax = xmin + w
            ymax = ymin + h

            # 중심 계산 및 제어
            box_center_x = xmin + w // 2
            box_center_y = ymin + h // 2
            frame_h, frame_w, _ = frame.shape
            frame_center_x = frame_w // 2
            frame_center_y = frame_h // 2
            turn_direc = box_center_x - frame_center_x
            distance_scale = frame_h - ymax
            
            
            # 이동 평균 필터 적용 (새로운 거리값을 부드럽게 업데이트)
            global previous_distance
            global previous_turn
            
            smoothed_distance = alpha * distance_scale + (1 - alpha) * previous_distance  # 이동 평균 필터
            previous_distance = smoothed_distance  # 현재 거리값을 다음에 사용할 이전 값으로 저장
            smoothed_turn = alpha * turn_direc + (1 - alpha) * previous_turn
            previous_turn = smoothed_turn

            # 거리 제어
            if smoothed_distance < (DISTANCE_MIN_VALUE - 5): # 가까움
                distance_command = 'b'
                pwm_distance = RangeCalc(abs(smoothed_distance), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE_DISTANCE[1], PWM_SCALE_DISTANCE[0])
            elif smoothed_distance > (DISTANCE_MIN_VALUE + 5): # 멀다
                distance_command = 'f' 
                pwm_distance = RangeCalc(abs(smoothed_distance), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE_DISTANCE[1], PWM_SCALE_DISTANCE[0])
            else: # 거리가 5~15 사이
                distance_command = 's'
                pwm_distance = 0

            # 회전 제어
            if abs(smoothed_turn) > TURN_MIN_VALUE: # 중심에서 벗어남
                pwm_turn = RangeCalc(abs(smoothed_turn), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE_TURN[1], PWM_SCALE_TURN[0])
                if smoothed_turn > 0: # 좌회전
                    turn_command = 'l'
                else: # 우회전
                    turn_command = 'r'
            else:
                turn_command = 's'
                pwm_turn = 0

            # 명령 전송
            if distance_command == 's' and turn_command == 's':   # 정지 + 회전x
                send_command('s', pwm_distance, pwm_turn)
            elif distance_command == 'f' and turn_command == 's': # 전진 + 회전x
                send_command('f', pwm_distance, pwm_turn)
            elif distance_command == 'f' and turn_command == 'l': # 전진 + 좌회전
                send_command('z', pwm_distance, pwm_turn)
            elif distance_command == 'f' and turn_command == 'r': # 전진 + 우회전
                send_command('x', pwm_distance, pwm_turn)
            elif distance_command == 'b' and turn_command == 's': # 후진 + 회전x
                send_command('b', pwm_distance, pwm_turn)
            elif distance_command == 'b' and turn_command == 'l': # 후진 + 좌회전
                send_command('c', pwm_distance, pwm_turn)
            elif distance_command == 'b' and turn_command == 'r': # 후진 + 우회전
                send_command('v', pwm_distance, pwm_turn)
            elif distance_command == 's' and turn_command == 'l': # 정지 + 좌회전
                send_command('l', pwm_distance, pwm_turn)
            elif distance_command == 's' and turn_command == 'r': # 정지 + 우회전
                send_command('r', pwm_distance, pwm_turn)
            else:
                print("모르는 명령")


            # UI for visualization
            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 0), cv2.FILLED)
            cv2.circle(frame, (box_center_x, box_center_y), 5, (255, 0, 0), cv2.FILLED)  # Bounding box center point
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)  # Bounding box
            cv2.putText(frame, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.line(frame, (box_center_x, box_center_y), (frame_center_x, frame_center_y), (255, 0, 255), 2)
        else:
            print("Tracking lost. Re-detecting object...")
            send_command('s')
            tracking = False

    return frame

# Main roop
try:
    while True:
        # Capture frame from camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame. Exiting...")
            break
        
        """
        # Calculate FPS
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        """

        # Perform object detection and tracking
        frame = detect_and_track(frame)

        # Display the frame
        cv2.imshow("Detected Objects", frame)

        # Press key 'q' to stop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exit key pressed.")
            break
except KeyboardInterrupt: # Ctrl + C 눌러서 정지했을 때
    print("\nProgram interrupted by user.")
finally:
    # 프로그램이 어떤 이유로 종료되든, 반드시 실행되는 코드
    send_command('s')  # 멈춰
    ser.close()  # 블루투스 연결 종료
    cap.release()  # 카메라 종료
    cv2.destroyAllWindows()  # opencv 창 종료
    print("Resources released. Program terminated.")
