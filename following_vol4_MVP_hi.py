import cv2
import time
from ultralytics import YOLO

import serial

pTime = 0
TURN_MIN_VALUE = 100
TURN_MAX_VALUE = 500

DISTANCE_MIN_VALUE = 10
DISTANCE_MAX_VALUE = 100

PWM_SCALE_TURN = [51, 77]
PWM_SCALE_DISTANCE = [102, 127]

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
def send_command(char, num=0): # send_command는 블루투스를 통해 명령을 아두이노로 전송하는 함수.\
    # num의 기본 값은 0으로 설정해서 정지할 때는 pwm 값을 아두이노로 안 보내도 된다.
    signal = f"{char},{num}\n" # 문자와 숫자를 쉼표로 구분해서 전송. 
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

# Function to handle object detection and tracking in parallel (for performance)
def detect_and_track(frame):
    global bbox, tracking, tracker, frame_count

    frame_count += 1

    if not tracking or frame_count % 10 == 0:  # Use YOLO every 10 frames or when not tracking
        results = model.predict(source=frame, conf=0.7, iou=0.7, save=False, verbose=False)
        detections = results[0]

        if len(detections) > 0:
            boxes = detections.boxes.xyxy.cpu().numpy()
            class_ids = detections.boxes.cls.cpu().numpy()

            # Flag to track the first detected wheelchair
            detected = False

            for box, class_id in zip(boxes, class_ids):
                if int(class_id) == 0:  # Check for wheelchair user class
                    xmin, ymin, xmax, ymax = map(int, box)
                    detected_bbox = (xmin, ymin, xmax - xmin, ymax - ymin)

                    if tracking:
                        # Calculate IoU between YOLO and tracker bbox
                        iou = calculate_iou(detected_bbox, bbox)
                        if iou < 0.7:  # IoU threshold for reinitialization (improve threshold)
                            print("Low IoU. Gradually reinitializing tracker with YOLO detection.")
                            
                            del tracker
                            
                            tracker = cv2.TrackerCSRT_create()  # Create a new tracker
                            tracker.init(frame, detected_bbox)  # Reinitialize tracker
                            bbox = detected_bbox
                        else:
                            print("IoU sufficient. Keeping tracker bbox.")
                    else:
                        tracker.init(frame, detected_bbox)
                        bbox = detected_bbox
                        print("Tracking initialized.")

                    tracking = True
                    detected = True
                    break  # Only track the first detected wheelchair

            if not detected:
                print("No wheelchair user detected.")
        else:
            print("No detections found.")

    if tracking:
        success, updated_bbox = tracker.update(frame)
        if success:
            bbox = updated_bbox
            xmin, ymin, w, h = map(int, bbox)
            xmax = xmin + w
            ymax = ymin + h

            # Calculate the center of the frame
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

            # 모터 제어
            if abs(turn_direc) > TURN_MIN_VALUE : # 휠체어가 화면 중심에서 벗어남
                
                if turn_direc > 0 : # 휠체어가 왼쪽에 있으면
                    pwm = RangeCalc(abs(smoothed_turn), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE_TURN[1], PWM_SCALE_TURN[0])
                    send_command('l', pwm) # 아두이노로 좌회전 신호와 pwm 값을 보냄, 쉼표로 구분 함!!!!
                    print("좌회전")
                    print(f"PWM: {pwm}") # pwm 값이 0~255 사이를 잘 전달하는지 확인할려고 적어둠

                else : # 휠체어가 오른쪽에 있으면
                    pwm = RangeCalc(abs(smoothed_turn), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE_TURN[1], PWM_SCALE_TURN[0])
                    send_command('r', pwm) # 아두이노로 우회전 신호와 pwm 값을 보냄
                    print("우회전")
                    print(f"PWM: {pwm}")

            else : # 휠체어가 화면 중심에 잘 있을 때
                if smoothed_distance < (DISTANCE_MIN_VALUE - 5) : # 거리가 너무 가까우면
                    pwm = RangeCalc(abs(smoothed_distance), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE_DISTANCE[1], PWM_SCALE_DISTANCE[0])
                    send_command('b', pwm) # 후진
                    print("후진")
                    print(f"PWM: {pwm}") 

                elif smoothed_distance > (DISTANCE_MIN_VALUE + 5) : # 거리가 너무 멀면
                    pwm = RangeCalc(abs(smoothed_distance), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE_DISTANCE[1], PWM_SCALE_DISTANCE[0])
                    send_command('f', pwm) # 전진
                    print("전진")
                    print(f"PWM: {pwm}")

                else : # 기준거리 5~15에 도달하면
                    send_command('s') # 정지
                    print("정지")

            # UI
            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 0), cv2.FILLED)
            cv2.circle(frame, (box_center_x, box_center_y), 5, (255, 0, 0), cv2.FILLED)  # Bounding box center point
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)  # Bounding box
            cv2.putText(frame, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.line(frame, (box_center_x, box_center_y), (frame_center_x, frame_center_y), (255 ,0, 255), 2)
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
