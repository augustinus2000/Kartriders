import cv2
import time
from ultralytics import YOLO

import serial # pip install pyserial

# Initialize FPS tracking
pTime = 0
TURN_MIN_VALUE = 100 # 회전을 부드럽게 하도록 값 조정하기
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30 # 거리도 카트 굴려보면서 정하기
DISTANCE_MAX_VALUE = 120

PWM_SCALE = [51, 77] # 아두이노에서는 pwm 제어 0~255 범위를 가진다.
# 51은 20%의 출력, 77은 30%의 출력, 이 값은 카트 속도 보고 정하기.
# 카트 무게 때문인지... 최소를 51 밑으로 잡으면 안 굴러가요ㅠㅠ

# Bluetooth 포트 설정
bluetooth_port = 'COM3'  # 실제 포트에 맞게 수정해야 함
baudrate = 9600 # 이거 아두이노에 설정한 값이랑 같아야 한다.

# 시리얼 연결 설정
ser = serial.Serial(bluetooth_port, baudrate) # 블루투스 장치와 연결

# 이동 평균 필터 관련 변수 초기화
alpha = 0.5  # 이동 평균 필터 가중치, 0~1 사이 값을 줘야 한다. 
# 0에 가까울수록 안정성이 올라가지만, 반응 속도는 느려진다.
previous_distance = 0  # 이전 거리값 저장 변수

# 명령 전송
def send_command(char, num=0): # send_command는 블루투스를 통해 명령을 아두이노로 전송하는 함수.\
    # num의 기본 값은 0으로 설정해서 정지할 때는 pwm 값을 아두이노로 안 보내도 된다.
    signal = f"{char},{num}\n" # 문자와 숫자를 쉼표로 구분해서 전송. 
    ser.write(signal.encode()) # command 문자열을 바이트 형식으로 변환하고 ser을 통해 전송.
    print(f"명령 '{signal}' 전송됨")

def RangeCalc(In, in_max, in_min, out_max, out_min): # pwm 계산 함수
    # mapped_value = (x_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    mapped_value = max(min(mapped_value, out_max), out_min) # 한번 더 클리핑해서 pwm 범위 못 벗어나게 막음.
    return mapped_value

# Initialize the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize YOLOv8 object detector
model_path = "C:/vscode/Arduino_Bluetooth/best.pt"
model = YOLO(model_path)

# Initialize OpenCV Tracker
tracker = cv2.TrackerCSRT_create()  # Use CSRT tracker
tracking = False  # Flag to check if we are tracking an object
bbox = None  # Bounding box for the object

while True:
    # Capture frame from camera
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame. Exiting...")
        break

    # Calculate FPS
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    # If not tracking, use YOLO for object detection
    if not tracking:
        results = model.predict(source=frame, conf=0.5, iou=0.5, save=False, verbose=False)
        detections = results[0]
       
        if len(detections) > 0:
            boxes = detections.boxes.xyxy.cpu().numpy()
            class_ids = detections.boxes.cls.cpu().numpy()

            for box, class_id in zip(boxes, class_ids):
                if int(class_id) == 1:  # 0이면 휠체어, 1이면 사람
                    xmin, ymin, xmax, ymax = map(int, box)
                    bbox = (xmin, ymin, xmax - xmin, ymax - ymin)

                    # Initialize Tracker with detected bounding box
                    tracker.init(frame, bbox)
                    tracking = True
                    print("Tracking initialized.")
                    break
        else:
            print("No wheelchair user detected.")
            continue
    else:
        # Update tracker
        success, bbox = tracker.update(frame)

        if success:
            # Tracker successfully tracked the object
            xmin, ymin, w, h = map(int, bbox)
            xmax = xmin + w
            ymax = ymin + h
            box_center_x = xmin + w // 2
            box_center_y = ymin + h // 2

            # Calculate the center of the frame
            frame_h, frame_w, _ = frame.shape
            frame_center_x = frame_w // 2
            frame_center_y = frame_h // 2

            # Calculate turn direction based on horizontal offset
            turn_direc = box_center_x - frame_center_x

            # 거리 계산 (프레임 높이를 기준으로 거리 스케일 계산)
            pixel_height = ymax - ymin
            distance = frame_h - ymax  # 기본 거리 계산
            # distance = max(1, distance)  # 0으로 나뉘는 경우 방지 -> 혹시 오류나면 주석 지우기.

            # 이동 평균 필터 적용 (새로운 거리값을 부드럽게 업데이트)
            smoothed_distance = alpha * distance + (1 - alpha) * previous_distance  # 이동 평균 필터
            previous_distance = smoothed_distance  # 현재 거리값을 다음에 사용할 이전 값으로 저장

            if abs(turn_direc) > TURN_MIN_VALUE:  # 휠체어가 화면 중심에서 벗어남
                pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                if turn_direc > 0:  # 휠체어가 왼쪽에 있으면
                    send_command('l', pwm)
                    print("좌회전")
                else:  # 휠체어가 오른쪽에 있으면
                    send_command('r', pwm)
                    print("우회전")
                print(f"PWM: {pwm}")
            else:  # 휠체어가 화면 중심에 있을 때
                if smoothed_distance < DISTANCE_MIN_VALUE:  # 거리가 너무 가까우면
                    pwm = RangeCalc(abs(smoothed_distance), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    send_command('b', pwm)
                    print("후진")
                elif smoothed_distance > DISTANCE_MAX_VALUE:  # 거리가 너무 멀면
                    pwm = RangeCalc(abs(smoothed_distance), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    send_command('f', pwm)
                    print("전진")
                else:  # 적절한 거리 유지
                    send_command('s')
                    print("정지")

            # UI
            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 0), cv2.FILLED)
            cv2.circle(frame, (box_center_x, box_center_y), 5, (255, 0, 0), cv2.FILLED)  # Bounding box center point
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)  # Bounding box
            cv2.putText(frame, f"Distance: {smoothed_distance:.2f}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Turn Offset: {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        else:
            # If tracking fails, reset tracking
            print("Tracking lost. Re-detecting object...")
            send_command('s')
            tracking = False

    cv2.imshow("Detected Objects", frame)

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        send_command('s')
        break

if ser.is_open:
    send_command('s')
    ser.close()  # 종료 시 시리얼 포트 닫기

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
