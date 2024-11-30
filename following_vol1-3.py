import cv2
import time
from ultralytics import YOLO

import serial # pip install pyserial

# Initialize FPS tracking
pTime = 0
TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

PWM_SCALE = [0, 51] # 아두이노에서는 pwm 제어 0~255 범위를 가진다.
# 51은 20%의 출력, 최대 속도를 20%의 출력으로 맞춰놓았다. 이건 카트 속도보고 조정하기!!

# Bluetooth 포트 설정
bluetooth_port = 'COM3'  # 실제 포트에 맞게 수정해야 함
baudrate = 9600 # 이거 아두이노에 설정한 값이랑 같아야 한다.

# 시리얼 연결 설정
ser = serial.Serial(bluetooth_port, baudrate) # 블루투스 장치와 연결

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
            distance_scale = frame_h - ymax

            if abs(turn_direc) > TURN_MIN_VALUE : # 휠체어가 화면 중심에서 벗어남
                
                if turn_direc > 0 : # 휠체어가 왼쪽에 있으면
                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    send_command('l', pwm) # 아두이노로 좌회전 신호와 pwm 값을 보냄, 쉼표로 구분 함!!!!
                    print("좌회전")
                    print(f"PWM: {pwm}") # pwm 값이 0~51 사이를 잘 전달하는지 확인할려고 적어둠

                else : # 휠체어가 오른쪽에 있으면
                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    send_command('r', pwm) # 아두이노로 우회전 신호와 pwm 값을 보냄
                    print("우회전")
                    print(f"PWM: {pwm}")

            else : # 휠체어가 화면 중심에 잘 있을 때
                if distance_scale < DISTANCE_MIN_VALUE : # 거리가 너무 가까우면
                    pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    send_command('b', pwm) # 후진
                    print("후진")
                    print(f"PWM: {pwm}") 

                elif distance_scale > DISTANCE_MIN_VALUE : # 거리가 너무 멀면
                    pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    send_command('f', pwm) # 전진
                    print("전진")
                    print(f"PWM: {pwm}")

                else : # 정확한 기준 거리에 도착하면
                    send_command('s') # 정지
                    print("정지")

            # UI
            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 0), cv2.FILLED)
            cv2.circle(frame, (box_center_x, box_center_y), 5, (255, 0, 0), cv2.FILLED)  # Bounding box center point
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)  # Bounding box
            cv2.putText(frame, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
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
