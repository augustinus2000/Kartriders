import cv2
import time
from ultralytics import YOLO

from pyfirmata import Arduino, PWM, OUTPUT, util

class Motor: # BTS7960 모터 드라이버는 핀 4개 사용
    def __init__(self, board, Lpwm_pin, Rpwm_pin):
        self.Lpwm = board.get_pin(f'd:{Lpwm_pin}:p')
        self.Rpwm = board.get_pin(f'd:{Rpwm_pin}:p')
        
    def forward(self, speed):
        self.Lpwm.write(speed)
        self.Rpwm.write(0)
        
    def backward(self, speed):
        self.Lpwm.write(0)
        self.Rpwm.write(speed)

    def stop(self):
        self.Lpwm.write(0)
        self.Rpwm.write(0)

# Initialize FPS tracking
pTime = 0
TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

PWM_SCALE = [0.10, 0.40] # 최대 속도 40% 출력, 최소 속도 10% 출력

# Board settings
bluetooth_port = 'COM3'
board = Arduino(bluetooth_port)
it = util.Iterator(board)

motor1 = Motor(board, 5, 6)
motor2 = Motor(board, 9, 10)

def RangeCalc(In, in_max, in_min, out_max, out_min):
    # mapped_value = (x_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    mapped_value = round(mapped_value, 2)
    mapped_value = max(min(mapped_value, out_max), out_min) # 한번 더 클리핑해서 pwm 범위 못 벗어나게 막음.
    return mapped_value

it.start()

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
                if int(class_id) == 1:  # 1이면 사람, 0이면 휠체어 인식
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

            if abs(turn_direc) > TURN_MIN_VALUE:
                if turn_direc > 0 : # 휠체어가 왼쪽에 있으면
                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    motor1.backward(pwm)
                    motor2.forward(pwm)
                    # 바퀴 순서 -> 1번 바퀴(왼쪽) ||| 카트 ||| 2번 바퀴(오른쪽)
                    print("좌회전")
                    print(f"PWM: {pwm}") # pwm 값이 0~1 사이를 잘 전달하는지 확인할려고 적어둠

                else : # 휠체어가 오른쪽에 있으면
                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    motor1.forward(pwm)
                    motor2.backward(pwm)
                    print("우회전")
                    print(f"PWM: {pwm}")

            else : # 휠체어가 화면 중심에 잘 있을 때
                if distance_scale < DISTANCE_MIN_VALUE : # 거리가 너무 가까우면
                    pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    motor1.backward(pwm)
                    motor2.backward(pwm)
                    print("후진")
                    print(f"PWM: {pwm}") 

                elif distance_scale > DISTANCE_MIN_VALUE : # 거리가 너무 멀면
                    pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    motor1.forward(pwm)
                    motor2.forward(pwm)
                    print("전진")
                    print(f"PWM: {pwm}")

                else : # 정확한 기준 거리에 도착하면
                    motor1.stop()
                    motor2.stop()
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
            motor1.stop()
            motor2.stop()
            tracking = False

    cv2.imshow("Detected Objects", frame)

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        motor1.stop()
        motor2.stop()
        break

motor1.stop()
motor2.stop()

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
board.exit()
