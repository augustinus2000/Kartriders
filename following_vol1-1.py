import cv2
import time
from yolov8 import YOLOv8
from picamera2 import Picamera2, Preview
from pyfirmata import Arduino, PWM, OUTPUT, util # 아두이노와 라즈베리파이 연결

class Motor: # BTS7960 모터 드라이버는 핀 4개 사용
    def __init__(self, board, Lpwm_pin, Rpwm_pin, Len_pin, Ren_pin):
        self.Lpwm = board.get_pin(f'd:{Lpwm_pin}:p')
        self.Rpwm = board.get_pin(f'd:{Rpwm_pin}:p')
        self.Len = board.get_pin(f'd:{Len_pin}:o')
        self.Ren = board.get_pin(f'd:{Ren_pin}:o')
        
    def forward(self, speed):
        self.Len.write(1)
        self.Ren.write(0)
        self.Lpwm.write(speed)
        self.Rpwm.write(0)
        
    def backward(self, speed):
        self.Len.write(0)
        self.Ren.write(1)
        self.Lpwm.write(0)
        self.Rpwm.write(speed)

    def stop(self):
        self.Len.write(0)
        self.Ren.write(0)
        self.Lpwm.write(0)
        self.Rpwm.write(0)

# Initialize FPS tracking
pTime = 0
TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

PWM_SCALE = [0.10, 0.40] # 최대 속도 40% 출력, 최소 속도 10% 출력

board = Arduino('/dev/ttyACM0')
it = util.Iterator(board)

motor1 = Motor(board, 5, 6, 2, 4)
motor2 = Motor(board, 9, 10, 7, 8)

def RangeCalc(In, in_max, in_min, out_max, out_min):
    # mapped_value = (x_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    mapped_value = round(mapped_value, 2)
    return mapped_value

it.start()

# Initialize YOLOv8 object detector
model_path = "/home/kartriders/Documents/kart/yolo8n_OD/models/best.onnx"
yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)

# Initialize Picamera2
picam2 = Picamera2()

# Create a still configuration and set resolution to 640x480
config = picam2.create_still_configuration()

# Modify the resolution of the main camera sensor to 640x480
config["main"]["size"] = (640, 480)

# Configure the camera with the updated resolution
picam2.configure(config)

# Start the camera
picam2.start()

# Wait for the camera to initialize
time.sleep(2)

# Initialize tracker
tracker = cv2.TrackerCSRT_create()  # You can also try TrackerKCF_create() or others

tracking = False  # Flag to check if we are tracking an object
bbox = None  # Bounding box for the object

while True:
    # Capture frame from camera
    frame = picam2.capture_array()

    # Calculate FPS
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    # If we are not tracking, detect the object
    if not tracking:
        # Object detection using YOLOv8
        boxes, scores, class_ids = yolov8_detector(frame)

        if 0 in class_ids:
            # Get bounding box information for the wheelchair user (class_id == 0)
            for box, class_id in zip(boxes, class_ids):
                if class_id == 0:
                    xmin, ymin, xmax, ymax = box.astype(int)
                    bbox = (xmin, ymin, xmax - xmin, ymax - ymin)

                    # Initialize the tracker with the bounding box
                    tracker.init(frame, bbox)
                    tracking = True
                    print("Tracking wheelchair user...")
                    break
        else:
            print("No wheelchair user detected.")
            continue
    else:
        # Update the tracker
        success, bbox = tracker.update(frame)

        if success:
            # Tracker successfully tracked the object
            xmin, ymin, w, h = [int(v) for v in bbox]
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

            cv2.imshow("Detected Objects", frame)
        else:
            # If tracking fails, we need to re-detect the object
            print("Tracking lost. Re-detecting object...")
            motor1.stop()
            motor2.stop()
            tracking = False

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        motor1.stop()
        motor2.stop()
        break

motor1.stop()
motor2.stop()

# Stop the camera and close all windows
picam2.stop()
cv2.destroyAllWindows()
board.exit()