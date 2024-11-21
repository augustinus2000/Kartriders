from picamera2 import Picamera2, Preview
from cvzone.PoseModule import PoseDetector
from pyfirmata import Arduino, PWM, OUTPUT, util

import cv2
import cvzone
import math
import time

class Motor:
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


# Picamera2 settings
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(main={"size": (640, 360)})
picam2.configure(camera_config)
picam2.start()

# Board settings
board = Arduino('/dev/ttyACM0')
it = util.Iterator(board)

motor1 = Motor(board, 5, 6, 2, 4)
motor2 = Motor(board, 9, 10, 7, 8)

# Pose detector
detector = PoseDetector(staticMode=False, modelComplexity=0, smoothLandmarks=True, enableSegmentation=False, smoothSegmentation=True, detectionCon=0.5, trackCon=0.5)

pTime = 0
TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160
DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120
PWM_SCALE = [0.10, 0.20]

def RangeCalc(In, in_max, in_min, out_max, out_min):
    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return round(mapped_value, 2)

it.start()

while True:
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    # Capture frame
    frame = picam2.capture_array()
    img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    height, width, _ = img.shape
    img_center_x = width // 2
    img_center_y = height // 2

    img = detector.findPose(img)
    imList, bboxs = detector.findPosition(img, draw=True, bboxWithHands=False)

    if bboxs:
        bbox = bboxs
        center = bbox["center"]
        x, y, w, h = bbox['bbox']

        turn_direc = img_center_x - center[0]
        distance_scale = img_center_y - center[1]

        cv2.circle(img, center, 5, (255, 0, 0), cv2.FILLED)
        cvzone.cornerRect(img, (x, y, w, h), 30, 3, 3, (255, 0, 255), (255, 0, 255))
        cv2.line(img, center, (img_center_x, img_center_y), (255, 0, 255), 2)

        cv2.putText(img, f"Distance : {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        cv2.putText(img, f"Turn Offset Value : {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

        if abs(distance_scale) > DISTANCE_MIN_VALUE:
            pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
            if distance_scale < 0:
                cv2.putText(img, "Action : Far", (20, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                motor1.forward(pwm)
                motor2.forward(pwm)
            else:
                cv2.putText(img, "Action : Near", (20, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                motor1.backward(pwm)
                motor2.backward(pwm)
        elif abs(turn_direc) > TURN_MIN_VALUE:
            pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
            if turn_direc < 0:
                cv2.putText(img, "Turn Direction : Right", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                motor1.forward(pwm)
                motor2.backward(pwm)
            else:
                cv2.putText(img, "Turn Direction : Left", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                motor1.backward(pwm)
                motor2.forward(pwm)
        else:
            cv2.putText(img, "good", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            motor1.stop()
            motor2.stop()
    else:
        cv2.putText(img, "No person detected", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
        motor1.stop()
        motor2.stop()

    cv2.putText(img, f'FPS : {int(fps)}', (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2)
    cv2.circle(img, (img_center_x, img_center_y), 5, (255, 0, 0), cv2.FILLED)
    cv2.line(img, (0, img_center_y), (width, img_center_y), (0, 255, 0), 1)
    cv2.line(img, (img_center_x, 0), (img_center_x, height), (0, 255, 0), 1)

    img = cv2.resize(img, (width * 2, height * 2))
    cv2.imshow("Image", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        motor1.stop()
        motor2.stop()
        break

cv2.destroyAllWindows()
picam2.stop()
