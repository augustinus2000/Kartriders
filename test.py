from cvzone.PoseModule import PoseDetector
from pyfirmata import Arduino, PWM, OUTPUT, util # SERVO 삭제

import cv2
import cvzone
import math
import time
# import RPi.GPIO as GPIO

class Motor: # BTS7960 모터 드라이버에 핀 4개 사용하도록 변경
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

# OpenCV / Board Settings
cap = cv2.VideoCapture('/dev/video0')
board = Arduino('/dev/ttyACM0')

# GPIO.setmode(GPIO.BCM)

cap.set(cv2.CAP_PROP_FPS, 20)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

detector = PoseDetector(staticMode=False, modelComplexity=0, smoothLandmarks=True, enableSegmentation=False, smoothSegmentation=True, detectionCon=0.5, trackCon=0.5)
it = util.Iterator(board)

# 모터 두개만 사용하고, 핀은 4개 사용하도록 변경.
motor1_Lpwm = 5
motor1_Rpwm = 6
motor1_Len = 2
motor1_Ren = 4

motor2_Lpwm = 9
motor2_Rpwm = 10
motor2_Len = 7
motor2_Ren = 8

# LED 삭제
""""
GPIO.setup(17, GPIO.OUT) # Red
GPIO.setup(27, GPIO.OUT) # Yellow
GPIO.setup(22, GPIO.OUT) # Green
GPIO.setup(16, GPIO.OUT) # Buzzer
"""

# Pixel Values Settings
pTime = 0

TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

PWM_SCALE = [0.60, 1.00]

# 모터 두개만 쓰도록 변경, def __init__(self, board, Lpwm_pin, Rpwm_pin, Len_pin, Ren_pin): 이거랑 순서 꼭 맞춰줘야 함!!
motor1 = Motor(board, motor1_Lpwm, motor1_Rpwm, motor1_Len, motor1_Ren)
motor2 = Motor(board, motor2_Lpwm, motor2_Rpwm, motor2_Len, motor2_Ren)

# Buzzer 삭제
""""
def Buzzer():
    GPIO.output(16, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(16, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(16, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(16, GPIO.LOW)
"""

def RangeCalc(In, in_max, in_min, out_max, out_min):
    # mapped_value = (x_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    mapped_value = round(mapped_value, 2)
    return mapped_value

it.start()
# Buzzer()

while True:
    
    # GPIO.output(17, GPIO.HIGH)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    success, img = cap.read()
    height, width, _ = img.shape
    img_center_x = width // 2
    img_center_y = height // 2
    img_center = [img_center_x, img_center_y]

    img = detector.findPose(img)

    imList, bboxs = detector.findPosition(img, draw=True, bboxWithHands=False)

    if bboxs:
        
        # GPIO.output(22, GPIO.HIGH)

        bbox = bboxs[0]

        center = bbox["center"]
        x, y, w, h = bbox['bbox']
        
        turn_direc = img_center_x - center[0]
        distance_scale = img_center_y - center[1]

        # UI
        cv2.circle(img, center, 5, (255, 0, 0), cv2.FILLED) # Bbox Center
        cvzone.cornerRect(img, (x, y, w, h), 30, 3, 3, (255,0,255), (255,0,255)) # Bbox
        cv2.line(img, center, (img_center_x, img_center_y), (255 ,0, 255), 2) # Line Bbox -> Center Img

        cv2.putText(img, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        cv2.putText(img, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

        if abs(distance_scale) > DISTANCE_MIN_VALUE: # 거리가 멀거나 가까우면
            
            if distance_scale < 0: # 거리가 멀면

                cv2.putText(img, "Action : Far", (20, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                
                pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                cv2.putText(img, f"PWM :  {pwm}", (20, 220), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
                
                motor1.forward(pwm)
                motor2.forward(pwm)
                
            else: # 거리가 가까우면

                cv2.putText(img, "Action : Near", (20, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                cv2.putText(img, f"PWM :  {pwm}", (20, 220), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
                
                motor1.backward(pwm)
                motor2.backward(pwm)

        else: # 거리가 적당하면

            if abs(turn_direc) > TURN_MIN_VALUE: # 사람이 영상 중심에서 벗어났으면

                if turn_direc < 0: # 사람이 오른쪽에 있으면
                    cv2.putText(img, "Turn Direction : Right", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    cv2.putText(img, f"PWM :  {pwm}", (20, 120), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

                    motor1.forward(pwm)
                    motor2.backward(pwm)
                    # 바퀴 순서는 1번바퀴(왼쪽), 2번바퀴(오른쪽) -> 1번바퀴 ||| 몸통 ||| 2번바퀴
                
                else: # 사람이 왼쪽에 있으면
                    cv2.putText(img, "Turn Direction : Left", (20, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    cv2.putText(img, f"PWM :  {pwm}", (20, 120), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
                    
                    # 모터 두개로 변경
                    motor1.backward(pwm)
                    motor2.forward(pwm)
            else:
      
                motor1.stop()
                motor2.stop()
    else:
        
        # GPIO.output(22, GPIO.LOW)
        motor1.stop()
        motor2.stop()

    cv2.putText(img, f'FPS : {int(fps)}', (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2) # FPS
    cv2.circle(img, (img_center_x, img_center_y), 5, (255, 0, 0), cv2.FILLED) # Middle Circle
    cv2.line(img, (0, img_center_y), (width, img_center_y), (0, 255, 0), 1)  # Horizontal line
    cv2.line(img, (img_center_x, 0), (img_center_x, height), (0, 255, 0), 1) # Vertical line
    
    img = cv2.resize(img, (width*2, height*2))
    #cv2.imshow("Image", img)

    key = cv2.waitKey(1)
    if key == ord('q'):
        # GPIO.output(17, GPIO.LOW)
        # GPIO.output(22, GPIO.LOW)
        
        motor1.stop()
        motor2.stop()
        break

# GPIO.output(17, GPIO.LOW)
# GPIO.output(22, GPIO.LOW)
# GPIO.cleaup()

motor1.stop()
motor2.stop()

cap.release()
board.exit()
