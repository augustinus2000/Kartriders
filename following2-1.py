import cv2
from yolov8 import YOLOv8
import math
import time

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

pTime = 0
TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

# PWM_SCALE = [0.60, 1.00]
PWM_SCALE = [0.10, 0.20]
"""
1번.
뱅뱅카트 홈페이지에 카트 예상 속도 8~10 적혀있었어요. 시속 8km~10km라 생각하고 rpm은 15000, 바퀴직경은 25.5cm, 뱅뱅카트 + 사람무게를 50kg이라고 가정했어요.
이렇게 모터 스펙을 구했습니다.
그리고 우리 카트 무게를 15kg로 잡고, 카트 최대 속도가 초속 0.5m일 때 pwm비는 얼마인지를 계산했더니 0.2가 나왔어요.

-> gpt한테 계속 물어봤는데, 물어볼 때마다 자꾸 다르게 답해서... 모터 돌려보면서 정해야 할 거 같아요...
"""

# 아두이노와 직렬 통신 연결, /dev/ttyACM0 포트 사용(리눅스 환경에서 사용하는 직렬 포트 경로)
# 오류 발생시 코드 종료
try:
    board = Arduino('/dev/ttyACM0')
except Exception as e:
    print(f"아두이노 연결 실패: {e}")
    exit(1)

it = util.Iterator(board)

motor1_Lpwm = 5
motor1_Rpwm = 6
motor1_Len = 2
motor1_Ren = 4

motor2_Lpwm = 9
motor2_Rpwm = 10
motor2_Len = 7
motor2_Ren = 8

motor1 = Motor(board, motor1_Lpwm, motor1_Rpwm, motor1_Len, motor1_Ren)
motor2 = Motor(board, motor2_Lpwm, motor2_Rpwm, motor2_Len, motor2_Ren)

def RangeCalc(In, in_max, in_min, out_max, out_min):
    # mapped_value = (x_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    mapped_value = round(mapped_value, 2)
    return mapped_value

it.start()

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Initialize yolov8 object detector
model_path = "/home/soeunan/KART/yolov8n_OD/models/best.onnx"
yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)

while cap.isOpened():

    #프레임 계산
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    # Read frame from the video
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다. 종료합니다.")
        break
    
    #캠 프레임의 이미지 크기와 좌표
    frame_h, frame_w, _ = frame.shape
    frame_center_x = frame_w // 2
    frame_center_y = frame_h // 2
    frame_center = [frame_center_x, frame_center_y]
    frame_center_tu = (int(frame_center_x), int(frame_center_y))
    
    #객체 탐지
    boxes, scores, class_ids = yolov8_detector(frame)
    
    #탐지된 아이디에 0이 있다면
    if 0 in class_ids :
        
        
        #바운딩 박스 정보를 얻음
        for box, class_id in zip(boxes, class_ids) :
            
            if class_id ==0 :
            
                xmin, ymin, xmax, ymax = box.astype(int)
                box_h = ymax-ymin
                box_w = xmax-xmin
                box_center_x = xmin + (box_w // 2)
                box_center_y = ymin + (box_h // 2)
                box_center = [box_center_x, box_center_y]
                box_center_tu = (box_center_x, box_center_y)
    

                turn_direc = frame_center_x - box_center[0] # 화면 중심 좌표와 박스 중심 좌표
                distance_scale = frame_h - ymax # 화면 맨 밑 좌표와 박스 맨 밑 좌표

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

            


        #ui
        cv2.circle(frame, frame_center_tu, 5, (0,255,0), cv2.FILLED) 
        cv2.circle(frame, box_center_tu, 5, (255,0,0), cv2.FILLED) #바운딩 박스 중심점
        cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (255,0,0), 2) #바운딩 박스
        #cv2.line(frame, box_center_tu, frame_center_tu, (255 ,0, 255), 2) #서로의 중심점 연결선
        cv2.putText(frame, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

        
      


    
        cv2.imshow("Detected Objects", frame)
        print("휠체어 사용자가 탐지됨")
        
    else : 
        print("휠체어 사용자가 탐지되지 않았습니다.")
        motor1.stop()
        motor2.stop()
        continue
    

    
        
  

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        motor1.stop()
        motor2.stop()
        break

motor1.stop()
motor2.stop()

cap.release()
cv2.destroyAllWindows()
board.exit()