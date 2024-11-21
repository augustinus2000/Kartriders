from pyfirmata import Arduino, PWM, OUTPUT, util

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

# Board settings
board = Arduino('/dev/ttyACM0')
it = util.Iterator(board)


motor1_Lpwm = 5
motor1_Rpwm = 6
motor1_Len = 2  
motor1_Ren = 4 

motor2_Lpwm = 9
motor2_Rpwm = 10
motor2_Len = 7
motor2_Ren = 8

"""
모터 드라이버에서 모터 구동부 전원은 따로 켜고 끌 수 있어요. Vcc는 모터 드라이버 전원이고 enable은 모터 구동부 전원입니다.
목요일에 테스트했을 때는 Len과 Ren 신호를 둘 다 빵판의 5v에 연결해두고 썼는데, 이렇게 하면 모터 구동부 전원이 항상 켜져 있게 돼요.
위의 코드처럼 Len과 Ren 신호를 빵판 말고 아두이노 핀에 연결해서, 쓸 때는 1로 안 쓸 때는 0으로 제어하면 배터리를 절약할 수 있어요.
예시) 전진할 때는 Len은 1, Ren은 0으로 두기.
"""

motor1 = Motor(board, motor1_Lpwm, motor1_Rpwm, motor1_Len, motor1_Ren)
motor2 = Motor(board, motor2_Lpwm, motor2_Rpwm, motor2_Len, motor2_Ren)

# 위의 코드 이렇게 줄일 수 있어요.
"""
motor1 = Motor(board, 5, 6, 2, 4)
motor2 = Motor(board, 9, 10, 7, 8)
"""

it.start()

# pwm 제어처럼 움직이게 만들어 봤어요.
print("전진")
motor1.forward(0.4)
motor2.forward(0.4)
time.sleep(3)  # 3초 동안 전진

motor1.forward(0.3)
motor2.forward(0.3)
time.sleep(0.5)  # 0.5초 동안 0.3의 속도로 전진

motor1.forward(0.2)
motor2.forward(0.2)
time.sleep(0.5)  # 0.5초 동안 0.2의 속도로 전진

motor1.forward(0.1)
motor2.forward(0.1)
time.sleep(0.5)  # 0.5초 동안 0.1의 속도로 전진

motor1.stop()
motor2.stop()

print("후진")
motor1.backward(0.4)
motor2.backward(0.4)
time.sleep(3)  # 3초 동안 후진

motor1.backward(0.3)
motor2.backward(0.3)
time.sleep(0.5)  # 0.5초 동안 0.3의 속도로 후진

motor1.backward(0.2)
motor2.backward(0.2)
time.sleep(0.5)  # 0.5초 동안 0.2의 속도로 후진

motor1.backward(0.1)
motor2.backward(0.1)
time.sleep(0.5)  # 0.5초 동안 0.1의 속도로 후진

motor1.stop()
motor2.stop()

print("좌회전")
motor1.backward(0.4)
motor2.forward(0.4)
time.sleep(3)  # 3초 동안 좌회전

motor1.backward(0.3)
motor2.forward(0.3)
time.sleep(0.5)  # 0.5초 동안 0.3의 속도로 좌회전

motor1.backward(0.2)
motor2.forward(0.2)
time.sleep(0.5)  # 0.5초 동안 0.2의 속도로 좌회전

motor1.backward(0.1)
motor2.forward(0.1)
time.sleep(0.5)  # 0.5초 동안 0.1의 속도로 좌회전

motor1.stop()
motor2.stop()

print("처음 위치로") # 좌회전 한 시간만큼 우회전해서 처음 위치로 이동
motor1.forward(0.4)
motor2.backward(0.4)
time.sleep(3)  # 3초 동안 우회전

motor1.forward(0.3)
motor2.backward(0.3)
time.sleep(0.5)  # 0.5초 동안 0.3의 속도로 우회전

motor1.forward(0.2)
motor2.backward(0.2)
time.sleep(0.5)  # 0.5초 동안 0.2의 속도로 우회전

motor1.forward(0.1)
motor2.backward(0.1)
time.sleep(0.5)  # 0.5초 동안 0.1의 속도로 우회전

print("우회전")
motor1.forward(0.4)
motor2.backward(0.4)
time.sleep(3)  # 3초 동안 우회전

motor1.forward(0.3)
motor2.backward(0.3)
time.sleep(0.5)  # 0.5초 동안 0.3의 속도로 우회전

motor1.forward(0.2)
motor2.backward(0.2)
time.sleep(0.5)  # 0.5초 동안 0.2의 속도로 우회전

motor1.forward(0.1)
motor2.backward(0.1)
time.sleep(0.5)  # 0.5초 동안 0.1의 속도로 우회전

print("처음 위치로") # 우회전 한 시간만큼 좌회전해서 처음 위치로 이동
motor1.backward(0.4)
motor2.forward(0.4)
time.sleep(3)  # 3초 동안 좌회전

motor1.backward(0.3)
motor2.forward(0.3)
time.sleep(0.5)  # 0.5초 동안 0.3의 속도로 좌회전

motor1.backward(0.2)
motor2.forward(0.2)
time.sleep(0.5)  # 0.5초 동안 0.2의 속도로 좌회전

motor1.backward(0.1)
motor2.forward(0.1)
time.sleep(0.5)  # 0.5초 동안 0.1의 속도로 좌회전

motor1.stop()
motor2.stop()