from pyfirmata import Arduino, PWM, OUTPUT, util

import math
import time

class Motor:
    def __init__(self, board, Lpwm_pin, Rpwm_pin, Len_pin, Ren_pin):
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

# Board settings
bluetooth_port = 'COM3'
board = Arduino(bluetooth_port)
it = util.Iterator(board)

motor1 = Motor(board, 5, 6)
motor2 = Motor(board, 9, 10)

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

print("블루투스 통신 종료")
board.exit()