from pyfirmata import Arduino, PWM, OUTPUT, util

import math
import time

speed = 0.4

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

motor1 = Motor(board, 5, 6, 2, 4)
motor2 = Motor(board, 9, 10, 7, 8)

it.start()

print("앞으로 가")
motor1.forward(speed)
motor2.forward(speed)

time.sleep(5)  # 5초 대기

motor1.stop()
motor2.stop()

print("뒤로 가")
motor1.backward(speed)
motor2.backward(speed)

time.sleep(5)  # 5초 대기

motor1.stop()
motor2.stop()