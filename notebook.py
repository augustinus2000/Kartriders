import serial # pip install pyserial
import time

# Bluetooth 포트 설정
bluetooth_port = 'COM3'  # 실제 포트에 맞게 수정해야 함
baudrate = 9600

# 시리얼 연결 설정
ser = serial.Serial(bluetooth_port, baudrate)

# 명령 전송
def send_command(command):
    ser.write(command.encode())
    print(f"명령 '{command}' 전송됨")

# 명령 테스트
try:
    while True:
        user_input = input("전송할 명령 (f: 전진, b: 후진, l: 좌회전, r: 우회전, s: 정지, q: 종료): ")
        if user_input in ['f', 'b', 'l', 'r', 's']:
            send_command(user_input)
        elif user_input == 'q':
            print("종료.")
            break
        else:
            print("잘못된 명령어.")
except KeyboardInterrupt:
    print("\n프로그램이 중단.")
finally:
    ser.close()
    print("블루투스 연결이 종료.")
