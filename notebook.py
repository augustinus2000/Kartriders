import serial # pip install pyserial
import time

# Bluetooth 포트 설정
bluetooth_port = 'COM3'  # 실제 포트에 맞게 수정해야 함
# 제 노트북은 COM3 였어요. 장치관리자 들어가서 확인할 수 있어요.
baudrate = 9600 # 이거 아두이노에 설정한 값이랑 같아야 한다.

# 시리얼 연결 설정
ser = serial.Serial(bluetooth_port, baudrate) # 블루투스 장치와 연결

# 명령 전송
def send_command(command): # send_command는 블루투스를 통해 명령을 아두이노로 전송하는 함수.
    ser.write(command.encode()) # command 문자열을 바이트 형식으로 변환하고 ser을 통해 전송.
    print(f"명령 '{command}' 전송됨")

# 명령 테스트
try:
    while True:
        user_input = input("전송할 명령 (f: 전진, b: 후진, l: 좌회전, r: 우회전, s: 정지, q: 종료): ").lower()
        if user_input in ['f', 'b', 'l', 'r', 's']:
            send_command(user_input)
        elif user_input == 'q': # q 누르면 정지
            print("종료.")
            break
        else:
            print("잘못된 명령어.")
except KeyboardInterrupt: # Ctrl + C 눌러서 정지했을 때
    print("\n프로그램이 중단.")
finally: # q 또는 Ctrl + C 눌러서 정지해도 반드시 실행되는 코드
    ser.close()
    print("블루투스 연결이 종료.")
