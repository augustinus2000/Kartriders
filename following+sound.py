import cv2
import time
from ultralytics import YOLO

import pygame
import threading
#import serial # pip install pyserial

# Initialize FPS tracking
pTime = 0
TURN_MIN_VALUE = 100
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

PWM_SCALE = [51, 77] # 아두이노에서는 pwm 제어 0~255 범위를 가진다.
# 51은 20%의 출력, 최대 속도를 20%의 출력으로 맞춰놓았다. 이건 카트 속도보고 조정하기!!

# Bluetooth 포트 설정
bluetooth_port = 'COM5'  # 실제 포트에 맞게 수정해야 함
baudrate = 9600 # 이거 아두이노에 설정한 값이랑 같아야 한다.

# 시리얼 연결 설정
#ser = serial.Serial(bluetooth_port, baudrate) # 블루투스 장치와 연결


# 명령 전송
def send_command(char, num=0): # send_command는 블루투스를 통해 명령을 아두이노로 전송하는 함수.\
    # num의 기본 값은 0으로 설정해서 정지할 때는 pwm 값을 아두이노로 안 보내도 된다.
    signal = f"{char},{num}\n" # 문자와 숫자를 쉼표로 구분해서 전송. 
    ser.write(signal.encode()) # command 문자열을 바이트 형식으로 변환하고 ser을 통해 전송.
    print(f"명령 '{signal}' 전송됨")

def RangeCalc(In, in_max, in_min, out_max, out_min): # pwm 계산 함수
    # mapped_value = (x_clipped - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    x = min(max(In, in_min), in_max)
    mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    mapped_value = max(min(mapped_value, out_max), out_min) # 한번 더 클리핑해서 pwm 범위 못 벗어나게 막음.
    return mapped_value

# Initialize the webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("웹캠 초기화 실패. 종료합니다.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize YOLOv8 object detector
model_path = "/home/soeunan/KART/best.pt"
model = YOLO(model_path)

# Initialize OpenCV Tracker
tracker = cv2.TrackerCSRT_create()  # Use CSRT tracker
tracking = False  # Flag to check if we are tracking an object
bbox = None  # Bounding box for the object
frame_count = 0  # Frame counter for periodic YOLO updates

def calculate_iou(box1, box2):
    """두 박스의 IoU 계산."""
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    # 두 박스의 우측 하단 좌표 계산
    x1_end, y1_end = x1 + w1, y1 + h1
    x2_end, y2_end = x2 + w2, y2 + h2

    # 교차 영역의 좌상단과 우하단 좌표 계산
    x_intersect = max(x1, x2)
    y_intersect = max(y1, y2)
    x_intersect_end = min(x1_end, x2_end)
    y_intersect_end = min(y1_end, y2_end)

    # 교차 영역의 넓이 계산
    intersection_width = max(0, x_intersect_end - x_intersect)
    intersection_height = max(0, y_intersect_end - y_intersect)

    intersection_area = intersection_width * intersection_height

    # 두 박스의 넓이 계산
    area1 = w1 * h1
    area2 = w2 * h2

    # IOU 계산
    union_area = area1 + area2 - intersection_area
    iou = intersection_area / union_area if union_area > 0 else 0

    return iou

# Function to handle object detection and tracking in parallel (for performance)
def detect_and_track(frame):
    global bbox, tracking, tracker, frame_count

    frame_count += 1

    if not tracking or frame_count % 10 == 0:  # Use YOLO every 10 frames or when not tracking
        results = model.predict(source=frame, conf=0.7, iou=0.7, save=False, verbose=False)
        detections = results[0]

        if len(detections) > 0:
            boxes = detections.boxes.xyxy.cpu().numpy()
            class_ids = detections.boxes.cls.cpu().numpy()

            # Flag to track the first detected wheelchair
            detected = False

            for box, class_id in zip(boxes, class_ids):
                if int(class_id) == 0:  # Check for wheelchair user class
                    xmin, ymin, xmax, ymax = map(int, box)
                    detected_bbox = (xmin, ymin, xmax - xmin, ymax - ymin)

                    if tracking:
                        # Calculate IoU between YOLO and tracker bbox
                        iou = calculate_iou(detected_bbox, bbox)
                        if iou < 0.7:  # IoU threshold for reinitialization (improve threshold)
                            print("Low IoU. Gradually reinitializing tracker with YOLO detection.")
                            tracker = cv2.TrackerCSRT_create()  # Create a new tracker
                            tracker.init(frame, detected_bbox)  # Reinitialize tracker
                            bbox = detected_bbox
                        else:
                            print("IoU sufficient. Keeping tracker bbox.")
                    else:
                        tracker.init(frame, detected_bbox)
                        bbox = detected_bbox
                        print("Tracking initialized.")

                    tracking = True
                    detected = True
                    break  # Only track the first detected wheelchair

            if not detected:
                print("No wheelchair user detected.") 
        else:
            print("No detections found.")

    if tracking:
        success, updated_bbox = tracker.update(frame)
        if success:
            bbox = updated_bbox
            xmin, ymin, w, h = map(int, bbox)
            xmax = xmin + w
            ymax = ymin + h

            # Calculate the center of the frame
            box_center_x = xmin + w // 2
            box_center_y = ymin + h // 2
            frame_h, frame_w, _ = frame.shape
            frame_center_x = frame_w // 2
            frame_center_y = frame_h // 2
            turn_direc = box_center_x - frame_center_x
            distance_scale = frame_h - ymax

            # Control movements based on the detected position
            if abs(turn_direc) > TURN_MIN_VALUE : # 휠체어가 화면 중심에서 벗어남
                
                if turn_direc > 0 : # 휠체어가 왼쪽에 있으면
                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    #send_command('l', pwm) # 아두이노로 좌회전 신호와 pwm 값을 보냄, 쉼표로 구분 함!!!!
                    print("좌회전")
                    print(f"PWM: {pwm}") # pwm 값이 0~51 사이를 잘 전달하는지 확인할려고 적어둠

                else : # 휠체어가 오른쪽에 있으면
                    pwm = RangeCalc(abs(turn_direc), TURN_MAX_VALUE, TURN_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    #send_command('r', pwm) # 아두이노로 우회전 신호와 pwm 값을 보냄
                    print("우회전")
                    print(f"PWM: {pwm}")

            else : # 휠체어가 화면 중심에 잘 있을 때
                if distance_scale < DISTANCE_MIN_VALUE : # 거리가 너무 가까우면
                    pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    #send_command('b', pwm) # 후진
                    print("후진")
                    print(f"PWM: {pwm}") 

                elif distance_scale > DISTANCE_MIN_VALUE : # 거리가 너무 멀면
                    pwm = RangeCalc(abs(distance_scale), DISTANCE_MAX_VALUE, DISTANCE_MIN_VALUE, PWM_SCALE[1], PWM_SCALE[0])
                    #send_command('f', pwm) # 전진
                    print("전진")
                    print(f"PWM: {pwm}")

                else : # 정확한 기준 거리에 도착하면
                    #send_command('s') # 정지
                    print("정지")

            # UI
            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 0), cv2.FILLED)
            cv2.circle(frame, (box_center_x, box_center_y), 5, (255, 0, 0), cv2.FILLED)  # Bounding box center point
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)  # Bounding box
            cv2.putText(frame, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.line(frame, (box_center_x, box_center_y), (frame_center_x, frame_center_y), (255 ,0, 255), 2)
        else:
            print("Tracking lost. Re-detecting object...")
            mp3_file = '/home/soeunan/KART/4.mp3'
            threading.Thread(target=play_sound, args=(mp3_file,), daemon=True).start()
            wait_with_time(5)
            #send_command('s')
            tracking = False

    return frame




#--------------------------------------------------

#음성 재생
def play_sound(mp3_file):
    pygame.mixer.init()
    pygame.mixer.music.load(mp3_file)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy(): #play 중이면
        pygame.time.Clock().tick(10) #루프가 10초에 한 번씩 돌게 함

#음성 재생동안 코드가 진행되지 않게 대기하는 함수        
def wait_with_time(duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        time.sleep(0.01)

#초음파 센서 계산 함수
def sensor_data():
    while True:
        try:
            get_data = ser.readline().decode()  # 아두이노 데이터 읽기
            if get_data:
                num = int(get_data)
                print(f"초음파센서거리: {num}")
                if num <= 100:
                    threading.Thread(
                        target=play_sound, args=('/home/soeunan/KART/back.mp3',), daemon=True).start()
        except ValueError:
            print("Invalid data received.")  # 데이터 형식 오류 처리
        except Exception as e:
            print(f"Error: {e}")
            break
#--------------------------------------------------
sound_time_2min = time.time()

mp3_file = '/home/soeunan/KART/intro.mp3'
play_sound(mp3_file)
mp3_file = '/home/soeunan/KART/ready.mp3'
play_sound(mp3_file)

arduino_thread = threading.Thread(target=sensor_data, daemon=True)
arduino_thread.start()
#--------------------------------------------------
# Main loop
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

    # Perform object detection and tracking
    frame = detect_and_track(frame)

    # Display the frame
    cv2.imshow("Detected Objects", frame)
    print(fps)

    # Press key 'q' to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        #send_command('s')
        break
    elif time.time() - sound_time_2min >= 120 :
        mp3_file = '/home/soeunan/KART/2.mp3'
        threading.Thread(target=play_sound, args=(mp3_file,), daemon=True).start()
        sound_time_2min = time.time()
    
    
        



try:
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            #send_command('s')
            break
except KeyboardInterrupt: # Ctrl + C 눌러서 정지했을 때
    #send_command('s')
    print("\n프로그램이 중단.")
finally: # q 또는 Ctrl + C 눌러서 정지해도 반드시 실행되는 코드
    #send_command('s')
    #ser.close()
    print("블루투스 연결이 종료.")

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()