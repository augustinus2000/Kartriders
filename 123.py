import torch
import cv2
from ultralytics import YOLO
import time
import numpy as np

GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
CONFIDENCE_THRESHOLD = 0.6

# Set device
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# Load model
model = YOLO('/home/junhyeok/document/KART/runs/detect/train2/weights/best.pt').to(device)

# Initialize recognition state variables
initial_user_image = None
initial_user_kp = None
initial_user_box = None
initial_recognition_done = False
initial_img_save = None


#특징 추출
detector= cv2.ORB_create()
matcher=cv2.BFMatcher(cv2.NORM_HAMMING)

# Start camera
cap = cv2.VideoCapture(0)

start_time = time.time()


while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    #초기 이미지 저장
    if initial_img_save == None :
        save_time = time.time() - start_time

        if save_time >= 5 :
            cv2.imwrite("/home/junhyeok/document/KART/save/img.jpg",frame)

            initial_img_save = True

    #yolo 사용한 프레임 받음
    #detection = model(frame)[0]

    #받은 프레임의 바운딩 박스 내에서 [xmin, ymin, xmax, ymax, confidence, class_id]을 가져오는 data로 for문
    #초기 인식이 되지 않았을 때, if문
    if initial_recognition_done == False :
        #초기 이미지 불러오기
        img1 = cv2.imread("/home/junhyeok/document/KART/save/img.jpg")
        #현재 영상 프레임 불러오기
        img2 = cv2.imread("/home/junhyeok/document/KART/save/carts.png")
        ##############
        detection = model(img2)[0]
        for data in detection.boxes.data.tolist():
            xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
            roi=img2[ymin:ymax, xmin:xmax]
            cv2.imwrite('/home/junhyeok/document/KART/save/crop_img.jpg', roi)
            roi_1=cv2.imread('/home/junhyeok/document/KART/save/crop_img.jpg')
###########################################
        #그레이스케일변환
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(roi_1, cv2.COLOR_BGR2GRAY)
        #특징점과 디스크립터 계산
        kp1, desc1 = detector.detectAndCompute(gray1, None)
        kp2, desc2 = detector.detectAndCompute(gray2, None)

        #두 이미지간 특징 매칭 수행
        matches=matcher.match(desc1,desc2)
        #거리 작은 순으로 정렬
        matches = sorted(matches, key=lambda x: x.distance)
        good_matches = matches[:50]


    res = cv2.drawMatches(img1, kp1, roi_1, kp2, good_matches, None, flags = cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

    #cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
    #cv2.putText(frame, str(class_id) + ' ' + str(round(confidence, 2)) + '%', (xmin, ymin), cv2.FONT_ITALIC, 1, WHITE, 2)



    # 화면 표시
    cv2.imshow("Wheelchair Detection and Tracking", res)

    # Press 'q' to exit
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()