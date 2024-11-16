import cv2
from yolov8 import YOLOv8
import math
import time

pTime = 0
TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

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
    

                turn_direc = frame_center_x - box_center[0]
                distance_scale = frame_h - ymax

                if abs(turn_direc) > TURN_MIN_VALUE :
                
                    if turn_direc > 0 :
                        print("좌회전")
                    else :
                        print("우회전")

                else :
                    if distance_scale < DISTANCE_MIN_VALUE :
                        print("정지")
                    elif distance_scale > DISTANCE_MIN_VALUE :
                        print("전진")
                    else :
                        print("후진")

            


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
        continue
    

    
        
  

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()