import cv2
import time
from ultralytics import YOLO

# Initialize FPS tracking
pTime = 0
TURN_MIN_VALUE = 30
DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

# Initialize the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize YOLOv8 object detector
model_path = "/home/junhyeok/document/KART/models/best.pt"
model = YOLO(model_path)

# Initialize OpenCV Tracker
tracker = cv2.TrackerCSRT_create()  # Use CSRT tracker
tracking = False  # Flag to check if we are tracking an object
bbox = None  # Bounding box for the object

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

    # If not tracking, use YOLO for object detection
    if not tracking:
        results = model.predict(source=frame, conf=0.7, iou=0.7, save=False, verbose=False)
        detections = results[0]
       
        if len(detections) > 0:
            boxes = detections.boxes.xyxy.cpu().numpy()
            class_ids = detections.boxes.cls.cpu().numpy()

            for box, class_id in zip(boxes, class_ids):
                if int(class_id) == 0:  # Check for wheelchair user class
                    xmin, ymin, xmax, ymax = map(int, box)
                    bbox = (xmin, ymin, xmax - xmin, ymax - ymin)

                    # Initialize Tracker with detected bounding box
                    tracker.init(frame, bbox)
                    tracking = True
                    print("Tracking initialized.")
                    break
        else:
            print("No wheelchair user detected.")
            
    else:
        # Update tracker
        success, bbox = tracker.update(frame)

        if success:
            # Tracker successfully tracked the object
            xmin, ymin, w, h = map(int, bbox)
            xmax = xmin + w
            ymax = ymin + h
            box_center_x = xmin + w // 2
            box_center_y = ymin + h // 2
            box_center_tu = (box_center_x, box_center_y)

            # Calculate the center of the frame
            frame_h, frame_w, _ = frame.shape
            frame_center_x = frame_w // 2
            frame_center_y = frame_h // 2
            frame_center_tu = (int(frame_center_x), int(frame_center_y))

            # Calculate turn direction based on horizontal offset
            turn_direc = box_center_x - frame_center_x
            distance_scale = frame_h - ymax

            if abs(turn_direc) > TURN_MIN_VALUE:
                if turn_direc > 0:
                    print("우회전")
                else:
                    print("좌회전")
            else:
                if distance_scale < DISTANCE_MIN_VALUE:
                    print("정지")
                elif distance_scale > DISTANCE_MIN_VALUE:
                    print("전진")
                else:
                    print("후진")

            # UI
            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 0), cv2.FILLED)
            cv2.circle(frame, (box_center_x, box_center_y), 5, (255, 0, 0), cv2.FILLED)  # Bounding box center point
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)  # Bounding box
            cv2.putText(frame, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.line(frame, box_center_tu, frame_center_tu, (255 ,0, 255), 2) #서로의 중심점 연결선
        else:
            # If tracking fails, reset tracking
            print("Tracking lost. Re-detecting object...")
            tracking = False

    cv2.imshow("Detected Objects", frame)

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

