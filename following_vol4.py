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
if not cap.isOpened():
    print("웹캠 초기화 실패. 종료합니다.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize YOLOv8 object detector
model_path = "/home/junhyeok/document/KART/models/best.pt"
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
            cv2.line(frame, (box_center_x, box_center_y), (frame_center_x, frame_center_y), (255 ,0, 255), 2)
        else:
            print("Tracking lost. Re-detecting object...")
            tracking = False

    return frame


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

    # Press key 'q' to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
