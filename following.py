import cv2
import time
from yolov8 import YOLOv8
from picamera2 import Picamera2, Preview

# Initialize FPS tracking
pTime = 0
TURN_MIN_VALUE = 30
TURN_MAX_VALUE = 160

DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

# Initialize YOLOv8 object detector
model_path = "/home/kartriders/Documents/kart/yolo8n_OD/models/best.onnx"
yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)

# Initialize Picamera2
picam2 = Picamera2()

# Create a still configuration and set resolution to 640x480
config = picam2.create_still_configuration()

# Modify the resolution of the main camera sensor to 640x480
config["main"]["size"] = (640, 480)

# Configure the camera with the updated resolution
picam2.configure(config)

# Start the camera
picam2.start()

# Wait for the camera to initialize
time.sleep(2)

# Initialize tracker
tracker = cv2.TrackerCSRT_create()  # You can also try TrackerKCF_create() or others

tracking = False  # Flag to check if we are tracking an object
bbox = None  # Bounding box for the object

while True:
    # Capture frame from camera
    frame = picam2.capture_array()

    # Calculate FPS
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    # If we are not tracking, detect the object
    if not tracking:
        # Object detection using YOLOv8
        boxes, scores, class_ids = yolov8_detector(frame)

        if 0 in class_ids:
            # Get bounding box information for the wheelchair user (class_id == 0)
            for box, class_id in zip(boxes, class_ids):
                if class_id == 0:
                    xmin, ymin, xmax, ymax = box.astype(int)
                    bbox = (xmin, ymin, xmax - xmin, ymax - ymin)

                    # Initialize the tracker with the bounding box
                    tracker.init(frame, bbox)
                    tracking = True
                    print("Tracking wheelchair user...")
                    break
        else:
            print("No wheelchair user detected.")
            continue
    else:
        # Update the tracker
        success, bbox = tracker.update(frame)

        if success:
            # Tracker successfully tracked the object
            xmin, ymin, w, h = [int(v) for v in bbox]
            xmax = xmin + w
            ymax = ymin + h
            box_center_x = xmin + w // 2
            box_center_y = ymin + h // 2

            # Calculate the center of the frame
            frame_h, frame_w, _ = frame.shape
            frame_center_x = frame_w // 2
            frame_center_y = frame_h // 2

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

            cv2.imshow("Detected Objects", frame)
        else:
            # If tracking fails, we need to re-detect the object
            print("Tracking lost. Re-detecting object...")
            tracking = False

    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the camera and close all windows
picam2.stop()
cv2.destroyAllWindows()
