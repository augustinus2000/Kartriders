import numpy as np
import cv2
from ultralytics import YOLO
from short import Sort

TURN_MIN_VALUE = 30
DISTANCE_MIN_VALUE = 30
DISTANCE_MAX_VALUE = 120

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    model = YOLO("best.pt")

    tracker = Sort()

    while cap.isOpened():
        status, frame = cap.read()

        if not status:
            break

        #캠 프레임의 이미지 크기와 좌표
        frame_h, frame_w, _ = frame.shape
        frame_center_x = frame_w // 2
        frame_center_y = frame_h // 2
        frame_center = [frame_center_x, frame_center_y]
        frame_center_tu = (int(frame_center_x), int(frame_center_y))

        results = model(frame, stream=True)

        for res in results:
            filtered_indices = np.where(res.boxes.conf.cpu().numpy() > 0.2)[0]
            boxes = res.boxes.xyxy.cpu().numpy()[filtered_indices].astype(int)
            class_ids = res.boxes.cls.cpu().numpy().astype(int)
            if len(boxes) > 0:
                for i, class_id in enumerate(class_ids):
                    if class_id == 0:
                        tracks = tracker.update(boxes)
                        tracks = tracks.astype(int)
                        if len(tracks) > 0:
                            xmin, ymin, xmax, ymax, track_id = tracks[-1]
                            
                            w=xmax-xmin
                            h=ymax-ymin
                            box_center_x = xmin + w // 2
                            box_center_y = ymin + h // 2
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

                            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 0), cv2.FILLED)
                            cv2.circle(frame, (box_center_x, box_center_y), 5, (255, 0, 0), cv2.FILLED)  # Bounding box center point
                            cv2.putText(frame, f"Distance :  {distance_scale}", (20, 180), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                            cv2.putText(frame, f"Turn Offset Value :  {turn_direc}", (20, 80), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
                            cv2.line(frame, (box_center_x, box_center_y), (frame_center_x, frame_center_y), (255 ,0, 255), 2)
                            cv2.putText(img=frame, text=f"Id: {track_id}", org=(xmin, ymin - 10), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=2, color=(0, 255, 0), thickness=2)
                            cv2.rectangle(img=frame, pt1=(xmin, ymin), pt2=(xmax, ymax), color=(0, 255, 0), thickness=2)


        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()