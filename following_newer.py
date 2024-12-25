import numpy as np
import cv2
from ultralytics import YOLO
from short import Sort
import torch
import torchvision.ops as ops

# IOU 계산 함수
def calculate_iou(box1, box2):
    x1, y1, x2, y2 = box1
    x3, y3, x4, y4 = box2
    xi1 = max(x1, x3)
    yi1 = max(y1, y3)
    xi2 = min(x2, x4)
    yi2 = min(y2, y4)
    inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
    box1_area = (x2 - x1) * (y2 - y1)
    box2_area = (x4 - x3) * (y4 - y3)
    union_area = box1_area + box2_area - inter_area
    return inter_area / union_area if union_area > 0 else 0

# Constants
TURN_MIN_VALUE = 30 # 휠체어가 좌우로 회전하는 최소 기준값
DISTANCE_MIN_VALUE = 30 # 휠체어가 너무 가까이 왔을 때 정지하는 기준
DISTANCE_MAX_VALUE = 120 # 휠체어가 너무 멀리 갔을 때 전진하는 기준
TRACK_LOSS_TOLERANCE = 50  # 추적 대상이 사라질 때 허용되는 최대 프레임 수

# 웹캠 초기화 및 설정
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("웹캠 초기화 실패. 종료합니다.")
    exit()

# 웹캠 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# YOLOv8n 시작
model_path = "/home/junhyeok/document/KART2/tracker-yolov8-sort-python/best.pt"
model = YOLO(model_path)

# Initialize tracker with custom settings
tracker = Sort(max_age=15, iou_threshold=0.3) # max_age=15: 추적이 지속될 수 있는 최대 프레임 수, iou_threshold=0.3: 두 객체의 ioU가 이 값 이상일 때 추적을 유지

# Variables for tracking
tracking_id = None # 현재 추적 중인 휠체어의 ID를 저장
tracking_bbox = None # 추적 중인 휠체어의 바운딩 박스 저장
tracking_loss_count = 0 # 추적 실패 횟수 저장

if __name__ == "__main__":
    while cap.isOpened(): # 웹캠이 열려있는 동안 반복
        status, frame = cap.read()
        if not status:
            break # 웹캠에서 프레임을 읽음. 읽지 못하면 루프를 종료

        # YOLOv8n 모델을 사용해 현재 프레임에서 객체를 감지
        results = model(frame, stream=True)

        for res in results:
            filtered_indices = np.where(res.boxes.conf.cpu().numpy() > 0.3)[0] # 신뢰도가 0.3 이상인 객체들만 필터링
            boxes = res.boxes.xyxy.cpu().numpy()[filtered_indices] # 감지된 객체의 바운딩 박스를 가져옴
            confidences = res.boxes.conf.cpu().numpy()[filtered_indices] # 각 객체의 신뢰도 점수
            class_ids = res.boxes.cls.cpu().numpy()[filtered_indices].astype(int) # 각 객체의 클래스 ID (휠체어는 0번, 사람은 1번)

            if len(boxes) > 0: # NMS(Non-Maximum Suppression)을 적용하여 겹치는 박스를 제거 (여러개의 중복된 박스가 감지되었을 때, 가장 신뢰도가 높은 박스만을 남기고 나머지는 제거)
                keep_indices = ops.nms(torch.tensor(boxes), torch.tensor(confidences), iou_threshold=0.3) # IoU가 30% 이상 겹치는 박스는 제거
                filtered_boxes = boxes[keep_indices.numpy()].astype(int)
                filtered_classes = class_ids[keep_indices.numpy()]
            else:
                filtered_boxes = np.array([])
                filtered_classes = np.array([])

            if len(filtered_boxes) > 0: # filtered_boxes에 해당하는 객체들을 추적
                tracks = tracker.update(filtered_boxes)
                tracks = tracks.astype(int)
               
                # 휠체어만 추적
                wheelchair_tracks = [] # 추적된 휠체어들만 모은 리스트
                for t in tracks:
                    matching_indices = np.where(filtered_boxes[:, 0] == t[0])[0]
                    if len(matching_indices) > 0:
                        matching_index = matching_indices[0]
                        class_id = filtered_classes[matching_index]
                        if class_id == 0:  # 휠체어 클래스 ID만 추적
                            wheelchair_tracks.append(t)

                # Update tracking ID and bounding box
                if len(wheelchair_tracks) > 0:
                    if tracking_id is None:
                        # 초기 설정: 화면 중앙에 가장 가까운 휠체어 선택
                        frame_h, frame_w, _ = frame.shape
                        frame_center_x, frame_center_y = frame_w // 2, frame_h // 2
                        # ID 기준으로 휠체어를 정렬 (가장 작은 ID를 먼저 선택)
                        sorted_wheelchairs = sorted(
                            wheelchair_tracks,
                            key=lambda w: (abs((w[0] + w[2]) // 2 - frame_center_x) + abs((w[1] + w[3]) // 2 - frame_center_y), w[-1])
                        )
                        # 가장 작은 ID를 가진 휠체어를 추적
                        tracking_id = sorted_wheelchairs[0][-1] # 가장 작은 ID
                        tracking_bbox = sorted_wheelchairs[0][:4]
                        tracking_loss_count = 0
                    else:
                        # 현재 추적 중인 ID와 가장 유사한 휠체어 선택
                        best_match = None
                        best_iou = 0
                        for track in wheelchair_tracks:
                            iou = calculate_iou(tracking_bbox, track[:4])
                            if track[-1] == tracking_id and iou > 0.3:
                                best_match = track
                                best_iou = iou
        
                        if best_match is not None:
                            tracking_bbox = best_match[:4]
                            tracking_loss_count = 0
                        else:
                            # ID와 일치하지 않을 경우, 가장 작은 ID를 가진 휠체어 선택
                            sorted_wheelchairs = sorted(
                                wheelchair_tracks,
                                key=lambda w: w[-1]  # ID 기준으로 정렬 (가장 작은 ID를 선택)
                            )
                            tracking_id = sorted_wheelchairs[0][-1]
                            tracking_bbox = sorted_wheelchairs[0][:4]
                            tracking_loss_count = 0
                else:
                    # 휠체어가 감지되지 않을 경우
                    tracking_loss_count += 1
                    if tracking_loss_count > TRACK_LOSS_TOLERANCE:  # 추적 대상이 50프레임보다 더 오랫동안 화면에서 사라진다면,
                        # 추적 대상 초기화
                        tracking_id = None
                        tracking_bbox = None

                # 추적 시각화 부분
                for xmin, ymin, xmax, ymax, track_id in tracks.astype(int):
                    if tracking_id is not None and track_id == tracking_id:
                        color = (0, 255, 0)
                    else:
                        color = (255, 0 ,0)    
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                    cv2.putText(frame, f"ID: {track_id}", (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                if tracking_bbox is not None:
                    xmin, ymin, xmax, ymax = map(int, tracking_bbox)
                    box_center_x, box_center_y = xmin + (xmax - xmin) // 2, ymin + (ymax - ymin) // 2
                    frame_h, frame_w, _ = frame.shape
                    frame_center_x, frame_center_y = frame_w // 2, frame_h // 2

                    turn_direc = box_center_x - frame_center_x
                    distance_scale = frame_h - ymax

                    if abs(turn_direc) > TURN_MIN_VALUE:
                        print("우회전" if turn_direc > 0 else "좌회전")
                    else:
                        if distance_scale < DISTANCE_MIN_VALUE:
                            print("정지")
                        elif distance_scale > DISTANCE_MAX_VALUE:
                            print("전진")
                        else:
                            print("후진")

                    # Draw tracking line
                    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 0), cv2.FILLED)
                    cv2.circle(frame, (box_center_x, box_center_y), 5, (255, 0, 0), cv2.FILLED)
                    cv2.line(frame, (box_center_x, box_center_y), (frame_center_x, frame_center_y), (255, 0, 255), 2)

        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows() 