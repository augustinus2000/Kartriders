import cv2
from ultralytics import YOLO

# 학습된 best.pt 모델 로드
model = YOLO('runs/train/yolo_trained_model/weights/best.pt')

# 웹캠 열기 (기본 웹캠은 0번 인덱스)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("웹캠에서 프레임을 읽어올 수 없습니다.")
        break

    # 프레임에서 객체 감지
    results = model(frame)

    # 감지된 결과를 시각화하여 이미지에 그리기
    annotated_frame = results[0].plot()  # 시각화된 프레임을 얻음

    # 화면에 결과 출력
    cv2.imshow("YOLOv8 Object Detection", annotated_frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

# 웹캠과 창 닫기
cap.release()
cv2.destroyAllWindows()
