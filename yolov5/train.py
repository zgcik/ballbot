from ultralytics import YOLO


model = YOLO("runs/detect/train7/weights/best.pt")
results = model.train(
    data="../datasets/ball-and-box-detetction-4-yolov5/data.yaml",
    batch=32,
    epochs=100,
    patience=5,
    imgsz=640,
    device=0,
)
