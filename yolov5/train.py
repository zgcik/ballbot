from ultralytics import YOLO

model = YOLO("model.pt")
results = model.train(data="data/data.yaml", epochs=100, imgsz=640, device=0, workers=0)
