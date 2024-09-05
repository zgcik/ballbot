import cv2
import numpy as np
from copy import deepcopy
from ultralytics import YOLO
from ultralytics.utils import ops

class Detector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.target_dim = 0.0342 * 2

    def get_bboxes(self, img):
        # making predictions
        preds = self.model.predict(img, verbose=False)

        # get bounding box and class label for target(s) detected
        bboxes = []
        coords = []
        for p in preds:
            boxes = p.boxes
            if boxes is None:
                return []
            for box in boxes:
                b_cord = box.xywh[0]
                b_label = box.cls
                bboxes.append([p.names[int(b_label)], np.asarray(b_cord)])

        return bboxes

    def detect(self, frame):
        bboxes = self.get_bboxes(frame)
        img_out = deepcopy(frame)

        for bbox in bboxes:
            xyxy = ops.xywh2xyxy(bbox[1])
            x1 = int(xyxy[0])
            y1 = int(xyxy[1])
            x2 = int(xyxy[2])
            y2 = int(xyxy[3])

            # draw yellow bounding box
            img_out = cv2.rectangle(img_out, (x1, y1), (x2, y2), (0, 255, 255), thickness=2)
        
        return bboxes, img_out
    
