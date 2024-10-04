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
        xyxys = []
        for p in preds:
            boxes = p.boxes
            if boxes is None:
                return []
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                xyxys.append(([x1,y1],[x2,y2]))

                b_cord = box.xywh[0]
                b_label = box.cls
                bboxes.append([p.names[int(b_label)], np.asarray(b_cord)])

        return bboxes, xyxys

    def detect(self, frame):
        bboxes, xyxys = self.get_bboxes(frame)
        img_out = deepcopy(frame)
        

        for bbox in bboxes:
            xyxy = ops.xywh2xyxy(bbox[1])
            x1 = int(xyxy[0])
            y1 = int(xyxy[1])
            x2 = int(xyxy[2])
            y2 = int(xyxy[3])

            # draw yellow bounding box
            # img_out = cv2.rectangle(img_out, (x1, y1), (x2, y2), (0, 255, 255), thickness=2)
        xyxys = self.flip_xyxy_180(xyxys, 640, 480)
        return bboxes, img_out, xyxys
    def flip_xyxy_180(self,bboxes, image_width, image_height):
        # Unpack the original bounding box
        xyxys = []
        for bbox in bboxes:
            x1, y1 = bbox[0]
            x2, y2 = bbox[1]
            
            # Flip the coordinates
            new_x1 = image_width - x2
            new_y1 = image_height - y2
            new_x2 = image_width - x1
            new_y2 = image_height - y1

            xyxys.append(([new_x1, new_y1], [new_x2, new_y2]))
        
        # Return the flipped bounding box
        return xyxys
    
