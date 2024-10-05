import cv2
import numpy as np
from copy import deepcopy
from ultralytics import YOLO
from ultralytics.utils import ops
import typing

Detection = typing.Tuple[str, np.ndarray, np.ndarray]


class Detector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.target_dim = 0.0342 * 2

    def get_detections(self, img):
        # making predictions
        preds = self.model.predict(img, verbose=False)

        # get bounding box and class label for target(s) detected
        detections = []
        for p in preds:
            boxes = p.boxes
            if boxes is None:
                return []
            for box in boxes:
                b_label = box.cls

                detections.append(
                    (
                        p.names[int(b_label)],
                        np.asarray(box.xywh[0]),
                        np.asarray(box.xyxy[0]),
                    )
                )

        return detections

    def detect(self, frame) -> list[Detection]:
        detections = self.get_detections(frame)
        return detections

    def flip_xyxy_180(self, xyxy, image_dims):
        # Unpack the original bounding box
        x1, y1, x2, y2 = xyxy

        # Flip the coordinates
        image_height, image_width = image_dims
        new_x1 = image_width - x2
        new_y1 = image_height - y2
        new_x2 = image_width - x1
        new_y2 = image_height - y1

        # Return the flipped bounding box
        return [new_x1, new_y1], [new_x2, new_y2]
