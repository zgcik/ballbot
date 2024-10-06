import time
from typing import Optional, cast
import cv2
import os
import numpy as np

from line import LineDetector

try:
    from picamera2 import Picamera2  # type: ignore

    pi = True
except:
    pi = False


from detector import Detection, Detector


class Camera:
    frame: cv2.typing.MatLike
    debug: Optional[cv2.typing.MatLike]

    def __init__(self, device=0):
        if pi:
            cam = Picamera2()
            # mode = cam.sensor_modes[2]
            # config = cam.create_still_configuration(sensor={"output_size": mode["size"]})
            # cam.configure(config)  # type: ignore
            cam.start()
        else:
            cam = cv2.VideoCapture(device)
        self.cam = cam

        # importing calibration
        script_dir = os.path.dirname(__file__)
        self.int_matrix = np.load(
            os.path.join(script_dir, "calibration", "int_matrix.npy")
        )
        self.dist_matrix = np.load(
            os.path.join(script_dir, "calibration", "dist_matrix.npy")
        )

        self.detector = Detector(os.path.join(script_dir, "model-v2.pt"))
        self.line = LineDetector(self.int_matrix, calibrate=False)

        self.frame = self.get_frame()
        self.debug = None

        self.cam_dim = (4608, 2592)  # gets updated to match actual dims in get_frame
        self.ball_dim = 0.0334225 * 2

    def __undistort_img__(self, frame):
        h, w = frame.shape[:2]
        n_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.int_matrix, self.dist_matrix, (w, h), 1, (w, h)
        )

        undistorted = cv2.undistort(
            frame, self.int_matrix, self.dist_matrix, None, n_camera_matrix
        )
        return undistorted

    def get_frame(self):
        # this outputs in RGB (not BGR like cv2 video capture)
        if pi:
            frame = cast(cv2.typing.MatLike, self.cam.capture_array("main"))  # type: ignore
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = self.cam.read()
        if frame is None:
            return np.array([[[]]])
        # self.frame = self.__undistort_img__(frame)
        self.frame = frame
        if not pi:
            self.debug = frame.copy()
        # cv2.imwrite("test.jpg", self.frame)
        self.cam_dim = self.frame.shape
        return self.frame

    def __est_pose__(self, detection):
        focal_length = self.int_matrix[0][0]
        target_box = detection[1]
        true_dim = self.ball_dim

        # compute target pose based on pixel dims
        pix_h = target_box[3]
        pix_c = target_box[0]
        dis = true_dim / pix_h * focal_length

        x_shift = (self.cam_dim[1] / 2) - pix_c  #### Make sure it looks good 640
        theta = -np.arctan(x_shift / focal_length)

        return dis, theta

    def detect_box(self):
        self.get_frame()
        detections = self.detector.detect(self.frame)

        if len(detections) == 0:
            return

        valid_box_detection = self.get_valid_box_detections(detections)

        if valid_box_detection is None:
            return

        if self.debug is not None:
            x1, y1, x2, y2 = list(map(int, valid_box_detection[2]))
            cv2.rectangle(self.debug, (x1, y1), (x2, y2), (255, 0, 0), 2)

        return valid_box_detection

    def detect_closest(self):
        self.get_frame()
        detections = self.detector.detect(self.frame)

        # bboxes = self.get_valid_detections(bboxes)
        if len(detections) == 0:
            return
        ###Line stuff
        valid_ball_detections = self.get_valid_ball_detections(detections)

        if self.debug is not None:
            for detection in valid_ball_detections:
                x1, y1, x2, y2 = list(map(int, detection[2]))
                cv2.rectangle(self.debug, (x1, y1), (x2, y2), (0, 255, 0), 2)

        dis_min = 6
        theta_min = np.pi
        for i, detection in enumerate(valid_ball_detections):
            dis, theta = self.__est_pose__(detection)
            if i == 0 or dis < dis_min:
                dis_min = dis
                theta_min = theta

        return (dis_min, theta_min)

    # Finds ball detections that have no white line below the bottom of its bbox
    def get_valid_ball_detections(self, detections: list[Detection]) -> list[Detection]:
        mask = self.line.detect(self.frame)
        if not pi:
            cv2.imshow("mask", mask)
            cv2.waitKey(1)
        valid_bboxes = []

        for detection in detections:
            if detection[0] != "ball":
                continue
            x1, y1, x2, y2 = list(map(int, detection[2]))
            image_below = mask[y2:, x1:x2]
            line_exists = np.any(image_below)
            if not line_exists:
                valid_bboxes.append(detection)

        return valid_bboxes

    # Finds box detections that have no white line below the midpoint of the bottom of its bbox
    # This is minimise spurious detections of bags and shit that will be outside the court
    # while keeping in mind that some of the collection box will overlap with the court lines (but the middle of the bbox shouldn't, hopefully)
    # Returns none if there are no "valid" boxes
    def get_valid_box_detections(
        self, detections: list[Detection]
    ) -> Optional[Detection]:
        mask = self.line.detect(self.frame)
        if not pi:
            cv2.imshow("mask", mask)
            cv2.waitKey(1)

        for detection in detections:
            if detection[0] != "box":
                continue
            x1, y1, x2, y2 = list(map(int, detection[2]))
            image_below = mask[y2:, (x1 + x2) // 2]
            line_exists = np.any(image_below)
            if not line_exists:
                return detection

        return None


if __name__ == "__main__":
    cam = Camera()

    while True:
        t = time.time()
        ret = cam.detect_closest()
        cam.detect_box()

        if not pi:
            cv2.imshow("frame", cam.debug)
            cv2.waitKey(1)

        print(f"Time taken: {time.time() - t}s")

        # try:
        #     d, th = ret
        #     print(f"d:{d}, th:{th}")
        #     break
        # except:
        #     print("no balls found")
