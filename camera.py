from typing import cast
import cv2
import os
import numpy as np
from picamera2 import Picamera2

from detector import Detector


class Camera:
    frame: cv2.typing.MatLike
    debug: cv2.typing.MatLike

    def __init__(self, device=0):
        cam = Picamera2()
        # mode = cam.sensor_modes[2]
        # config = cam.create_still_configuration(sensor={"output_size": mode["size"]})
        # cam.configure(config)  # type: ignore
        cam.start()
        self.cam = cam

        # importing calibration
        script_dir = os.path.dirname(__file__)
        self.int_matrix = np.load(
            os.path.join(script_dir, "calibration", "int_matrix.npy")
        )
        self.dist_matrix = np.load(
            os.path.join(script_dir, "calibration", "dist_matrix.npy")
        )

        self.detector = Detector(os.path.join(script_dir, "yolov5", "model.pt"))

        self.frame = self.get_frame()

        self.cam_dim = (4608, 2592)
        self.target_dim = 0.0334225*2  # * 2

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
        frame = cast(cv2.typing.MatLike, self.cam.capture_array("main"))
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if frame is None:
            return np.array([[[]]])
        # self.frame = self.__undistort_img__(frame)
        self.frame = frame
        self.debug = frame.copy()
        # cv2.imwrite("test.jpg", self.frame)
        return self.frame

    def __est_pose__(self, detection):
        focal_length = self.int_matrix[0][0]
        target_box = detection[1]
        true_dim = self.target_dim

        # compute target pose based on pixel dims
        pix_h = target_box[3]
        pix_c = target_box[0]
        dis = true_dim / pix_h * focal_length

        x_shift = (640 / 2) - pix_c #### Make sure it looks good 640
        theta = -np.arctan(x_shift / focal_length)

        return dis, theta


    def detect_closest(self):
        self.get_frame()
        bboxes, _ = self.detector.detect(self.frame)

        # bboxes = self.get_valid_detections(bboxes)
        if len(bboxes) == 0:
            return

        dis_min = 6
        theta_min = np.pi
        for i, detection in enumerate(bboxes):
            print(detection)
            dis, theta = self.__est_pose__(detection)
            if i == 0 or dis < dis_min:
                dis_min = dis
                theta_min = theta

        return (dis_min, theta_min)
    def __get_lines__(self):
        # filtering frame to get lines
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

        # canny edge detection
        edges = cv2.Canny(thresh, 50, 150, apertureSize=3)

        # hough line detection
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=10
        )

        for line in lines:
            cv2.line(self.debug, (line[0], line[1]), (line[2], line[3]), (0, 0, 255))

        return lines

    # def __line_equation__(self, x1, y1, x2, y2):
    #     a = y2 - y1
    #     b = x1 - x2
    #     c = -(a * x1 + b * y1)
    #     return a, b, c # in form ax + by + c = 0

    # def __find_intersection__(self, line1, line2):
    #     a1, b1, c1 = line1
    #     a2, b2, c2 = line2

    #     det = a1 * b2 - a2 * b1
    #     if det == 0:
    #         return None
    #     else:
    #         x = (b2 * c1 - b2 * c2) / det
    #         y = (a1 * c2 - a2 * c1) / det
    #         return (x, y)

    # def boundary_check(self):
    #     # updating frame
    #     self.get_frame()

    #     # finding the lines in frame
    #     lines = self.__get_lines__()

    #     if lines is not None:
    #         for i, line1 in enumerate(lines):
    #             x1, y1, x2, y2 = line1[0]
    #             line1_eq = self.__line_equation__(x1, y1, x2, y2)

    #             for j, line2 in enumerate(lines):
    #                 if i == j:
    #                     continue

    #                 x3, y3, x4, y4 = line2[0]
    #                 line2_eq = self.__line_equation__(x3, y3, x4, y4)

    #                 intersection = self.__find_intersection__(line1_eq, line2_eq)
    #                 if intersection:
    #                     return intersection
    #                 else:
    #                     return None

    def get_valid_detections(self, bboxes):
        lines = self.__get_lines__()

        valid_detections = []
        for detection in bboxes:
            y2_bbox = detection[1][2] + detection[1][3] / 2

            check = False
            for line in lines:
                _, y1, _, y2 = line[0]
                if y2_bbox < (y1 + y2) // 2:
                    check = False
                    break
                else:
                    check = True

            if check:
                valid_detections.append(detection)

        return valid_detections


if __name__ == "__main__":
    cam = Camera()

    while True:
        ret = cam.detect_closest()

        try:
            d, th = ret
            print(f"d:{d}, th:{th}")
        except:
            print("no balls found")