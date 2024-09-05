import cv2
import os
import numpy as np

from detector import Detector

class Camera:
    def __init__(self, device=0):
        self.cam = cv2.VideoCapture(device)
        self.frame = self.get_frame()

        # importing calibration
        script_dir = os.path.dirname(__file__)
        self.int_matrix = np.load(
            os.path.join(script_dir, "calibration", "int_matrix.npy")
        )
        self.dist_matrix = np.load(
            os.path.join(script_dir, "calibration", "dist_matrix.npy")
        )
        
        self.detector = Detector(
            os.path.join(script_dir, "calibration", "model.pt")
        )
        self.cam_dim = (640, 480)
        self.target_dim = 0.0342 * 2

    def undistort_img(self, frame):
        h, w = frame.shape[:2]
        n_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.int_matrix, self.dist_matrix, (w, h), 1, (w, h)
        )

        undistorted = cv2.undistort(
            frame, self.int_matrix, self.dist_matrix, None, n_camera_matrix
        )
        return undistorted
    
    def get_frame(self):
        _, frame = self.cam.read()
        if frame is None: return
        self.frame = self.undistort_img(frame)
        return self.frame
    
    def est_pose(self, detection):
        focal_length = self.int_matrix[0][0]
        target_box = detection[1]
        true_dim = self.target_dim

        # compute target pose based on pixel dims
        pix_h = target_box[3]
        pix_c = target_box[0]
        dis = true_dim / pix_h * focal_length

        x_shift = (self.cam_dim[2]/2) - pix_c
        theta = np.arctan(x_shift / focal_length)

        return dis, theta
    
    def detect_closest(self):
        bboxes, _ = self.detector.detect(self.frame)

        dis_min = 999
        theta = 999
        for detection in bboxes:
            dis, theta = self.est_pose(detection)
            if dis < dis_min:
                dis_min = dis

        return dis_min, theta