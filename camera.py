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
        bboxes, img, xyxys = self.detector.detect(self.frame)
        
        # bboxes = self.get_valid_detections(bboxes)
        if len(bboxes) == 0:
            return
        ###Line stuff
        valid_det, idx = self.get_valid_detections(xyxys,img)
        
        print(valid_det)
        print(idx)
        print(bboxes)
        print(idx)
        valid_bboxes = []
        # if len(idx) !=0:
        #     for i in idx:
        #         valid_bboxes.append(bboxes[i])
        # else: 
        #     return
        ###
        
        dis_min = 6
        theta_min = np.pi
        for i, detection in enumerate(bboxes):
            dis, theta = self.__est_pose__(detection)
            if i == 0 or dis < dis_min:
                dis_min = dis
                theta_min = theta

        return (dis_min, theta_min)

    def get_valid_detections(self, bboxes,image):
        indx = []
        image = cv2.rotate(image, cv2.ROTATE_180)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply a binary threshold to isolate white regions
        _, binary_mask = cv2.threshold(gray_image, 250, 255, cv2.THRESH_BINARY)
        
        # Find contours of the white blobs
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        
        # Initialize lists to hold valid contours and pixel locations
        valid_contours = []
        blob_pixel_locations = []
        # Collect all contour points (blobs)
        for contour in contours:
            for point in contour[:, 0, :]:
                blob_pixel_locations.append([point[0], point[1]])  # Collect all blob points


        
        # Initialize a list for valid bounding boxes
        valid_bbox = []
        # Loop through each bounding box
        j = 0
        for bbox in bboxes:
            # Calculate the center of the current bounding box
            bbox_center_x = int((bbox[0][0] + bbox[1][0]) / 2)
            bbox_center_y = int((bbox[0][1] + bbox[1][1]) / 2)
            # Get the bottom boundary (y-coordinate) of the bounding box
            bbox_bottom_y = int(bbox[1][1])
            # Check if there are any blobs below the bounding box's bottom boundary
            has_blob_below = False
            for blob in blob_pixel_locations:
                # Check if the blob is directly below the bounding box
                if bbox[0][0] <= blob[0] <= bbox[1][0] and blob[1] > bbox_bottom_y:
                    has_blob_below = True
                    break  # Stop checking if we found a blob below
            # Draw blue or green dot based on whether there are blobs below
            if not has_blob_below:
                valid_bbox.append(bbox)  # Append to valid bbox list if no blobs below
                indx.append(j)
            j+=1
                

        # for box in valid_bbox:
        #     # Get the box coordinates
        #     bbox_center_x = int((box[0][0] + box[1][0]) / 2)
        #     bbox_center_y = int((box[0][1] + box[1][1]) / 2)
        #     #Dot:
        #     cv2.circle(image, (int(bbox_center_x), int(bbox_center_y)), radius=5, color=(255, 0, 255), thickness=-1)
        #     # Optionally, draw the bounding boxes on the image
        #     cv2.rectangle(image, (int(box[0][0]), int(box[0][1])), (int(box[1][0]), int(box[1][1])), (255, 0, 255), 2)
        # cv2.imwrite("Img.jpg", binary_mask)

        return valid_bbox,indx  # Return the valid bounding boxes


if __name__ == "__main__":
    cam = Camera()

    while True:
        ret = cam.detect_closest()

        try:
            d, th = ret
            print(f"d:{d}, th:{th}")
            break
        except:
            print("no balls found")
            