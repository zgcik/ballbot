from camera import Camera
import cv2
import os
import time
import numpy as np
import glob
from picamera2 import Picamera2

BOARD_SIZE = (6, 9)


def take_calibration_imgs(cam):
    i = 0
    while True:
        # key = input("enter to take photo")
        frame = cam.capture_array("main")
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        print("frame captured")
        debug_frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

        if ret:
            cv2.drawChessboardCorners(debug_frame, BOARD_SIZE, corners, ret)
        cv2.imwrite("test.jpg", debug_frame)

        # take pic with 'c'
        if ret:
            last = int(time.time())
            script_dir = os.path.dirname(__file__)
            img_dir = os.path.join(script_dir, "calibration", "imgs")
            cv2.imwrite(os.path.join(img_dir, f"cal_img_{i}.jpg"), frame)
            print(f"image {i} saved...")
            i += 1

        # break with 'q'
        # if key == "q":
        #     break
        time.sleep(0.5)

        # cv2.imshow("camera feed", debug_frame)

    print("done")
    cam.release()
    cv2.destroyAllWindows()


# def calibrate_camera():
#     obj_points = []
#     img_points = []

#     objp = np.zeros((np.prod(BOARD_SIZE), 3), np.float32)
#     objp[:, :2] = np.mgrid[0 : BOARD_SIZE[0], 0 : BOARD_SIZE[1]].T.reshape(-1, 2)

#     script_dir = os.path.dirname(__file__)
#     img_dir = os.path.join(script_dir, "calibration", "imgs", "cal_img_*.jpg")
#     imgs = glob.glob(img_dir)
#     imgs = [cv2.imread(img) for img in imgs]

#     print('starting')

#     image_shape = None
#     i=0
#     for img in imgs:
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

#         if ret:
#             obj_points.append(objp)
#             img_points.append(corners)
#             if image_shape is None:
#                 image_shape = gray.shape[
#                     ::-1
#                 ]  # Capture the shape from the first valid image
#                 print(i)
#                 i += 1
#         else:
#             print("chessboard corners not found in one image.")

#     if not image_shape:
#         print("no valid images for calibration.")
#         return

#     # Perform camera calibration to get camera matrix and distortion coefficients
#     ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
#         obj_points, img_points, image_shape, None, None
#     )

#     print("camera matrix:\n", camera_matrix)
#     print("distortion coefficients:\n", dist_coeffs)

#     np.save(os.path.join(script_dir, "calibration", "int_matrix.npy"), camera_matrix)
#     np.save(os.path.join(script_dir, "calibration", "dist_matrix.npy"), dist_coeffs)

#     print('finished camera calibration')


def calibrate_camera():
    obj_points = []  # 3D points in real-world space
    img_points = []  # 2D points in image plane

    # Prepare the object points based on the known board size
    objp = np.zeros((np.prod(BOARD_SIZE), 3), np.float32)
    objp[:, :2] = np.mgrid[0 : BOARD_SIZE[0], 0 : BOARD_SIZE[1]].T.reshape(-1, 2)

    # Define the directory for calibration images
    script_dir = os.path.dirname(__file__)
    img_dir = os.path.join(script_dir, "calibration", "imgs", "cal_img_*.jpg")
    imgs = glob.glob(img_dir)
    imgs = [cv2.imread(img) for img in imgs]

    print("Starting calibration...")

    if not imgs:
        print("No images found for calibration.")
        return

    image_shape = None
    for i, img in enumerate(imgs):
        if img is None:
            print(f"Image {i} could not be loaded.")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

        if ret:
            obj_points.append(objp)
            img_points.append(corners)
            if image_shape is None:
                image_shape = gray.shape[
                    ::-1
                ]  # Capture shape from the first valid image
            print(f"Chessboard detected in image {i}.")
        else:
            print(f"Chessboard corners not found in image {i}.")

    if not image_shape:
        print("No valid images found for calibration.")
        return

    # Perform camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_shape, None, None
    )

    if not ret:
        print("Calibration failed.")
        return

    # Display results
    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coeffs)

    # Save the results
    np.save(os.path.join(script_dir, "calibration", "int_matrix.npy"), camera_matrix)
    np.save(os.path.join(script_dir, "calibration", "dist_matrix.npy"), dist_coeffs)

    print("Finished camera calibration successfully.")


def calibrate_baseline():
    pass


def calibrate_wheel():
    pass


if __name__ == "__main__":
    cam = Picamera2()
    # mode = cam.sensor_modes[2]
    # config = cam.create_still_configuration(sensor={"output_size": mode["size"]})
    # cam.configure(config)  # type: ignore
    cam.start()

    # take_calibration_imgs(cam)
    calibrate_camera()
