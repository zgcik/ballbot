from picamera2 import Picamera2
import cv2
import numpy as np
import time

# Set chessboard dimensions
chessboard_size = (9, 6)  # Number of inner corners per a chessboard row and column
square_size = 0.025  # Size of a chessboard square (in meters, for real-world scaling)

# Prepare object points, like (0, 0, 0), (1, 0, 0), (2, 0, 0), ..., (8, 5, 0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp = objp * square_size  # Multiply by square size to get real-world dimensions

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Initialize the Pi Camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.start()

# Allow the camera to warm up
time.sleep(2)

# Capture chessboard images
num_images = 100  # Number of images to capture for calibration
captured_images = 0

while captured_images < num_images:
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        captured_images += 1
        print(f"Chessboard detected! Captured {captured_images}/{num_images} calibration images.")
    else:
        print("Chessboard not detected in this frame, retrying...")

    time.sleep(1)  # Add a slight delay to allow for better capture intervals

# Stop the camera
picam2.stop()

# Perform camera calibration
ret, int_matrix, dist_matrix, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the intrinsic and distortion matrices as .npy files
np.save('int_matrix.npy', int_matrix)
np.save('dist_matrix.npy', dist_matrix)

# Print the results
print("Intrinsic Matrix:")
print(int_matrix)

print("\nDistortion Coefficients:")
print(dist_matrix)

# Optional: Reprojection error (to assess the accuracy of the calibration)
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], int_matrix, dist_matrix)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

print("\nTotal reprojection error: ", mean_error / len(objpoints))
