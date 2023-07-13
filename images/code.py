import cv2
import glob
import numpy as np

# Define the path to the folder containing calibration images
calibration_images_folder = "/home/bhajneet/catkin_ws/src/zebu/images"

# Define the pattern size of the calibration board
pattern_size = (9, 6)  # Number of inner corners on the calibration pattern

# Create a list to store the calibration images
calibration_images = []

# Read all images from the calibration folder
image_files = glob.glob(calibration_images_folder + "/*.jpg")  # Adjust the file extension if needed

# Iterate through the image files
for file in image_files:
    # Read the image
    img = cv2.imread(file)

    # If the image is not None, add it to the list
    if img is not None:
        calibration_images.append(img)

# Display the number of calibration images found
print("Number of calibration images:", len(calibration_images))

# Define the calibration board properties
calibration_board_size = pattern_size
calibration_square_size = 1.0  # Assuming the calibration pattern is printed on a square grid with 1 unit size

# Create arrays to store object points and image points from all calibration images
object_points = []  # 3D points in real-world coordinate space
image_points = []   # 2D points in image plane

# Prepare the object points, which are the (x, y, z) coordinates of the calibration pattern
objp = np.zeros((calibration_board_size[0] * calibration_board_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:calibration_board_size[0], 0:calibration_board_size[1]].T.reshape(-1, 2)
objp *= calibration_square_size

# Find the corners of the calibration pattern in each image
for img in calibration_images:
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the corners of the pattern
    ret, corners = cv2.findChessboardCorners(gray, calibration_board_size, None)

    # If corners are found, add the object points and image points to their respective lists
    if ret:
        object_points.append(objp)
        image_points.append(corners)

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    object_points, image_points, gray.shape[::-1], None, None
)

# Print the camera matrix and distortion coefficients
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)
