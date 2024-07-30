# Import necessary libraries
import cv2 as cv
from cv2 import aruco
import numpy as np

# Path to the calibration data file containing camera intrinsics
calib_data_path = "Distance Estimation/calib_data/MultiMatrix.npz"
# calib_data_path = "Distance Estimation\calib_data\MultiMatrix.npz"

# Load the calibration data (camera matrix, distortion coefficients, etc.)
calib_data = np.load(calib_data_path)
# Print the contents of the calibration data (for debugging)
print(calib_data.files)

# Extract camera calibration parameters from loaded dat
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]


# Define the marker size in centimeters (adjust based on your actual marker)
MARKER_SIZE = 4  # centimeters (measure your printed marker size)
name = 'cube'
if (name == 'ball'):
    # Use Aruco predefined dictionary for marker detection (4x4 markers with 50 bits)
    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
elif (name == 'cube'):
    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
# ? DICT_4X4_50: is the Aruko size

# Optional Aruco detector parameters
param_markers = aruco.DetectorParameters()

cap = cv.VideoCapture(0)

while True:
    ret, frame = cap.read()
    # Read a frame from the video capture
    if not ret:
        break

    # Convert the frame from BGR to grayscale (better for Aruco detection)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detect Aruco markers in the grayscale frame
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers)

    # Process and display markers if any are detected
    if marker_corners:

        # This line estimates the pose (rotation and translation) of a single Aruco marker.
        # rVec(rotation vector) and tVec(translation vector) tell you how the marker is positioned and oriented relative to the camera.
        # MARKER_SIZE is the actual size of the marker in the same units used during camera calibration.
        # cam_mat and dist_coef are from camera calibration and describe the camera's properties.
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):

            # Draw the detected marker polygon
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)

            # Convert marker corners to a list for easier processing
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)

            # Extract corner points for text and distance calculation
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Calculate the distance of the marker from the camera
            distance = np.sqrt(tVec[i][0][2] ** 2 +
                               tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)

            # Draw the marker's pose axes (optional for visualization)
            point = cv.drawFrameAxes(
                frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

            # Draw text on the frame to display marker ID and distance
            cv.putText(
                frame,
                f"id: {ids[0]} name : {name} Dist: {round(distance, 2)}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,  # Font scale
                (0, 0, 255),
                2,  # Font size
                cv.LINE_AA,
            )

            cv.putText(
                frame,
                f"x:{round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)} ",
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            print(f"id: {ids[0]} name :{name} Dist: {round(distance, 2)}")
            # TODO: uncomment this to see the the (x,y)
            print(f"x:{round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)} ")

            # print(ids, "  ", corners)

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()


# ARUCO_DICT = {
# 	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
# 	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
# 	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
# 	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
# 	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
# 	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
# 	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
# 	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
# 	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
# 	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
# 	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
# 	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
# 	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
# 	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
# 	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
# 	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
# 	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
# 	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
# 	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
# 	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
# 	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
# }
