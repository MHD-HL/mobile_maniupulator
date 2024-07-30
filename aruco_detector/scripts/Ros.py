import rospy
from cv2 import aruco
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PoseStamped  # ROS message for pose data
from std_msgs.msg import String  # Import for String message type

# ROS specific variables
node_name = "aruco_marker_detector"
name = ""  # Global variable to store the name


def voice_callback(data):
    global name
    name = data.data
    rospy.loginfo(f"Received name: {name}")
    


def main():
    # ROS initialization
    global name
    voice_topic="recognized_voice_topic"

    rospy.init_node(node_name)
    pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
    sub = rospy.Subscriber(voice_topic, String, voice_callback, queue_size=1)

    rospy.loginfo(f"Node: {node_name} started")

    # Wait for the first message to be received
    rospy.wait_for_message(voice_topic, String)
    # Path to the calibration data file containing camera intrinsics
    calib_data_path = "/home/mhdkher/catkin_ws/src/aruco_detector/scripts/MultiMatrix.npz"

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

    if (name == 'square'):
        print(f"we are in the sqaure")
        marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    elif (name == 'cube'):
        marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        print(f"we are in the cube")
    
    else:
        return 
        #  default is ball
        #  marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    # Optional Aruco detector parameters
    param_markers = aruco.DetectorParameters()

    cap = cv.VideoCapture(0)

    # ... (camera capture, marker detection, pose estimation)
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

        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef)
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
                    f"id: {ids[0]} Dist: {round(distance, 2)}    the obj is : {name}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,  # Font scale
                    (0, 0, 255),
                    2,  # Font size
                    cv.LINE_AA,
                )

                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)}",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                    )       
            # Convert marker pose to ROS PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            # Replace if needed (camera frame)
            pose_msg.header.frame_id = "camera_link"
            pose_msg.pose.position.x = tVec[i][0][0]
            pose_msg.pose.position.y = -tVec[i][0][1]
            pose_msg.pose.position.z = tVec[i][0][2]
            # ... (orientation - TODO: implement conversion from rotation vector)

            # Publish marker pose
            pub.publish(pose_msg)

            # Print marker information (optional)
            print(f"id: {ids[0]} Dist: {round(distance, 2)}")
            print(f"x:{round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)}",)

        # ...

        # ROS housekeeping
        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break
    cap.release()
    cv.destroyAllWindows()


def callback(data):
    # Process data received from "other_topic" (replace with your logic)
    # ...
    rospy.loginfo(f"Received data from {data.topic}: {data}")


if __name__ == "__main__":
    main()
