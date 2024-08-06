#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def joint_command_publisher():
    # Initialize the ROS node
    rospy.init_node('joint_command_publisher', anonymous=True)

    # Create a publisher for the joint command topic
    joint_command_pub = rospy.Publisher('arm/arm_controller/command', JointTrajectory, queue_size=10)

    # Create a message to hold joint trajectory
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

    # Create a trajectory point
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.positions = [0.0, 0.5, -0.5, 1.0, 0.0]  # Set desired positions for 5 joints (in radians)
    trajectory_point.time_from_start = rospy.Duration(1.0)  # Time for the trajectory point

    # Add the trajectory point to the message
    joint_trajectory.points.append(trajectory_point)

    # Set the header timestamp to the current time
    joint_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)  # Set to 1 second in the future

    # Set the loop rate
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Publish the joint trajectory
        joint_command_pub.publish(joint_trajectory)
        rospy.loginfo("Publishing joint trajectory: %s", joint_trajectory.points[0].positions)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_command_publisher()
    except rospy.ROSInterruptException:
        pass