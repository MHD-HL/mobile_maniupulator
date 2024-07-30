#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def voice_publisher():
    # Initialize the ROS node
    rospy.init_node('voice_publisher', anonymous=True)
    
    # Create a publisher that publishes String messages on the 'voice_topic' topic
    pub = rospy.Publisher('voice_topic', String, queue_size=10)
    
    # Set the rate at which to publish (1 Hz in this example, adjust as needed)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Get user input for the name
        name = input("Enter a name (or 'q' to quit): ")
        
        if name.lower() == 'q':
            break

        # Create a String message
        message = String()
        message.data = name

        print(f"Publishing '{name}'. Press Ctrl+C to stop and enter a new name.")

        try:
            while not rospy.is_shutdown():
                # Publish the message
                pub.publish(message)
                rospy.loginfo(f"Published: {name}")
                # rate.sleep()
        except rospy.ROSInterruptException:
            print("Stopped publishing. You can enter a new name or 'q' to quit.")

    print("Exiting the publisher node.")

if __name__ == '__main__':
    try:
        voice_publisher()
    except rospy.ROSInterruptException:
        pass