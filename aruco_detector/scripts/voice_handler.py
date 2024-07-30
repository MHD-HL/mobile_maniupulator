#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

node_name='speed_to_word'
Subscriber_voice_topic="speech_recognition/final_result"
bublisher_voice_topic="recognized_voice_topic"



def find_word(text,word1,word2):
    """
    Searches a given text for a specific word and returns the word if found.
    
    Args:
        text (str): The input text to search.
        word (str): The word to search for.
    
    Returns:
        str: The found word, or an empty string if not found.
    """
    if word1.lower()  in text.lower():
        return word1
    elif word2.lower()in text.lower():
        return word2
    else:
        return ""
    
def callback(text):
    """
    Callback function for the subscriber.
    
    Args:
        data (std_msgs.msg.String): The received message.
    """
    
    # Modify the received message and publish it
    word=find_word(text.data,'cube','square')

    rospy.loginfo("Received message: %s", word)
    
    modified_message =  word
    if word == 'cube' or word == 'square' :
        pub.publish(modified_message)
        rospy.loginfo("Published message: %s", word)


def listener():
    """
    ROS node that subscribes to a topic and publishes a modified message.
    """
    rospy.init_node(node_name, anonymous=True)

    # Subscribe to the 'input_topic' topic
    rospy.Subscriber(Subscriber_voice_topic, String, callback)

    # Publish to the 'output_topic' topic
    global pub
    pub = rospy.Publisher(bublisher_voice_topic, String, queue_size=10)
    # pub.publish(modified_message)


    # Spin the node to keep it running
    rospy.spin()

if __name__ == '__main__':
    listener()