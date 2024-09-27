
## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
#for more info look into: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29


#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

# Function to append a character to the string message
def append_char(message, char_to_append):
    return message + char_to_append

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard ' + '%s', data.data)

# Append a character (e.g., '!') to the string message
    modified_message = append_char(data.data, ' :received time')
    rospy.loginfo('Modified message: %s', modified_message)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
