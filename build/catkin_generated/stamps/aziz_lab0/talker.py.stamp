#
#For details regearding this code, lookup
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        lab0_str = "This is Aziz lab0 talker %s" % rospy.get_time()
        rospy.loginfo(lab0_str)
        pub.publish(lab0_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
