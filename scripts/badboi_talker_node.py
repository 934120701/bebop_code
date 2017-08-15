#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def badboi_talker():
    pub = rospy.Publisher('robot_chat', String, queue_size=10)
    rospy.init_node('badboi_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "takoff" 
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        badboi_talker()
    except rospy.ROSInterruptException:
        pass