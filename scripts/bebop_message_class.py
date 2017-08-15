#!/usr/bin/env python
import rospy
from std_msgs.msg import String


class bebop_message_class:

        def __init__(self):
                self.bebopMsgReceived = ""
                self.badboiMsg = "takeoff"
                self.badboiPub = rospy.Publisher("badboi_chat", String, queue_size=10)

        def bebop_callback(self, data):
                self.bebopMsgReceived = data.data

        def bebop_caller(self):
                self.badboiSub = rospy.Subscriber("bebop_chat", String, self.bebop_callback)

        def badboi_send(self):
                self.badboiPub.publish(self.badboiMsg)



if __name__ == '__main__':
        try:
                rospy.init_node('badboi_node', anonymous=True)
                rate = rospy.Rate(0.5)
                bebopClassCall = bebop_message_class()
                while not rospy.is_shutdown() and bebopClassCall.bebopMsgReceived != "drive":
                        bebopClassCall.bebop_caller() 
                        bebopClassCall.badboi_send()
                        print(bebopClassCall.bebopMsgReceived)
                        rate.sleep()
                print("Message received: ", bebopClassCall.bebopMsgReceived)
        except KeyboardInterrupt:
                pass

