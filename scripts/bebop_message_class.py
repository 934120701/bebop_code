import rospy
from std_msgs.msg import String


class bebop_message_class:

	def __init__(self):
		self.bebopMsgReceived = ""
		self.badboiMsg = "takeoff"
		self.badboiPub = rospy.Publisher("robot_chat", String, queue_size=10)

	def badboi_callback(self, data):
		self.bebopMsgReceived = data.data

	def badboi_caller(self):
    	self.badboiSub = rospy.Subscriber("robot_chat", String, callback)

    def badboi_send(self):
    	self.bebopPub.publish(self.badboiMsg)


if __name__ == '__main__':
	rospy.init_node('badboi_talker', anonymous=True)
	rate = rospy.Rate(1)
    try:
    bebopClassCall = bebop_message_class()
    while 1:
    	bebopClassCall.badboi_send()
        bebopClassCall.bebop_caller() 
        if bebopClassCall.bebopiMsgReceived == "drive":
            print("Message received: ", bebopClassCall.bebopMsgReceived)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
