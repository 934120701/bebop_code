#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist   #Message that move the
from std_msgs.msg import Empty 

class ControlBebop():
	def __init__(self):
	    # ControlBebop is the name of the node sent to the master
	    rospy.init_node('ControlBebop', anonymous=False)

	    # Message to screen
	    rospy.loginfo(" Press CTRL+c to stop Bebop")
	    # Keys CNTL + c will stop script   
	    rospy.on_shutdown(self.shutdown)
	
	    #Publisher will send Twist message on topic /bebop/camera_control
	       
	    self.cmd_vel = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
	    self.cmd_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10) 
	    self.cmd_land = rospy.Publisher('/bebop/land', Empty, queue_size=10) 

	    # Bebop will receive the message 10 times per second. 
	    rate = rospy.Rate(10);   
	    # 10 Hz is fine as long as the processing does not exceed 1/10 second.

	    # Twist is a type of geometry_msgs for linear and angular velocity
	    # Pan the camera by sending command to angular.y
	    pan_camera_cmd = Twist()
	    # Linear speed in x in meters/second is + (forward) or - (backwards)
	    pan_camera_cmd.angular.y = -70	# Modify this value to change angle

	    # Tilt the camera by sending command to angular.z
	    tilt_camera_cmd = Twist()
	    tilt_camera_cmd.angular.z = 20	# # Modify this value to change angle
	    # Takeoff 
	    takeoff_cmd = Empty()
	    # Land 
	    land_cmd = Empty()

	    # Lift 
	    lift_cmd = Twist()
	    lift_cmd.linear.z = 0.1
	    # Down 
	    down_cmd = Twist()
	    down_cmd.linear.z = -0.1 
	    # Go forward at 0.2 m/s
	    forward_cmd = Twist()
	    forward_cmd.linear.x = 0.15
	    # Go backward at 0.2 m/s
	    backward_cmd = Twist()
	    backward_cmd.linear.x =-0.15
	    # Go left at 0.2 m/s
	    left_cmd = Twist()
	    left_cmd.linear.y = 0.15
	    # Go right at 0.2 m/s
	    right_cmd = Twist()
	    right_cmd.linear.y = -0.15
	    # Rotate clockwise at 0.2 m/s
	    rotate_clock_cmd = Twist()
	    rotate_clock_cmd.angular.z = -0.2
	    # Rotate anti-clockwise at 0.2 m/s
	    rotate_anti_clock_cmd = Twist()
	    rotate_anti_clock_cmd.angular.z = 0.2
	    
	    # Loop and TurtleBot will move until you type CNTL+c
	    while not rospy.is_shutdown():
		# publish the Twist values to the TurtleBot node /cmd_vel_mux
		rospy.loginfo("Bebop Takeoff")
		for i in range(25):  
		    self.cmd_takeoff.publish(takeoff_cmd)
		    # wait for 0.1 seconds (10 HZ) and publish again
		    rate.sleep()
		#rospy.loginfo("Bebop lift")
		#for j in range(8):
		#    self.cmd_vel.publish(lift_cmd)
		#    rate.sleep()		
		#rospy.loginfo("Bebop down")
		#for j in range(8):
		#    self.cmd_vel.publish(down_cmd)
		#    rate.sleep()
		rospy.loginfo("Bebop go forward")
		for j in range(12):
		    self.cmd_vel.publish(forward_cmd)
		    rate.sleep()
		rospy.loginfo("Bebop go backward")
		for j in range(24):
		    self.cmd_vel.publish(backward_cmd)
		    rate.sleep()
		#rospy.loginfo("Bebop go right")
		#for j in range(15):
		#    self.cmd_vel.publish(right_cmd)
		#    rate.sleep()		
		#rospy.loginfo("Bebop go left")
		#for j in range(30):
		#    self.cmd_vel.publish(left_cmd)
		#    rate.sleep()
		rospy.loginfo("Bebop rotate clockwise")
		for j in range(30):
		    self.cmd_vel.publish(rotate_clock_cmd)
		    rate.sleep()
		rospy.loginfo("Bebop rotate anti-clockwise")
		for j in range(30):
		    self.cmd_vel.publish(rotate_anti_clock_cmd)
		    rate.sleep()
		rospy.loginfo("Bebop Land")
		for j in range(15):
		    self.cmd_land.publish(land_cmd)
		    rate.sleep()
		

	def shutdown(self):
            # You can stop Bebop by publishing an empty Twist message 
	    rospy.loginfo("Stopping Bebop")
	    # 
	    # self.cmd_land.publish(land_cmd)
	    # self.cmd_vel.publish(Twist())
	    self.cmd_land.publish(Empty())
	    # Give TurtleBot time to stop
	    rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        ControlBebop()
    except:
        rospy.loginfo("End of the trip for Bebop")
