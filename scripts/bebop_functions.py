from bebop_imports import *


# leave_gantry flys the drone forward ~ 2m at the current height
def leave_gantry():
  leave_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
  leave_cmd = Twist()
  timeout = 3
  current_time = time()

  while (time() < current_time + timeout):
    leave_cmd.linear.x = 0.1
    leave_pub.publish(leave_cmd)
    #print("leaving gantry ")

  leave_cmd.linear.x = 0
  leave_pub.publish(leave_cmd)

# drone_takeoff resets the drone and publishes a command to takeoff to the takeoff topic and flys to given altitude
def drone_takeoff():
  drone_reset()
  takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
  takeoff_cmd = Empty()
  takeoff_pub.publish(takeoff_cmd)
  takeoff_pub.publish(takeoff_cmd)
  sleep(3)
  #go_to_altitude(1.5)
  #sleep(3)

# drone_lane publishes to the land topic to cause the drone to land
def drone_land():

  drone_reset()
  land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
  land_cmd =Empty()
  land_pub.publish(land_cmd)
  sleep(3)

# Resets the drone, this is handy so multiple land/takeoff commands are not necessary
def drone_reset():

  reset_pub = rospy.Publisher('/bebop/reset', Empty, queue_size=10)
  reset_cmd =Empty()
  reset_pub.publish(reset_cmd)
  sleep(3)

# Takes a desired altitude in meters and invokes a callback that causes the drone to fly up or down
def go_to_altitude(desired_altitude):

  altitude_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, altitude_callback, (desired_altitude))

# Callback for the above subscriber. Sends one command to the drone to fly up/down 0.1 max speed if the current altitude is not within 10 cm of the desired altitude
def altitude_callback(data, args):

  altitude = data.altitude
  desired_altitude = args
  flight_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 10)
  flight_cmd = Twist()
  if (altitude < desired_altitude - 0.1):
    flight_cmd.linear.z = 0.2
    flight_pub.publish(flight_cmd)

  elif (altitude > desired_altitude + 0.1):
    flight_cmd.linear.z = -0.2
    flight_pub.publish(flight_cmd)

  else:
    flight_cmd.linear.z = 0
    #return
    flight_pub.publish(flight_cmd)
