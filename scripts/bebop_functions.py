from bebop_imports import *


# leave_gantry flys the drone forward ~ 2m at the current height
def leave_gantry():
  leavePub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
  leaveCmd = Twist()
  timeout = 3
  currentTime = time()

  while (time() < currentTime + timeout):
    leaveCmd.linear.x = 0.1
    leavePub.publish(leaveCmd)
    #print("leaving gantry ")

  leaveCmd.linear.x = 0
  leavePub.publish(leaveCmd)

# drone_takeoff resets the drone and publishes a command to takeoff to the takeoff topic and flys to given altitude


def drone_takeoff():
  drone_reset()
  takeoffPub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
  takeoffCmd = Empty()
  takeoffPub.publish(takeoffCmd)
  sleep(3)
  takeoffPub.publish(takeoffCmd)
  sleep(3)

# drone_lane publishes to the land topic to cause the drone to land


def drone_land():

  landPub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
  landCmd = Empty()
  landPub.publish(landCmd)
  sleep(3)

# Resets the drone, this is handy so multiple land/takeoff commands are not necessary


def drone_reset():

  resetPub = rospy.Publisher('/bebop/reset', Empty, queue_size=10)
  resetCmd = Empty()
  resetPub.publish(resetCmd)
  sleep(3)

# Takes a desired altitude in meters and invokes a callback that causes the drone to fly up or down

class altitude_class:

    def __init__(self, desiredAltitude):
        self.stopSubscribe = False
        self.desiredAltitude = desiredAltitude
        self.altitude = int
        print("desired altitude set at: ", self.desiredAltitude)

    def go_to_altitude(self):
        flightPub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        flightCmd = Twist()
        print("go to altitude called")
        rate = rospy.Rate(5)

        while self.stopSubscribe != True:

            self.altitudeSub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                                 Ardrone3PilotingStateAltitudeChanged, self.altitude_callback)
        

            print("desired: ", self.desiredAltitude)
            print("current: ", self.altitude)

            if (self.altitude < self.desiredAltitude - 0.1):
                flightCmd.linear.z = 0.2
                flightPub.publish(flightCmd)

            elif (self.altitude > self.desiredAltitude + 0.1):
                flightCmd.linear.z = -0.2
                flightPub.publish(flightCmd)

            else:
                flightCmd.linear.z = 0
                flightPub.publish(flightCmd)
                sleep(3)
                self.stopSubscribe = True
                self.altitudeSub.unregister()
                print("stopSubscribe made true in else")
            rate.sleep()

        '''print("stop subscibe in go_to_altitude: ", self.stopSubscribe)
        if (self.stopSubscribe != True):
            self.altitudeSub.unregister()
            print("Unsubscribed from altitude")
            print("Leaving with altitude: ", self.altitude)'''

# Callback for the above subscriber. Sends one command to the drone to fly up/down 0.1 max speed if the current altitude is not within 10 cm of the desired altitude


    def altitude_callback(self, data):

        self.altitude = data.altitude
        '''   flightPub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        flightCmd = Twist()
        print("desired: ", self.desiredAltitude)
        print("current: ", self.altitude)

        if (self.altitude < self.desiredAltitude - 0.1):
            flightCmd.linear.z = 0.2
            flightPub.publish(flightCmd)

        elif (self.altitude > self.desiredAltitude + 0.1):
            flightCmd.linear.z = -0.2
            flightPub.publish(flightCmd)

        else:
            flightCmd.linear.z = 0
            flightPub.publish(flightCmd)
            sleep(3)
            self.stopSubscribe = True
            self.altitudeSub.unregister()
            print("stopSubscribe made true in else")'''



class badboi_message_class:
    def __init__(self):
        self.badboiMsgReceived = ""
        self.bebopMsg = "drive"
        self.bebopPub = rospy.Publisher("bebop_chat", String, queue_size=10)

    def badboi_callback(self, data):  
        self.badboiMsgReceived = data.data

    def badboi_caller(self):
        self.badboiSub = rospy.Subscriber("badboi_chat", String, self.badboi_callback)

    def bebop_send(self):
        self.bebopPub.publish(self.bebopMsg)




