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
        self.altitudeSub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                              Ardrone3PilotingStateAltitudeChanged, self.altitude_callback)
        self.land = False
        self.altitudeAchieved = False


    def go_to_altitude(self):
        flightPub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        flightCmd = Twist()
        rate = rospy.Rate(5)
        self.stopSubscribe = False
        #print(self.altitude)

        #while self.stopSubscribe != True:

        
      

        if (self.altitude < self.desiredAltitude - 0.1):
            flightCmd.linear.z = 0.15
            #print("up ", self.altitude)
            flightPub.publish(flightCmd)

        elif (self.altitude > self.desiredAltitude + 0.1):
            flightCmd.linear.z = -0.2
            #print("down ", self.altitude)
            flightPub.publish(flightCmd)

        else:
            flightCmd.linear.z = 0
            #print("same ", self.altitude)
            flightPub.publish(flightCmd)
            self.altitudeAchieved = True

# Callback for the above subscriber. Sends one command to the drone to fly up/down 0.1 max speed if the current altitude is not within 10 cm of the desired altitude


    def altitude_callback(self, data):

        self.altitude = data.altitude
        if (self.land == False):
          self.go_to_altitude()
        else:
          drone_land()


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

def setup_client():
    host = '192.168.1.181'
    port = 10072
    client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    client.connect((host, port))
    return client

def heard_from_badboi(client): 
    while True:
        data_from_badboi = client.recv(1024)
        data_from_badboi = data_from_badboi.decode('utf-8')
        if data_from_badboi == "takeoff":
            print("client received message:", data_from_badboi)
            client.sendall(str.encode("received"))
            print("client sent message received")
            break

def send_msg_to_badboi(client):
    while True:
        client.sendall(str.encode("gohome"))
        print("client sent message go home")
        data_from_badboi = client.recv(1024)
        data_from_badboi = data_from_badboi.decode('utf-8')
        if data_from_badboi == 'received':
            print("client received message received")
            break
    client.close()



