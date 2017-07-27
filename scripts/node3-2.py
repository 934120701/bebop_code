#!/usr/bin/env python

'''
This program causes the camera on the Bebop to pan in the Y direction so that the tag is in the middle
of the screen.
'''
from __future__ import print_function
import roslib
roslib.load_manifest('opencv_package')
import sys
import rospy
import cv2
import cv2.aruco as aruco
import glob
import numpy as np


from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from time import sleep, time
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from collections import deque


class image_converter:

  def __init__(self):
    # Image after use in cv2 can be published
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
    # Added the line below so we can publish messages to the camera_control topic with message type Twist
    self.message_pub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
    self.bridge = CvBridge()
    # Subscribe to the image_raw topic, and use the callback function
    self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback)
    self.cmd_vel = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
    self.pan_camera_cmd = Twist()
    self.flight_cmd = Twist()
    self.flight_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
    self.aruco_code_center_pixel = None
    self.previous_aruco_code_center_pixels = deque([None] * 3)

  # tag_center takes the values for the pixels of the corners of an identified ArUco tag and calculates the 
  # position of the center pixel based on these, returning the two element list with x and y positions
  def tag_center(self, corners):
  
    aruco_code_center_x = (corners[0][0][0][0] + corners[0][0][2][0])/2
    aruco_code_center_y = (corners[0][0][0][1] + corners[0][0][2][1])/2
    aruco_code_center_pixel_position = [aruco_code_center_x, aruco_code_center_y]
    return aruco_code_center_pixel_position

  # camera_angle_correction takes the ArUco code center pixel position and the current angle of the camera
  # calculates whether we need to pan up or down to move the ArUco tag to the center of the screen
  # returns the new angle we need to publish to the topic
  def camera_angle_correction(self,aruco_code_center_pixel_position, current_angle_y):

    position_error_x = 428 - aruco_code_center_pixel_position[0]
    position_error_y = 240 - aruco_code_center_pixel_position[1]
  
    # Does some bounds checking as +/- 70 is maximum angle
    if ((position_error_y > 0) and (current_angle_y <= 65) and (current_angle_y >= -65)):
      if current_angle_y == 65:
        new_angle_y= current_angle_y
      else:
        new_angle_y = current_angle_y + 1
  
    elif ((position_error_y < 0) and (current_angle_y - 1 <= 65) and (current_angle_y - 1 >= -65)):
      if current_angle_y==-65:
        new_angle_y=current_angle_y
      else:
        new_angle_y = current_angle_y - 1

    # If the new_angle_y is going to go out of bounds, set it equal to the current angle
    else:
      new_angle_y = current_angle_y

    return new_angle_y


  # publish_the_message takes the camera_angle we want to publish and publishes it to the camera_control topic
  def publish_the_message(self, camera_angle):
      # Only changes the y plane values, we could additionally add in x also
      self.pan_camera_cmd.angular.y = camera_angle
      self.cmd_vel.publish(self.pan_camera_cmd)
      #print(camera_angle)

  # draw the coordinate lines on the image
  def draw(img, corners, imgpts):
      corner = tuple(corners[0].ravel())
      img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
      img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
      img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
      return img

  # flys the Bebop towards an identified tag to place the tag at the bottom of the video feed
  def flight_commands(self, aruco_code_center_pixel_position):
    if aruco_code_center_pixel_position[0] < 350:
      #turn anticlockwise
      self.flight_cmd.angular.z = 0.15
      self.flight_cmd.linear.x = 0
      print("anticlockwise")
    elif aruco_code_center_pixel_position[0] > 506:
      #turn clockwise
      self.flight_cmd.angular.z = -0.15
      self.flight_cmd.linear.x = 0
      print("clockwise")
    elif aruco_code_center_pixel_position[0] < 506 and aruco_code_center_pixel_position[0] > 350 and aruco_code_center_pixel_position[1] < 350:
      #fly forwards
      self.flight_cmd.angular.z = 0
      self.flight_cmd.linear.x = 0.04
      print("forwards")
    else:
      #hover
      self.flight_cmd.angular.z = 0
      self.flight_cmd.linear.x = 0
      self.flight_cmd.linear.y = 0
      self.flight_cmd.linear.z = 0

      print("hover")

    self.flight_pub.publish(self.flight_cmd)

  # adds a new tag position to the list the holds the last 3 detected positions and returns the list
  def update_previous_aruco_code_center_pixels_list(self, aruco_code_center_pixel_position):
    # if the 3 element list is full, pop the value at element 0 before appending the next position
    if type(self.previous_aruco_code_center_pixels[2]) != None:
      self.previous_aruco_code_center_pixels.popleft()
    self.previous_aruco_code_center_pixels.append(aruco_code_center_pixel_position)

    return self.previous_aruco_code_center_pixels

  # calculate the vector for the change in ArUco tag position from either the 0 and 3rd element or 0 and 1st element
  # if the list contains only 2 positions. Returns this vector
  def aruco_tag_position_change(self, previous_aruco_code_center_pixels):
    position_change_vector = []

    if type(previous_aruco_code_center_pixels[2]) == list:
      position_change_vector = [previous_aruco_code_center_pixels[2][0] - previous_aruco_code_center_pixels[0][0],
                                  previous_aruco_code_center_pixels[2][1] - previous_aruco_code_center_pixels[0][1]]
    elif (type(previous_aruco_code_center_pixels[2]) != list and type(previous_aruco_code_center_pixels[1]) == list):
      position_change_vector = [previous_aruco_code_center_pixels[1][0] - previous_aruco_code_center_pixels[0][0],
                                  previous_aruco_code_center_pixels[1][1] - previous_aruco_code_center_pixels[0][1]]
    else:
      position_change_vector = None
    return position_change_vector


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    global camera_angle, count

    markerLength = 5
    count = count + 1

    # Load the camera coefficients etc

    cv2.line(cv_image, (0, 350), (856, 350), 255, 2)
    cv2.line(cv_image, (350, 0), (350, 480), 255, 2)
    cv2.line(cv_image, (506, 0), (506, 480), 255, 2)

    with np.load('B.npz') as X:
      mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]


    # Make the image gray for ArUco tag detection
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Assign the dictionary we wish to use (6 bit, with 250 separate tags)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #aruco.drawDetectedMarkers(cv_image, corners, ids)

    camera_angle_birds_eye = -70
    self.publish_the_message(camera_angle_birds_eye)


    # this if statement lets us check if we have detected a tag by checking the size
    # of the corners list to see if it's greater than 0. If it is, then we want to
    # find the center position of the tag and then correct the camera angle to center
    # it by publishing back a new angle to the control_camera topic.
    if (len(corners) != 0):
      # Draw on the markers so we can see they've been detected
      gray = aruco.drawDetectedMarkers(cv_image, corners, ids)
      rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)  # For a single marker
      r33 = cv2.Rodrigues(rvec,jacobian=0)
      #print(r33[0])
      gray = aruco.drawAxis(gray, mtx, dist, rvec, tvec, 10)
      #cv_image = cv2.warpPerspective(gray, r33[0], (856, 480))
      self.aruco_code_center_pixel = self.tag_center(corners)
      previous_aruco_code_center_pixels = self.update_previous_aruco_code_center_pixels_list(self.aruco_code_center_pixel)
      #print(self.previous_aruco_code_center_pixels)
      #camera_angle = self.camera_angle_correction(aruco_code_center_pixel, camera_angle)
      camera_angle_birds_eye = -70
      self.publish_the_message(camera_angle_birds_eye)
      count = 0

    # if there was no tag detected in that frame, make sure we have at least 2 values in our previous_aruco_code_center_pixels
    # list before getting the direction vector
    elif (self.previous_aruco_code_center_pixels[1] != None):
      direction_vector = self.aruco_tag_position_change(self.previous_aruco_code_center_pixels)
      print(direction_vector)

    '''print("count: ", count)
    if (count <= 30):
      self.flight_commands(self.aruco_code_center_pixel)
      print("Flying to known position")'''

    # Display the video feed frames every 3 ms.
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(5) #5


    # Publish the image back into ROS image message type (not sure why, I guess it's if you
    # want to do something else with it after using OpenCV).
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    except CvBridgeError as e:
      print(e)


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

# drone_takeoff resets the drone and publishes a command to takeoff to the takeoff topic
def drone_takeoff():
  drone_reset()
  takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
  takeoff_cmd = Empty()
  takeoff_pub.publish(takeoff_cmd)
  sleep(5)

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


# Setup an initial camera angle
camera_angle = -70
count = 0
def main(args):
  # Initialise the node under the name image_converter
  rospy.init_node('image_converter', anonymous=True)

  '''drone_takeoff()
  drone_takeoff()
  print("going inside")
  go_to_altitude(1.5)
  sleep(5)
  print('outside')'''

  # Assign the ic variable to the class type of image_converter
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
