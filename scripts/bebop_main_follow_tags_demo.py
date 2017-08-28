#!/usr/bin/env python

'''
This program flys a Bebop drone by following ArUco tags to a maximum ID before publishing a message
for the grond robot. Flys in reverse and lands at a given tag. 
'''

from bebop_functions import *

class image_converter:

    def __init__(self):
        ''' Create the publishers and subscribers'''
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.message_pub = rospy.Publisher(
            "/bebop/camera_control", Twist, queue_size=10)
        self.flight_pub = rospy.Publisher(
            '/bebop/cmd_vel', Twist, queue_size=10)
        self.camera_pub = rospy.Publisher(
            '/bebop/camera_control', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw", Image, self.callback)

        ''' Create the variables used across the functions in the class image_converter'''
        self.bridge = CvBridge()
        self.panCameraCmd = Twist()
        self.flightCmd = Twist()
        self.highestTagCenterPixel = None
        self.lowestTagCenterPixel = None
        self.previousArucoCodeCenterPixels = deque([None] * 3)
        self.highestTagIdListWithPositions = deque()
        self.lowestTagIdListWithPositions = deque()
        self.targetTagPosition = []
        self.targetTagId = 0
        self.ids = int
        self.highestTagId = int
        self.lowestTagId = int
        self.highestTagIndex = int
        self.lowestTagId = int
        self.firstTagSeen = False
        self.atStartTag = False
        self.goToTag = 1
        self.m_pidX = PID_class(0.001,
                                0.001,
                                0.008,
                                -0.05,
                                0.05,
                                -0.1,
                                0.1)
        self.m_pidY = PID_class(0.001,
                                0.001,
                                0.008,
                                -0.1,
                                0.1,
                                -0.1,
                                0.1)

    ''' tag_center takes the values for the pixels of the corners of an identified ArUco tag and calculates the
  position of the center pixel based on these, returning the two element list with x and y positions'''

    def tag_center(self, corners):

        arucoCodeCenterX = (corners[0][0] + corners[2][0]) / 2
        arucoCodeCenterY = (corners[0][1] + corners[2][1]) / 2
        arucoCodeCenterPixelPosition = [arucoCodeCenterX, arucoCodeCenterY]
        return arucoCodeCenterPixelPosition

    ''' camera_angle_correction takes the ArUco code center pixel position and the current angle of the camera
  calculates whether we need to pan up or down to move the ArUco tag to the center of the screen
  returns the new angle we need to publish to the topic'''

    '''def camera_angle_correction(self, arucoCodeCenterPixelPosition, currentAngleY):

        positionErrorX = 428 - arucoCodeCenterPixelPosition[0]
        positionErrorY = 240 - arucoCodeCenterPixelPosition[1]

        ''' Does some bounds checking as +/- 70 is maximum angle'''
        if ((positionErrorY > 0) and (currentAngleY <= 65) and (currentAngleY >= -65)):
            if currentAngleY == 65:
                newAngleY = currentAngleY
            else:
                newAngleY = currentAngleY + 1

        elif ((positionErrorY < 0) and (currentAngleY - 1 <= 65) and (currentAngleY - 1 >= -65)):
            if currentAngleY == -65:
                newAngleY = currentAngleY
            else:
                newAngleY = currentAngleY - 1

            ''' If the newAngleY is going to go out of bounds, set it equal to the current angle'''
        else:
            newAngleY = currentAngleY

        return newAngleY'''

    ''' publish_camera takes the camera_angle we want to publish and publishes it to the camera_control topic'''

    def publish_camera(self, cameraAngle):
        ''' Only changes the y plane values, we could additionally add in x also'''
        self.panCameraCmd.angular.y = cameraAngle
        self.camera_pub.publish(self.panCameraCmd)

    ''' Draw the coordinate lines on the image'''
    def draw(img, corners, imgPts):

        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgPts[0].ravel()), (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgPts[1].ravel()), (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgPts[2].ravel()), (0, 0, 255), 5)
        return img

    ''' Flys the Bebop towards an identified tag to place the tag at the bottom of the video feed'''

    '''def flight_commands_current_tag(self, arucoCodeCenterPixelPosition):

        # print("current")
        # rospy.loginfo("current")
        if arucoCodeCenterPixelPosition[0] < 350:
            # turn anticlockwise
            self.flightCmd.angular.z = 0.15
            self.flightCmd.linear.x = 0
            # print("anticlockwise")
        elif arucoCodeCenterPixelPosition[0] > 506:
            # turn clockwise
            self.flightCmd.angular.z = -0.15
            self.flightCmd.linear.x = 0
           # print("clockwise")
        elif arucoCodeCenterPixelPosition[0] < 506 and arucoCodeCenterPixelPosition[0] > 350 and arucoCodeCenterPixelPosition[1] < 350:
            # fly forwards
            self.flightCmd.angular.z = 0
            self.flightCmd.linear.x = 0.04
            # print("forwards")
        else:
            # hover
            self.flightCmd.angular.z = 0
            self.flightCmd.linear.x = 0
            self.flightCmd.linear.y = 0
            self.flightCmd.linear.z = 0

            # print("hover")

        self.flight_pub.publish(self.flightCmd)'''

    ''' Flight_commands_previous_tags takes in the direction vector and calculates values for the x and y velocity
    of the drone in order to navigate it back in the direction the last ArUco tag was lost'''

    """def flight_commands_previous_tags(self, directionVector):

        rospy.loginfo("past")

        ''' Calculate the rate of the x and y components in the video feed to each other'''
        ratioX = abs(directionVector[
                      0]) * (0.05 / (abs(directionVector[0]) + abs(directionVector[1])))
        ratioY = abs(directionVector[
                      1]) * (0.05 / (abs(directionVector[0]) + abs(directionVector[1])))
        rospy.loginfo("directionVector: %d  %d", directionVector[0], directionVector[1])
        rospy.loginfo("ratioX: %f   ratioY: %f", ratioX, ratioY)

        ''' Set the larger ratio to a maximum of 0.05 velocity and scale the other lower'''
        if (ratioX < ratioY):
            multiplier = 0.05 / ratioY
            ratioY = ratioY * multiplier
            ratioX = ratioX * multiplier

        else:
            multiplier = 0.05 / ratioX
            ratioY = ratioY * multiplier
            ratioX = ratioX * multiplier

        rospy.loginfo(
            "ratioX multiplied: %f   ratioY multiplied: %f ", ratioX, ratioY)

        ''' If the direction vector for the x coordinate in the feed is less than 0, the ArUco tag will have disappeared to
        the left, therefore we need to fly the drone left (positive y in the drone's coordinates) to rediscover the tag.
        If it's positive, the tag disappeared to the right, so we should fly the drone to the right (negative y in the
        drone's coordinate system).'''
        if direction_vector[0] < 0:
            self.flightCmd.linear.x = + ratioYratioY
        else:
            self.flightCmd.linear.x = - ratioY

        ''' If the direction vector for the y coordinate in the feed is less than 0, the ArUco tag will have disappeared to
    at the top of the feed, therefore we need to fly forwards to rediscover the tag. If it's positive, the tag
    went off at the bottom of the feed and we therefore need to fly backwards to rediscover the tag.'''
        if direction_vector[1] < 0:
            self.flightCmd.linear.y = ratioX
        else:
            self.flightCmd.linear.y = - ratioX

        self.flightCmd.angular.z = 0
        self.flightCmd.linear.z = 0

        self.flight_pub.publish(self.flightCmd)"""

    ''' Adds a new tag position to the list the holds the last 3 detected positions and returns the list'''

    """def update_previous_aruco_code_center_pixels_list(self, arucoCodeCenterPixelPosition):
        ''' If the 3 element list is full, pop the value at element 0 before appending the next position'''
        if type(self.previousArucoCodeCenterPixels[2]) != None:
            self.previousArucoCodeCenterPixels.popleft()
        self.previousArucoCodeCenterPixels.append(arucoCodeCenterPixelPosition)

        return self.previousArucoCodeCenterPixels"""

    ''' Calculate the vector for the change in ArUco tag position from either the 0 and 3rd element or 0 and 1st element
  if the list contains only 2 positions. Returns the positionChangeVector (direction_change vector).'''

    """def aruco_tag_position_change(self, previousArucoCodeCenterPixels):

        positionChangeVector = []

        ''' If the 3rd element in the list is a list, we can use the last and the 3rd from last positions of the tag.'''
        if type(previousArucoCodeCenterPixels[2]) == list:
            positionChangeVector = [previousArucoCodeCenterPixels[2][0] - previousArucoCodeCenterPixels[0][0],
                                      previousArucoCodeCenterPixels[2][1] - previousArucoCodeCenterPixels[0][1]]
            '''elif the 3rd element in the list if not a list (therefore is = None), we should use the 1st and second elements
    to calculate the position change vector.'''
        elif (type(previousArucoCodeCenterPixels[2]) != list and type(previousArucoCodeCenterPixels[1]) == list):
            positionChangeVector = [previousArucoCodeCenterPixels[1][0] - previousArucoCodeCenterPixels[0][0],
                                      previousArucoCodeCenterPixels[1][1] - previousArucoCodeCenterPixels[0][1]]

            ''' If for some reason we don't have 2 previous tag positions to calculate with, set the vector to None.'''
        else:
            positionChangeVector = None
        return positionChangeVector"""

    ''' Conversion from x and y cartesian coordinates to distance and angle in polar'''

    """def cart2pol(self, cartesian):

        rho = np.sqrt((cartesian[0] ** 2) + (cartesian[1] ** 2))
        phi = np.arctan2(cartesian[1], cartesian[0])
        return(rho, phi)"""

    ''' Function to return the highest tag id'''

    def highest_or_lowest_tag(self, ids, headHome):


        tagIndex = 0

        if (headHome == False):
            tagId = [0]
            for x in range(0, len(ids)):
                if ids[x] > tagId:
                    tagId = ids[x]
                    tagIndex = x
        elif (headHome == True):
            tagId = [100]
            for x in range(0, len(ids)):
                if ids[x] < tagId:
                    tagId = ids[x]
                    tagIndex = x

        return tagId, tagIndex

    ''' Used the list of highest IDs seen in the previous x number of frames and sets the highest ID and latest
  center pixel position as the target to fly to.'''

    def target_tag(self, headHome):
        targetPosition = []

        if (headHome == False):
            targetId = None
            for x in range(0, len(self.highestTagIdListWithPositions)):
                # print("x = ", x)
                # print(self.highestTagIdListWithPositions[x][0][0])
                if self.highestTagIdListWithPositions[x][0][0] >= targetId:
                    targetId = self.highestTagIdListWithPositions[x][0][0]
                    targetPosition = self.highestTagIdListWithPositions[x][1]

        elif (headHome == True):
            targetId = 100
            for x in range(0, len(self.lowestTagIdListWithPositions)):
                if self.lowestTagIdListWithPositions[x][0][0] <= targetId:
                    targetId = self.lowestTagIdListWithPositions[x][0][0]
                    targetPosition = self.lowestTagIdListWithPositions[x][1]

        return targetId, targetPosition


    def update_tag_lists(self):

            ''' Update higher tags list '''
            self.highestTagId, self.highestTagIndex = self.highest_or_lowest_tag(self.ids, False)
            self.highestTagCenterPixel = self.tag_center(self.corners[self.highestTagIndex][0])
            if len(self.highestTagIdListWithPositions) == 30:
                    self.highestTagIdListWithPositions.popleft()
            self.highestTagIdListWithPositions.append([self.highestTagId, self.highestTagCenterPixel])

            ''' Update lowest tags list '''
            self.lowestTagId, self.lowestTagIndex = self.highest_or_lowest_tag(self.ids, True)
            self.lowestTagCenterPixel = self.tag_center(self.corners[self.lowestTagIndex][0])
            if len(self.lowestTagIdListWithPositions) == 30:
                    self.lowestTagIdListWithPositions.popleft()
            self.lowestTagIdListWithPositions.append([self.lowestTagId, self.lowestTagCenterPixel])


    def bebop_hover(self):
        self.flightCmd.angular.z = 0
        self.flightCmd.linear.x = 0
        self.flightCmd.linear.y = 0
        self.flightCmd.linear.z = 0
        self.flight_pub.publish(self.flightCmd)

    def get_tag_position_from_id(self, requestedId):
        position = [428, 320]
        found = False
        for x in range(0, len(self.ids)):
            if self.ids[x] == requestedId:
                position = self.tag_center(self.corners[x][0])
                found = True
        return position, found

    
   
    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        global camera_angle, count, headHome
        #firstTagSeen = False
        tvec = np.empty([])
        rvec = np.empty([])
        highestTagIndex = 0
        highestTagId = 0
        lowestTagId = 100
        lowestTagIndex = 0
        badBoiTagId = 18
        landTag = 0
        cameraAngleBirdsEye = -70
        markerLength = 5
        count = count + 1

        ''' Load in the camera cooefficients from the calibration.yaml file in config folder'''
        mtx = rospy.get_param("~camera_matrix")
        dist = rospy.get_param("~dist_coeff")
        rvecs = rospy.get_param("~rvecs")
        tvecs = rospy.get_param("~tvecs")

        ''' Gray the image for Aruco detection '''
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(gray, arucoDict, parameters=parameters)

        self.publish_camera(cameraAngleBirdsEye)

        ''' This if statement lets us check if we have detected a tag by checking the size
         of the corners list to see if it's greater than 0. If it is, then we want to
        find the center position of the tag and then correct the camera angle to center
        it by publishing back a new angle to the control_camera topic.'''
        if (len(self.corners) != 0):
            count = 0
            #self.firstTagSeen = True
            ''' Draw on the markers so we can see they've been detected'''
            gray = aruco.drawDetectedMarkers(cv_image, self.corners, self.ids)
            ''' Calls the function to add the highest and lowest tags to their lists '''
            self.update_tag_lists()

            if (self.firstTagSeen == False):
                if (self.goToTag == 1):
                    self.targetTagPosition, found = self.get_tag_position_from_id(1)
                    print("targetTagPosition: ", self.targetTagPosition, "found: ", found)
                    if (found == True):
                        if ((abs(self.targetTagPosition[0] - 428) > 15) or (abs(self.targetTagPosition[1] - 320) > 15)):
                            self.flightCmd.linear.y = self.m_pidX.update(self.targetTagPosition[0], 428)
                            self.flightCmd.linear.x = self.m_pidY.update(self.targetTagPosition[1], 320)
                            self.flight_pub.publish(self.flightCmd)
                        else:   
                            if (self.highestTagId[0] <= 2):
                                print("Entered else in 1")
                                self.goToTag = 0
                                found = False
                                self.flightCmd.linear.y = 0
                                self.flightCmd.linear.x = 0
                                self.flightCmd.angular.z = 0.2
                                self.flight_pub.publish(self.flightCmd)
                            else:
                                self.firstTagSeen = True
                                print("First tag seen")
                                self.flightCmd.angular.z = 0
                                self.flight_pub.publish(self.flightCmd)

                elif (self.goToTag == 0):
                    self.targetTagPosition, found = self.get_tag_position_from_id(0)
                    self.flight_pub.publish(self.flightCmd)
                    
                    if (found == True):
                        if ((abs(self.targetTagPosition[0] - 428) > 10) or (abs(self.targetTagPosition[1] - 320) > 10)):
                            self.flightCmd.angular.z = 0
                            self.flightCmd.linear.y = self.m_pidX.update(self.targetTagPosition[0], 428)
                            self.flightCmd.linear.x = self.m_pidY.update(self.targetTagPosition[1], 320)
                            self.flight_pub.publish(self.flightCmd)
                        else:
                            self.goToTag = 2
                            found = False      
                            self.targetTagPosition, found = self.get_tag_position_from_id(2)
                            if (found == False):
                                self.flightCmd.angular.z = -0.2
                                self.flight_pub.publish(self.flightCmd)


                elif (self.goToTag == 2):
                    self.targetTagPosition, found = self.get_tag_position_from_id(2)
                    self.flight_pub.publish(self.flightCmd)
                    if (found == True):
                        if ((abs(self.targetTagPosition[0] - 428) > 10) or (abs(self.targetTagPosition[1] - 320) > 10)):
                            self.flightCmd.angular.z = 0
                            print("zeroed in 2 first if")
                            self.flightCmd.linear.y = self.m_pidX.update(self.targetTagPosition[0], 428)
                            self.flightCmd.linear.x = self.m_pidY.update(self.targetTagPosition[1], 320)
                            self.flight_pub.publish(self.flightCmd)
                        if (self.highestTagId[0] <= 2):
                            found = False
                            self.flightCmd.linear.y = 0
                            self.flightCmd.linear.x = 0
                            self.flightCmd.angular.z = 0.1
                            self.flight_pub.publish(self.flightCmd) 
                        else:
                            self.firstTagSeen = True
                            print("First tag seen")
                            self.flightCmd.angular.z = 0
                            print("zeroed in 2 else")
                            self.flight_pub.publish(self.flightCmd)


            if (headHome == False and self.firstTagSeen == True):
                print("TAG SEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEN")
                self.targetTagId, self.targetTagPosition = self.target_tag(headHome)
                ''' Check to see if the BadBoi tag is within 20 pixels square of the desired pixel position, if it is we have arrived and it's time
                to head back '''
                if((self.targetTagId == badBoiTagId) and (abs(self.targetTagPosition[0] - 428) < 20) and (abs(self.targetTagPosition[1] - 320) < 20)):
                    print("Reached BadBoi")
                    headHome = True
                    self.bebop_hover()
                    sleep(5)

            elif (headHome == True and self.firstTagSeen == True):
                self.targetTagId, self.targetTagPosition = self.target_tag(headHome)
                if(self.targetTagId == landTag and (abs(self.targetTagPosition[0] - 428) < 20) and (abs(self.targetTagPosition[1] - 320) < 20)):
                    drone_land()
                 

            #print(" ")
            #print("targetTagId: ", self.targetTagId, "targetTagPosition", self.targetTagPosition)


        ''' if to check if we've seen a tag in the last 30 frames '''
        if(count <= 30 and self.firstTagSeen == True):
            cv2.circle(cv_image, (int(self.targetTagPosition[0]), int(self.targetTagPosition[1])), 10, (0, 0, 255), -1)
            ''' Send the current value and the target value for the Y position of the tag to the PID function'''
            print("targetTagPosition: ", self.targetTagPosition[0], self.targetTagPosition[1])

            ''' Send the positions of the tag we wish to fly to to the PID update function to get new velocities '''
            self.flightCmd.linear.y = self.m_pidX.update(self.targetTagPosition[0], 428)
            self.flightCmd.linear.x = self.m_pidY.update(self.targetTagPosition[1], 320)
            print("x (forward) and y (side) commands",self.flightCmd.linear.x, self.flightCmd.linear.y)
            
            ''' if the target tag is within these boundaries then we've arrived here and we cannot see the next tag so we should rotate '''
            if(abs(self.targetTagPosition[0] - 428) < 25 and abs(self.targetTagPosition[1] - 320) < 25):
                self.flightCmd.angular.z = 0.1

            else:
                self.flightCmd.angular.z = 0
                print("zeroed in count<= 30 first tage seen==true")
                self.flightCmd.linear.z = 0
        
            self.flight_pub.publish(self.flightCmd)

        elif (count > 30 and count <= 200 and self.firstTagSeen == True):
            self.flightCmd.angular.z = 0
            print("zeroed in count>30 count<=200")
            self.flightCmd.linear.x = 0
            self.flightCmd.linear.y = 0
            self.flightCmd.linear.z = 0
            self.flight_pub.publish(self.flightCmd)

        elif (count > 200 and self.firstTagSeen == True):
            self.flightCmd.angular.z = 0.1
            self.flightCmd.linear.x = 0
            self.flightCmd.linear.y = 0
            self.flightCmd.linear.z = 0
            self.flight_pub.publish(self.flightCmd)


        cv2.line(cv_image, (0, 320), (856, 320), 255, 2)
        cv2.line(cv_image, (428, 0), (428, 480), 255, 2)
        cv2.rectangle(cv_image, (403, 295), (453, 345), 255, 2)

        '''Display the video feed frames every 3 ms.'''
        cv2.imshow("Image window", gray)
        cv2.waitKey(5)  # 5

        ''' Publish the image back into ROS image message type (not sure why, I guess it's if you
        want to do something else with it after using OpenCV).'''
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except CvBridgeError as e:
            print(e)




# Setup an initial camera angle
#cameraAngle = -70
count = 0
headHome = False



def main(args):
    ''' Initialise the node under the name image_converter'''
    rospy.init_node('image_converter', anonymous=True)

    ''' wait for the message from the BadBoi to takeoff '''
    '''badboiClassCall = badboi_message_class()

    while 1:
        badboiClassCall.badboi_caller() 
        if badboiClassCall.badboiMsgReceived == "takeoff":
            print("Message received, now taking off")
            break'''

    drone_takeoff()
    alt = altitude_class(1.7)
    print("Aquiring altitude")
    alt.go_to_altitude()
    print("Altitude aquired")
    sleep(2)

    ic = image_converter()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown() and headHome == True:
        badboiClassCall.bebop_send()        
    rate.sleep()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
