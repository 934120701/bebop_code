#!/usr/bin/env python

'''
This program flys a Bebop drone by following ArUco tags to a maximum ID before publishing a message
for the grond robot. Flys in reverse and lands at a given tag. 
'''

from bebop_functions import *
""" Class that takes the images from the Bebop and processes the locations of the ArUco tags
using them to feed velocity commands back to the drone """
class image_converter:

    def __init__(self,s):
        ''' Create the publishers and subscribers '''
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
        self.highestTagIndex = int
        self.lowestTagId = int
        self.firstTagSeen = False
        self.atStartTag = False
        self.foundPreviously = False
        self.keepRotating = False
        self.goToTag = 0
        self.tagZeroVisitTimes = 0
        self.videoTargetPixelX = 428#428
        self.videoTargetPixelY = 400 #400 #320 previously
        self.s = s
        self.count = 0
        self.count2 = 0
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

    ''' publish_camera is used to change the angle of the camera in the Y direction '''
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

    ''' Function to return the highest or lowest tag id depending on if the drone is heading
    to the BadBoi or the home area '''
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


    ''' updates the two lists for the highest and lowest number tags seen in the past 30 frames '''
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

    ''' Checks the past 11 frames to see if we've seen a tag dropped by the BadBoi '''
    def check_frames_for_badboi_tag(self):

        currentHighest = 0
        gotABadBoiTag = False
        for x in range(0, 10):
            if (self.highestTagIdListWithPositions[20+x][0][0] > currentHighest):
                currentHighest = self.highestTagIdListWithPositions[20+x][0][0]
        if currentHighest > 2:
            gotABadBoiTag = True
        return gotABadBoiTag

    ''' Hovers the drone '''
    def bebop_hover(self):

        self.flightCmd.angular.z = 0
        self.flightCmd.linear.x = 0
        self.flightCmd.linear.y = 0
        self.flightCmd.linear.z = 0
        self.flight_pub.publish(self.flightCmd)

    ''' Function that returns the position of the requested tag and whether the requested tag was found '''
    def get_tag_position_from_id(self, requestedId):
        position = [self.videoTargetPixelX, self.videoTargetPixelY]
        found = False
        for x in range(0, len(self.ids)):
            if self.ids[x] == requestedId:
                position = self.tag_center(self.corners[x][0])
                found = True
        return position, found

    ''' Function that sets and publishs the velocity commands based on target tag position '''
    def update_and_publish_pid_flight(self):
        self.flightCmd.angular.z = 0
        self.flightCmd.linear.y = self.m_pidY.update(self.targetTagPosition[0], self.videoTargetPixelX)
        self.flightCmd.linear.x = self.m_pidX.update(self.targetTagPosition[1], self.videoTargetPixelY)
        self.flight_pub.publish(self.flightCmd)

    ''' Stops the drone and commands it to rotate counterclockwise '''
    def stop_and_rotate(self):
        self.flightCmd.linear.y = 0
        self.flightCmd.linear.x = 0
        self.flightCmd.angular.z = 0.3
        self.flight_pub.publish(self.flightCmd)

    ''' Draws the overlay on the image with the crosshairs '''
    def draw_overlay(self, cv_image):
        cv2.line(cv_image, (0, self.videoTargetPixelY), (856, self.videoTargetPixelY), 255, 2)
        cv2.line(cv_image, (self.videoTargetPixelX, 0), (self.videoTargetPixelX, 480), 255, 2)
        cv2.rectangle(cv_image, (self.videoTargetPixelX - 25, self.videoTargetPixelY - 25), (self.videoTargetPixelX + 25, self.videoTargetPixelY + 25), 255, 2)

   
    ''' The callback function from the bebop/image_raw subscription. Calculates and publishes new commands each frame if 
    necessary '''
    def callback(self, data):

        # Obtain the image using cv_bridge so we can use OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set some method specific variables
        global camera_angle,headHome#, count2, count
        tvec = np.empty([])
        rvec = np.empty([])
        highestTagIndex = 0
        lowestTagId = 100
        lowestTagIndex = 0
        badBoiTagId = 18
        landTag = 0
        cameraAngleBirdsEye = -70
        markerLength = 5
        self.count = self.count + 1
        

        ''' Load in the camera cooefficients from the calibration.yaml file in config folder '''
        mtx = rospy.get_param("~camera_matrix")
        dist = rospy.get_param("~dist_coeff")
        rvecs = rospy.get_param("~rvecs")
        tvecs = rospy.get_param("~tvecs")

        ''' Gray the image for Aruco detection and detect visible tags '''
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(gray, arucoDict, parameters=parameters)

        self.publish_camera(cameraAngleBirdsEye)

        if (self.firstTagSeen == False):
            self.count2 = self.count2 + 1
        else: 
            self.count2 = 0
        if (self.keepRotating == True):
            self.stop_and_rotate()

        ''' This if statement lets us check if we have detected a tag by checking the size
         of the corners list to see if it's greater than 0. If it is, then we want to
        find the center position of the tag and then correct the camera angle to center
        it by publishing back a new angle to the control_camera topic. '''
        if (len(self.corners) != 0 or self.count2 > 800):
            self.count = 0

            ''' Draw on the markers so we can see they've been detected'''
            gray = aruco.drawDetectedMarkers(cv_image, self.corners, self.ids)
            ''' Calls the function to add the highest and lowest tags to their lists '''
            self.update_tag_lists()
            print(" ")

            # Check to see if we've seen a tag dropped by the BadBoi
            if (self.firstTagSeen == False):
                if (self.goToTag == 1):
                    self.targetTagPosition, found = self.get_tag_position_from_id(1)
                    if (self.keepRotating == True):
                        self.stop_and_rotate()
                    # Check to see if we can see the tag we want to fly to
                    if (found == True):
                        # If we can, are we already over it
                        if (self.atStartTag == False):
                            # If we're not over it, get us within 20x20 pixels of the tag center
                            if ((abs(self.targetTagPosition[0] - self.videoTargetPixelX) > 20) or (abs(self.targetTagPosition[1] - self.videoTargetPixelY) > 20)):
                                self.update_and_publish_pid_flight()
                                cv2.circle(cv_image, (int(self.targetTagPosition[0]), int(self.targetTagPosition[1])), 10, (0, 0, 255), -1)
                                self.count2 = 0
                            # If we're wihin 20x20 pixels, set the self.atStartTag variable to true so we know there's no need to navigate to it
                            else:
                                self.atStartTag = True
                        # If we're within 20x20 pixels, start to rotate
                        else:   
                            self.stop_and_rotate()
                            self.keepRotating = True
                            # Check to see if we've seen a BadBoi tag within the last 11 frames
                            if (self.check_frames_for_badboi_tag() == False):
                                
                                # If we haven't and we have rotate over a period of 200 frames, head back to tag 0
                                if (self.count2 > 600):
                                    self.goToTag = 0
                                    found = False
                                    self.keepRotating = False
                                    self.atStartTag = False
                                    self.count2 = 0
                                
                            # If we have seen a BadBoi tag, set self.firstTagSeen to true and we can begin tracking tags
                            else:
                                self.firstTagSeen = True
                                self.bebop_hover()
                    # If we can't see the tag we want to fly to, rotate
                    elif (found == False):
                        self.stop_and_rotate()


                # Similar method to the above block
                elif (self.goToTag == 0):
                    self.targetTagPosition, found = self.get_tag_position_from_id(0)
                    self.flight_pub.publish(self.flightCmd)
                    
                    if (found == True):
                        if ((abs(self.targetTagPosition[0] - self.videoTargetPixelX) > 20) or (abs(self.targetTagPosition[1] - self.videoTargetPixelY) > 20)):
                            self.flightCmd.angular.z = 0
                            self.update_and_publish_pid_flight()
                            cv2.circle(cv_image, (int(self.targetTagPosition[0]), int(self.targetTagPosition[1])), 10, (0, 0, 255), -1)
                        # If we're at 0, decide whether we need to go to tag 1 or 2
                        else:
                            self.tagZeroVisitTimes = self.tagZeroVisitTimes + 1

                            if (self.tagZeroVisitTimes % 2 == 0):
                                self.goToTag = 2
                            else:
                                self.goToTag = 1

                            found = False  
                            self.foundPreviously = False    
                            self.targetTagPosition, found = self.get_tag_position_from_id(2)
                            if (found == False):
                                self.flightCmd.angular.z = -0.3
                                self.flight_pub.publish(self.flightCmd)
                    elif (found == False):
                        self.stop_and_rotate()


                # Same method employed in the block for flying to tag 1
                elif (self.goToTag == 2):
                    self.targetTagPosition, found = self.get_tag_position_from_id(2)
                    if (self.keepRotating == True):
                        self.stop_and_rotate()
                    self.flight_pub.publish(self.flightCmd)
                    if (found == True):
                        if(self.atStartTag == False):
                            if ((abs(self.targetTagPosition[0] - self.videoTargetPixelX) > 20) or (abs(self.targetTagPosition[1] - self.videoTargetPixelY) > 20)):
                                self.flightCmd.angular.z = 0
                                self.update_and_publish_pid_flight()
                                cv2.circle(cv_image, (int(self.targetTagPosition[0]), int(self.targetTagPosition[1])), 10, (0, 0, 255), -1)
                                self.count2 = 0
                            else:
                                self.atStartTag = True
                                self.stop_and_rotate()
                        
                        else:
                            if (self.check_frames_for_badboi_tag() == False):
                                if (self.count2 > 600):
                                    self.count2 = 0
                                    found = False
                                    self.keepRotating = False
                                    self.atStartTag = False
                                    self.goToTag = 0
                                    self.stop_and_rotate() 
                            else:
                                self.firstTagSeen = True
                                self.bebop_hover()
                    elif (found == False):
                        self.stop_and_rotate()


            # If we've seen the first tag and we're heading to the BadBoi, check if we're over it already 
            if (headHome == False and self.firstTagSeen == True):
                self.targetTagId, self.targetTagPosition = self.target_tag(headHome)
                ''' Check to see if the BadBoi tag is within 20 pixels square of the desired pixel position, if it is we have arrived and it's time
                to head back '''
                # ARRIVE AT BADBOI CHECK
                if((self.targetTagId == badBoiTagId) and (abs(self.targetTagPosition[0] - self.videoTargetPixelX) < 20) and (abs(self.targetTagPosition[1] - self.videoTargetPixelY) < 20)):
                    headHome = True
                    self.bebop_hover()
                    sleep(3)

            # LANDING
            elif (headHome == True and self.firstTagSeen == True):
                self.targetTagId, self.targetTagPosition = self.target_tag(headHome)
                if(self.targetTagId == landTag and (abs(self.targetTagPosition[0] - self.videoTargetPixelX) < 20) and (abs(self.targetTagPosition[1] - self.videoTargetPixelY) < 20)):
                    send_msg_to_badboi(self.s)
                    drone_land()
                 

        ''' if to check if we've seen a tag in the last 30 frames '''
        if(self.count <= 30 and self.firstTagSeen == True):
            cv2.circle(cv_image, (int(self.targetTagPosition[0]), int(self.targetTagPosition[1])), 10, (0, 0, 255), -1)
            ''' Send the current value and the target value for the Y position of the tag to the PID function'''
            '''if (headHome == True and self.targetTagId <= 2):
                altitudeCheck = altitude_class(1.6)
                altitudeCheck.go_to_altitude()'''

            ''' Send the positions of the tag we wish to fly to to the PID update function to get new velocities '''
            self.flightCmd.linear.y = self.m_pidX.update(self.targetTagPosition[0], self.videoTargetPixelX)
            self.flightCmd.linear.x = self.m_pidY.update(self.targetTagPosition[1], self.videoTargetPixelY)
            print("x (forward) and y (side) commands",self.flightCmd.linear.x, self.flightCmd.linear.y)
            
            ''' if the target tag is within these boundaries then we've arrived here and we cannot see the next tag so we should rotate '''
            if(abs(self.targetTagPosition[0] - self.videoTargetPixelX) < 25 and abs(self.targetTagPosition[1] - self.videoTargetPixelY) < 25):
                self.flightCmd.angular.z = 0.3
            # If not, stop rotating and stop gaining any height
            else:
                self.flightCmd.angular.z = 0
                self.flightCmd.linear.z = 0
        
            self.flight_pub.publish(self.flightCmd)

        # If we haven't seen a tag in a while, just hover a second (could be we temporarily lose it with light reflections)
        elif (self.count > 30 and self.count <= 200 and self.firstTagSeen == True):
            self.bebop_hover()

        # If we haven't seen a tag in a long while, rotate to see if we can see one
        elif (self.count > 200): # and self.firstTagSeen == True):
            self.stop_and_rotate()


        self.draw_overlay(cv_image)

        '''Display the video feed frames every 3 ms.'''
        cv2.imshow("Image window", gray)
        cv2.waitKey(5)  # 5

        ''' Publish the image back into ROS image message type (not sure why, I guess it's if you
        want to do something else with it after using OpenCV).'''
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except CvBridgeError as e:
            print(e)

headHome = False

def main(args):
    ''' Initialise the node under the name image_converter'''
    rospy.init_node('image_converter', anonymous=True)

    s = setup_client()
    heard_from_badboi(s)


    drone_takeoff()
    alt = altitude_class(1.6)
    print("Aquiring altitude")
    alt.go_to_altitude()
    print("Altitude aquired")
    sleep(2)

    ic = image_converter(s)
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
