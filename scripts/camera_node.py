#!/usr/bin/env python

import rospy
import cv2
from robotics_project.srv import *
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import colorsys
from code import interact
from collections import deque
from robotics_project.msg import objectPose

drawBall = True
drawGoal = True

#ball_hsv_color = (179, 184, 143)
ball_hsv_color = (1, 162, 132)
#ball_threshold = (50, 70, 70)
ball_threshold = (8, 20, 20)
goal_hsv_color = (35, 98, 135)
goal_threshold = (5, 5, 5)
goal_num_frames_to_ave = 20
openKernSizeForClose = 80
openKernSizeForFar = 40
closeKernSizeForClose = 50
closeKernSizeForFar = 30
openKernSizeForGoal = 1
closeKernSizeForGoal = 1

class CameraNode():
    def __init__(self):
        """Start camera_node and setup publishers/subscribers"""
        self.numPixelsToBelieveGoalIsInView = 5000
        self.bridge = CvBridge()
        self.testing = False
        self.ballWidthList = deque([0]*10)
        self.ballWidth = 0
        self.goalLeftList = deque([0]*goal_num_frames_to_ave)
        self.goalRightList = deque([0]*goal_num_frames_to_ave)
        self.goalTopList = deque([0]*goal_num_frames_to_ave)
        self.goalBotList = deque([0]*goal_num_frames_to_ave)
        self.goalWidthList = deque([0]*goal_num_frames_to_ave)
        self.goalLeft = 0
        self.goalRight = 0
        self.goalBot = 0
        self.goalTop = 0
        self.goalWidth = 0
        self.objectPosePub = rospy.Publisher("/camera_node/objectPose", 
                                            objectPose, queue_size = 10)
        self.image_pub = rospy.Publisher("/camera_node/processed_image",
                                         Image,
                                         queue_size = 10)
        rospy.init_node('camera_node')
        self.camera_subscriber = rospy.Subscriber(
                "/camera/visible/image",
                Image,
                self._handle_incoming_image
        )
        rospy.spin()

    def _convert_raw_2_hsv(self, raw_ros_image):
        """Convert a ROS image message into cv2 bgr8 format"""
        cv_image = self.bridge.imgmsg_to_cv2(raw_ros_image, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 
        return hsv_image

    # function to find an object of the specified color and threshold
    # frame must be in hsv
    def findObject(self, frame, color, threshold, closeKernSize, openKernSize):
        frame = self._threshold_image(frame, color, threshold)
        # close and open to get rid of noise and unify object
        kernel = np.ones((closeKernSize,closeKernSize),np.uint8)
        frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
        kernel = np.ones((openKernSize,openKernSize),np.uint8)
        frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
        # find contours
        contour_struct = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        contours = contour_struct[0]
        if len(contours)>0:
                x, y, w, h = cv2.boundingRect(contours[0])  # <<----  This assumes there is only 1 contour
                return [x, y, w, h, frame]
        else:
                return [-1, -1, -1, -1, -1]

    # special function to find the goal specifically
    def findGoal(self, frame, color, threshold, closeKernSize, openKernSize):
        frame = self._threshold_image(frame, color, threshold)
        numPixelsOfCorrectColor = frame.sum()
        nonzeroRows, nonzeroCols = np.nonzero(frame)
        if numPixelsOfCorrectColor < self.numPixelsToBelieveGoalIsInView:
            return [-1, -1, -1, -1, frame]
        if len(nonzeroRows) !=0:
            top = min(nonzeroRows)
            bot = max(nonzeroRows)
            left = min(nonzeroCols)
            right = max(nonzeroCols)
            return [left, top, right-left, bot-top, frame]
        return [-1, -1, -1, -1, frame]

    # function to return binary image of color with tolerance threshold
    def _threshold_image(self, hsv_image, color, threshold):
        hsv_upper = (color[0]+threshold[0], color[1]+threshold[1], color[2]+threshold[2])
        hsv_lower = (color[0]-threshold[0], color[1]-threshold[1], color[2]-threshold[2])
        binary_image = cv2.inRange(hsv_image, hsv_lower, hsv_upper)
        return binary_image

    # function to calculate distance from width for the ball
    def _calcBallDist(self, bw):
        if bw > 0:
            return (4997 * (bw**(-0.95)))
        else:
            return -1

    # function to calculate distance from width for the goal ############# <--- Still needs to be done!!!!
    def _calcGoalDist(self, gw):
        if gw!=-1:
            return (13525 * (gw**(-1.24)))
        else:
            return -1
        
    # function to find ball and goal and transmit the objectPose message to topic objectPose 
    def _process_image(self, hsv_image):
        # find ball
        bx, by, bw, bh, bMask = self.findObject(hsv_image, ball_hsv_color, ball_threshold, 
                                         openKernSizeForClose, closeKernSizeForFar)
        self.ballWidthList.append(bw)
        self.ballWidthList.popleft()
        self.ballWidth = round(sum(self.ballWidthList)/10.0)
        # find goal
        gx, gy, gw, gh, gMask = self.findGoal(hsv_image, goal_hsv_color, goal_threshold, 
                                         openKernSizeForGoal, closeKernSizeForGoal)
        #gx, gy, gw, gh, gMask = self.findObject(hsv_image, goal_hsv_color, goal_threshold, 
        #                                 openKernSizeForGoal, closeKernSizeForGoal)
        # update goal estimates
        #self.goalWidthList.append(gw)
        #self.goalWidthList.popleft()
        #self.goalWidth = round(sum(self.goalWidthList)/10.0)
        self.goalTopList.append(gy)
        self.goalTopList.popleft()
        self.goalTop = max(self.goalTopList)
        self.goalBotList.append(gy+gh)
        self.goalBotList.popleft()
        self.goalBot = min(self.goalBotList)
        self.goalLeftList.append(gx)
        self.goalLeftList.popleft()
        self.goalLeft = max(self.goalLeftList)
        self.goalRightList.append(gx+gw)
        self.goalRightList.popleft()
        self.goalRight = min(self.goalRightList)
        # create and populate objectPose message
        objectPoseMessage = objectPose() 
        objectPoseMessage.ball_center_x = round(bx + (bw/2))
        objectPoseMessage.ball_width = self.ballWidth
        objectPoseMessage.ball_distance = self._calcBallDist(self.ballWidth)
        objectPoseMessage.goal_center_x = round((self.goalLeft+self.goalRight)/2)
        objectPoseMessage.goal_width = self.goalRight - self.goalLeft
        objectPoseMessage.goal_distance = self._calcGoalDist(gw)
        if bx != -1:
            objectPoseMessage.ball_in_view = 1
        else:
            objectPoseMessage.ball_in_view = 0
        if gx != -1:
            objectPoseMessage.goal_in_view = 1
        else:
            objectPoseMessage.goal_in_view = 0
        # publish objectPose message
        self.objectPosePub.publish(objectPoseMessage)
        # draw ball rectangle on image
        if drawBall:            
            bPoint1, bPoint2 = (bx, by), (bx+bw, by+bh)
            #cv2.rectangle(masked_image, bPoint1, bPoint2, [255, 255, 255], 2)
            cv2.rectangle(hsv_image, bPoint1, bPoint2, [255, 255, 255], 2)
        # draw goal rectangle on image
        if drawGoal:
            gPoint1, gPoint2 = (gx, gy), (gx+gw, gy+gh)
            #cv2.rectangle(masked_image, bPoint1, bPoint2, [255, 255, 255], 2)
            cv2.rectangle(hsv_image, gPoint1, gPoint2, [255, 255, 255], 2)
        return hsv_image
        #return cv2.bitwise_and(hsv_image, hsv_image, mask = gMask)

    def _find_center(self, mask):
        contours, heirarchy = cv2.findContours(mask,
                                               cv2.RETR_LIST,
                                               cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            primary_contour = contours[0]
            moments = cv2.moments(primary_contour)
            centroid_x = int(moments['m10']/moments['m00'])
            centroid_y = int(moments['m01']/moments['m00'])
            centroid = (centroid_x, centroid_y)
            return centroid
    
    def _follow_ball(self, center, imsize):
        image_center = [val/2.0 for val in imsize]
        offset = [cent -  mid for cent, mid in zip(center, image_center)]
        x_offset = offset[0]
        scaled_deviation = (x_offset / image_center[0]) * 200
        rotation = scaled_deviation
        velocity = 0
        return velocity, rotation

    def _handle_incoming_image(self, raw_ros_image):
        """Convert and process an incoming ROS image"""
        hsv_image = self._convert_raw_2_hsv(raw_ros_image)
        # objectPose topic published in _process_image
        processed_image = self._process_image(hsv_image)
        #center = self._find_center(mask)
        #if (center is not None) and (self.testing == True):
        #    velocity, rotation = self._follow_ball(center, (640, 480))
        #    print velocity, rotation
        #    self.drive_robot(velocity, rotation)
        #cv2.rectangle(hsv_image, gPoint1, gPoint2, [255, 255, 255], 2)
        rgb_image = cv2.cvtColor(processed_image, cv2.COLOR_HSV2BGR)
        ros_image = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
        self.image_pub.publish(ros_image)

    def drive_robot(self, velocity, rotation):
        rospy.wait_for_service('requestDrive')
        try:
            service_request = rospy.ServiceProxy('requestDrive', requestDrive)
            response = service_request(velocity, rotation)
        except rospy.ServiceException, e:
            print e


if __name__ == "__main__":
    camera_node = CameraNode()
