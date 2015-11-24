#!/usr/bin/env python

import rospy
import cv2
from robotics_project.srv import *
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import colorsys
from code import interact

class CameraNode():
    def __init__(self):
        """Start camera_node and setup publishers/subscribers"""
        self.bridge = CvBridge()
        self.testing = False
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

    def _threshold_image(self, hsv_image, rgb_color):
        """
        Threshold an hsv image on a given rgb_color

        hsv_image -- a cv2 hsv image
        rgb_color -- a tuple of the form (r, g, b)
        Returns a binary mask of the same size as hsv_image which is 1 where
        the color is within dev of rgb_color and 0 elsewhere.
        """
        dev = 100 
        #hsv_color = colorsys.rgb_to_hsv(*rgb_color)
        hsv_color = (175, 150, 141)
        #hsv_lower = tuple([v - dev for v in hsv_color])
        #hsv_upper = tuple([v + dev for v in hsv_color])
        hsv_lower = (160, 10, 10)
        hsv_upper = (180, 250, 250)
        binary_image = cv2.inRange(hsv_image, hsv_lower, hsv_upper)
        return binary_image

    def _process_image(self, hsv_image):
        """Do some processing of hsv_image. Specifics will likely change."""
        crimson = (220, 20, 60)
        mask = self._threshold_image(hsv_image, crimson)

        kernel = np.ones((50,50),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        kernel = np.ones((80,80),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        masked_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        return mask, masked_image

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
        mask, masked_image = self._process_image(hsv_image)
        center = self._find_center(mask)
        if (center is not None) and (self.testing == True):
            velocity, rotation = self._follow_ball(center, (640, 480))
            print velocity, rotation
            self.drive_robot(velocity, rotation)
        rgb_image = cv2.cvtColor(masked_image, cv2.COLOR_HSV2BGR)
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
