#!/usr/bin/env python

import rospy
from robotics_project.msg import objectPose
from robotics_project.srv import *
from code import interact
from math import copysign

class ControllerNode():
    def __init__(self):
        rospy.init_node('controller_node')
        self.drive_request = rospy.ServiceProxy('requestDrive', requestDrive)
        self.angle_request = rospy.ServiceProxy('requestAngle', requestAngle)
        self.distance_request = rospy.ServiceProxy('driveDist', driveDist)
        self.pose_subscriber = rospy.Subscriber(
                "/camera_node/objectPose",
                objectPose,
                self.handle_incoming_pose
        )
        rospy.sleep(5)
        print "playing soccer"
        self.play_soccer()
        rospy.spin()

    def handle_incoming_pose(self, objectPose):
        self.objectPose = objectPose
        self.objectPose_dict = {
                'ball_in_view':objectPose.ball_in_view,
                'ball_center_x':objectPose.ball_center_x,
                'goal_in_view':objectPose.goal_in_view,
                'goal_center_x':objectPose.goal_center_x,
                'ball_distance':objectPose.ball_distance
        }

    def get_object_in_view(self, object_in_view):
        while not self.objectPose_dict[object_in_view]: 
            self.drive_robot(0, 50)
            rospy.sleep(0.1)
        rospy.sleep(0.5)
        print "ball is in view"
        self.drive_robot(0, 0)

    def center_object(self, object_center):
        offset = self.objectPose_dict[object_center] - 320
        while abs(offset) > 20:
            offset = self.objectPose_dict[object_center] - 320
            #turn_rate = max([abs(offset)/(320/50), 25])
            turn_rate = 40
            self.drive_robot(0, turn_rate)
        print "centered ball, sending stop command"
        self.drive_robot(0, 0)

    def play_soccer(self):
        #find the ball
        print "Finding ball..."
        self.get_object_in_view('ball_in_view')
        print "Ball in view, centering..."
        self.center_object('ball_center_x')
        print "Ball found."
        #find the angle between the ball and the goal
        print "Zeroing angle measurement..."
        angle = self.request_angle()
        print "Angle zeroed, finding goal..."
        self.get_object_in_view('goal_in_view')
        print "Goal in view, centering..."
        self.center_object('goal_center_x')
        print "Goal found."
        angle = self.request_angle()
        print "Angle between ball and goal: ", angle
        self.get_object_in_view('ball_in_view')
        self.center_object('ball_center_x')
        drive_success = self.drive_distance(self.objectPose.ball_distance)
        print drive_success

        #put the ball between the robot and the goal

        #push the ball into the goal

    def drive_robot(self, velocity, rotation):
        rospy.wait_for_service('requestDrive')
        try:
            self.drive_request(velocity, rotation)
        except rospy.ServiceException, e:
            print e

    def drive_distance(self, distance):
        rospy.wait_for_service('driveDist')
        try:
            response = self.distance_request(distance)
        except rospy.ServiceException, e:
            print e
        return response

    def request_angle(self):
        rospy.wait_for_service('requestAngle')
        try:
            angle = self.angle_request()
        except rospy.ServiceException, e:
            print e
        return angle

    def build_model(self, objectPose):
        """Calculate world coordinates of ball and goal, robot is (0,0)"""
        if objectPose.ball_in_view:
            ball_distance = objectPose.ball_distance
            ball_offset = objectPose.ball_center_x
            ball_coords = calculate_object_coords(ball_distance, ball_offset)
        if objectPose.goal_in_view:
            goal_distance = objectPose.goal_distance
            goal_offset = objectPose.goal_center_x
            goal_coords = calculate_object_coords(goal_distance, goal_offset)

    def calculate_object_coords(self, distance, offset): 
        pass

if __name__ == "__main__":
    controller_node = ControllerNode()
