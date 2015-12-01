#!/usr/bin/env python

import rospy
from robotics_project.msg import objectPose
from robotics_project.srv import *
from code import interact
from math import *
from copy import copy

class ControllerNode():
    def __init__(self):
        rospy.init_node('controller_node')
        self.drive_request = rospy.ServiceProxy('requestDrive', requestDrive)
        self.angle_request = rospy.ServiceProxy('requestAngle', requestAngle)
        self.distance_request = rospy.ServiceProxy('driveDist', driveDist)
        self.turn_request = rospy.ServiceProxy('turnAngle', turnAngle)
        self.strike_request= rospy.ServiceProxy('requestStrike', requestStrike)
        self.pose_subscriber = rospy.Subscriber(
                "/camera_node/objectPose",
                objectPose,
                self.handle_incoming_pose
        )
        rospy.sleep(5)
        print "playing soccer"
        self.play_soccer()
        #self.test_angles()
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

    def get_behind_ball(self, goal_dist, ball_dist, angle, desired_dist):
        behind_angle, behind_dist = calc_info_for_plan_nate1(goal_dist, ball_dist, angle, desired_dist)
        return 45.0, 36.0

    def calc_info_for_plan_nate1(self, goal_dist, ball_dist, angle, desired_dist):
        # plan "nate1": get behind the ball at a 45 degree angle
        # (so angle goal-ball-robot is 135 degrees) and 30" 
        # from the ball
        a = goal_dist
        b = ball_dist
        d = desired_dist
        theta = angle * (math.pi / 180)
        c = math.sqrt( (a*a)+(b*b)+(2*a*b*math.cos(theta))
        rho = math.acos( ((a*a)-(b*b)-(c*c)) / (2*a*c) )
        eta = 3*math.pi / 4
        x = math.sqrt( (b*b)+(d*d)+(2*b*d*math.cos(eta)) )
        behind_angle = math.acos( ((d*d)-(b*b)-(x*x)) / (2*b*x) )
        behind_dist = x
        return behind_angle, behind_dist

    def test_angles(self):
        angle1 = self.request_angle()
        print "Zeroing angle... ", angle1
        self.drive_robot(0, 75)
        print "turning robot"
        rospy.sleep(2)
        print "stopping robot"
        self.drive_robot(0, 0)
        angle2 = self.request_angle()
        print "angle turned: ", angle2

    def play_soccer(self):
        print "Finding Goal"
        self.get_object_in_view('goal_in_view')
        print "Goal in view, centering..."
        self.center_object('goal_center_x')
        print "Goal found."
        goal_dist = copy(self.objectPose.goal_distance)
        rospy.sleep(1.0)
        print "Zeroing angle measurement..."
        angle = self.request_angle()
        print "Finding ball..."
        self.get_object_in_view('ball_in_view')
        print "Ball in view, centering..."
        self.center_object('ball_center_x')
        print "Ball found."
        ball_dist = copy(self.objectPose.ball_distance)
        angle = self.request_angle()
        desired_dist = 30
        print "Angle between ball and goal: ", angle
        behind_angle, behind_dist = self.get_behind_ball(goal_dist, ball_dist, angle, desired_dist)
        print "going to turn ", behind_angle
        self.turn_angle(behind_angle)
        print "going to move ", behind_dist
        self.drive_distance(behind_dist)
        print "Now behind ball."
        self.get_object_in_view('ball_in_view')
        self.center_object('ball_center_x')
        print "Approaching ball..."
        drive_success = self.drive_distance(self.objectPose.ball_distance - 12)
        print drive_success
        print "Striking!"
        self.request_strike()

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

    def turn_angle(self, degrees):
        rospy.wait_for_service('turnAngle')
        try:
            response = self.turn_request(degrees)
        except rospy.ServiceException, e:
            print e
        return response

    def request_strike(self):
        rospy.wait_for_service('requestStrike')
        try:
            self.strike_request()
        except rospy.ServiceException, e:
            print e

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
