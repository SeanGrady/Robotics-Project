#!/usr/bin/env python

import serial
import rospy

def DriveNode():
    def __init__(self):
        self.connection = None
        self.drive_struct = struct.Struct('>Bhh')
        self.port = '/dev/ttyUSB0'
        rospy.init_node('DriveNode')
        self.drive_service = rospy.Service('requestDrive', requestDrive,
                                 self.handle_requestDrive)
        self.angle_service = rospy.Service('requestAngle', requestAngle,
                                           self.handle_requestAngle)
        self.connect_robot()
        rospy.spin()

    def send_drive_command(self, cmd):
        self.connection.write(cmd)
        
    def make_drive_command(self, vel, rot):
        #this is to keep vl and vr between -500 and 500 
        vl = sorted([-500, vel + rot, 500])[1]
        vr = sorted([-500, vel - rot, 500])[1]
        cmd = self.drive_struct.pack(145, vr, vl)
        return cmd
        
    def connect_robot(self):
        if self.connection is not None:
            print "Already connected!"
            return
        self.connection = serial.Serial(
                self.port,
                baudrate=115200,
                timeout=1
        )
        self.connection.write(self.command_dict['start'])
        self.connection.write(self.command_dict['safe'])

    def handle_requestDrive(self, request):
        vel = request.velocity
        rot = request.rotation
        drive_command = self.make_drive_command(vel, rot)
        self.send_drive_command(drive_command)

    def handle_requestAngle(self, request):
        
