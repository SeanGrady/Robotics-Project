#!/usr/bin/env python

import serial
import rospy
import struct
from robotics_project.srv import *

class DriveNode():
    def __init__(self):
        self.connection = None
        self.drive_struct = struct.Struct('>Bhh')
        self.angle_struct = struct.Struct('>BB')
        self.angle_request = self.make_angle_request()
        self.port = '/dev/ttyUSB0'
        self.command_dict = {
            'start':self.make_raw_command('128'),
            'safe':self.make_raw_command('131'),
            'passive':self.make_raw_command('128'),
            'full':self.make_raw_command('132'),
            'beep':self.make_raw_command('140 3 1 64 16 141 3'),
            'stop':self.make_raw_command('173'),
            'dock':self.make_raw_command('143'),
            'reset':self.make_raw_command('7')
        }
        rospy.init_node('drive_node')
        self.drive_service = rospy.Service('requestDrive', requestDrive,
                                 self.handle_requestDrive)
        self.angle_service = rospy.Service('requestAngle', requestAngle,
                                           self.handle_requestAngle)
        self.strike_service = rospy.Service('requestStrike', requestStrike,
                                            self.strike_forward)
        self.connect_robot()
        rospy.spin()

    def strike_forward(self):
        vel = 500
        rot = 0
        strike_cmd = self.make_drive_command(vel, rot)
        stop_cmd = self.make_drive_command(0,0)
        print "Strike_cmd", strike_cmd
        print "stop_cmd", stop_cmd
        self.connection.write(strike_cmd)
        rospy.sleep(.7)
        self.connection.write(stop_cmd)
        return "Striking"

    def make_drive_command(self, vel, rot):
        #this is to keep vl and vr between -500 and 500 
        vl = sorted([-500, vel + rot, 500])[1]
        vr = sorted([-500, vel - rot, 500])[1]
        cmd = self.drive_struct.pack(145, vr, vl)
        return cmd

    def make_raw_command(self, string):
        cmd = ''
        for num in string.split():
            cmd += chr(int(num))
        print cmd
        return cmd
        
    def connect_robot(self):
        if self.connection is not None:
            print "Already connected!"
            return
        try:
            self.connection = serial.Serial(
                    self.port,
                    baudrate=115200,
                    timeout=1
            )
            print "Connected to robot."
        except:
            print "Connection failed."
        else:
            self.connection.write(self.command_dict['start'])
            self.connection.write(self.command_dict['full'])
            self.connection.write(self.command_dict['beep'])

    def handle_requestDrive(self, request):
        vel = request.velocity
        rot = request.rotation
        drive_command = self.make_drive_command(vel, rot)
        self.connection.write(drive_command)

    def make_angle_request(self):
        req = self.angle_struct.pack(142, 20)
        return req

    def handle_requestAngle(self, request):
        """
        send [142] [Packet ID]
        for angle sensor, packet ID = 20
        will return signed 16 bit value, high byte first
        counterclockwise angles are positive
        value is capped at -32768, +32767
        """
        self.connection.write(self.angle_request)
        raw_angle = self.connection.read(2)
        angle = struct.unpack('>h', raw_angle)
        return angle

if __name__ == "__main__":
    driver_node = DriveNode()
