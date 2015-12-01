import struct
import math
import serial
from code import interact

port = '/dev/ttyUSB0'
baudrate=115200
timeout=1
connection = serial.Serial(port, baudrate=baudrate, timeout=1)
encoder_max = 65535

def make_drive_command(vel, rot):
    vl = sorted([-500, vel + rot, 500])[1]
    vr = sorted([-500, vel - rot, 500])[1]
    cmd = struct.pack('>Bhh', 145, vr, vl)
    return cmd

def drive(vel, rot):
    cmd = make_drive_command(vel, rot)
    print "writing ", cmd
    connection.write(cmd)

def rcom(string):
    cmd = ''
    for num in string.split():
        cmd += chr(int(num))
    return cmd

start = rcom('128')
safe = rcom('131')
shutdown_cmd = rcom('173')
connection.write(shutdown_cmd)
connection.write(start)
connection.write(safe)

def read_ang():
    ang_req = struct.pack('>BB', 142, 20)
    print ang_req
    connection.write(ang_req)
    read_vals = connection.read(2)
    angle = struct.unpack('>h', read_vals)
    return angle, read_vals

r_req = rcom('142 44')
l_req = rcom('142 43')

left_total, right_total = 0, 0

def encoder_count_reset():
    global left_total, right_total
    connection.write(l_req)
    raw_left_counts = connection.read(2)
    left_counts = struct.unpack('>H', raw_left_counts)
    connection.write(r_req)
    raw_right_counts = connection.read(2)
    right_counts = struct.unpack('>H', raw_right_counts)
    left_counts = left_counts[0]
    right_counts = right_counts[0]
    right_total = right_counts
    left_total = left_counts
    return left_total, right_total

def calc_angle():
    global left_total, right_total
    connection.write(l_req)
    raw_left_counts = connection.read(2)
    left_counts = struct.unpack('>H', raw_left_counts)
    left_counts = left_counts[0]
    connection.write(r_req)
    raw_right_counts = connection.read(2)
    right_counts = struct.unpack('>H', raw_right_counts)
    right_counts = right_counts[0]
    
    if right_counts > right_total:
        right_diff = right_total + (encoder_max - right_counts)
    else:
        right_diff = right_counts - right_total
    if left_counts < left_total:
        left_diff = left_counts + (encoder_max - left_total)
    else:
        left_diff = left_counts - left_total

    left_dist = left_diff* (1/508.8) * (math.pi*72)
    right_dist = right_diff* (1/508.8) * (math.pi*72)

    angle_rad = (right_dist - left_dist) / 235.0
    angle_deg = angle_rad*(180/math.pi)

    right_total = right_counts
    left_total = left_counts
    return abs(angle_deg)

encoder_count_reset()
print left_total, right_total

angle = calc_angle()
print angle

encoder_count_reset()
print left_total, right_total


#connection.write(shutdown_cmd)
#connection.close()
interact(local=locals())
