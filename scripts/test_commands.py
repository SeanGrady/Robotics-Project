import struct
import serial
from code import interact

port = '/dev/ttyUSB0'
baudrate=115200
timeout=1
connection = serial.Serial(port, baudrate=baudrate, timeout=1)

def rcom(string):
    cmd = ''
    for num in string.split():
        cmd += chr(int(num))
    return cmd

start = rcom('128')
safe = rcom('131')
connection.write(start)
connection.write(safe)

def read_ang():
    ang_req = struct.pack('>BB', 142, 20)
    print ang_req
    connection.write(ang_req)
    read_vals = connection.read(2)
    angle = struct.unpack('>h', read_vals)
    return angle, read_vals

def shutdown():
    shutdown_cmd = rcom('173')
    connection.write(shutdown_cmd)

angle, read_vals = read_ang()
print angle

interact(local=locals())
