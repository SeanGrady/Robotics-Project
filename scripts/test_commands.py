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

interact(local=locals())
