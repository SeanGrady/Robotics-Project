# Roomba Soccer

This is a ROS package to play soccer with an iRobot Create 2. It contains 3 nodes: the computer vision node, the driver node (to interface with the create base and provide ROS services for things like driving the robot or asking how far the robot has turned), and the controller node.

The driver node interfaces with the Create 2 via a serial connection, using PySerial. It uses the commands provided by the Create 2 Open Interface to drive the Create around and read its sensors. Since there is a bug in the Create 2 firmware that renders all sensors which report data in milimeters wildly inaccurate, the node performs its own distance measurements using the wheel encoders and the physical dimensions of the robot.

The computer vision node recognizes and provides distances to the 'soccer ball' and the goal using a combination of color thresholding and contour finding in OpenCV 2.4

The controller node contains the high-level logic and state transitions for the robot, the algorithm for playing soccer.
