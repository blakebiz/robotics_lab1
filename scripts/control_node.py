#!usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from robotics_lab1.msg import Turtlecontrol
import math

ROTATION_SCALE = 180.0/math.py

pos_msg = Turtlecontrol()

def pose_callback(data):
	global pos_msg
	pos_msg.velocity = data.kp * (data.xd - data.x)

def proportional_control(kp, xd, xt):
	return kp * (xd - xt)

if __name__ == '__main__':
	rospy.init_node('control_node', anonymous = True)
	rospy.Subscriber('/turtle1/control_params'), Pose, pose_callback)
	pos_pub = rospy.Publisher('/turtlesim/velocity', Turtlecontrol, queue_size=10)
	loop_rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pos_pub.publish(pos_msg)
		loop_rate.sleep()
