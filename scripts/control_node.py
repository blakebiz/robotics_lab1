#!/usr/bin/env python3

import rospy
# import pose message for movements
from turtlesim.msg import Pose
# import turtlecontrol to receive kp and xd
from robotics_lab1.msg import Turtlecontrol
# import twist to give commands to turtle
from geometry_msgs.msg import Twist

# Create a Pose object to track the x coordinate of the turtle
pos_msg = Pose()
# Create a Turtlecontrol object to track given kp and xd
control_msg = Turtlecontrol()

# callback function for pose subscriber
def pose_callback(data):
	# make the pos_msg object globally accessible so we can change it
	global pos_msg
	# track the x coordinate of the turtle
	pos_msg.x = data.x

# callback function for the control_params subscriber
def control_callback(data):
	# make the control_msg object globally accessible so we can change it
	global control_msg
	# track the given kp
	control_msg.kp = data.kp
	# track the given xd
	control_msg.xd = data.xd



if __name__ == '__main__':
	# initialize the script as a ros node
	rospy.init_node('control_node', anonymous = True)
	# add a subscriber to listen to turtle1/pose topic
	rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	# add a subscriber to listen to turtle1/control_params topic
	rospy.Subscriber('/turtle1/control_params', Turtlecontrol, control_callback)
	# add a publisher to update the turtle with the new velocity
	cmd_pub = rospy.Publisher('/turtlesim/cmd_vel', Twist, queue_size=10)
	# declare a twist object to publish
	twist = Twist()
	# set the loop rate
	loop_rate = rospy.Rate(10)
	# while the ros node is still running
	while not rospy.is_shutdown():
		# Set the linear velocity to the result of the Proportional controller formula
		twist.linear.x = control_msg.kp * (control_msg.xd - pos_msg.x)
		# publish the updated twist object
		cmd_pub.publish(twist)
		# sleep for a little
		loop_rate.sleep()
