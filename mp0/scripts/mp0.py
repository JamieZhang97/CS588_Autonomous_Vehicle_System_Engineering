#!/usr/bin/env python3

#==============================================================================
# File name          : mp0.py                                                                 
# Description        : MP0 for CS588                                                                                                                        
# Usage              : rosrun mp0 mp0.py                                                                                                                           
#==============================================================================
from __future__ import print_function

#Python Headers
import math
import os

# ROS Headers
import rospy

# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt

class Node():

	def __init__(self):

		self.rate = rospy.Rate(0.1)
		
		self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
		self.turn_cmd = PacmodCmd()
		self.turn_cmd.ui16_cmd = 1 # None

	def run(self):

		while not rospy.is_shutdown():
			#S
			self.turn_cmd.ui16_cmd = 2 # Turn left
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 2 # Turn left
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 2 # Turn left
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)

			#O
			self.turn_cmd.ui16_cmd = 0 # Turn right
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 0 # Turn right
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 0 # Turn right
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)

			#S
			self.turn_cmd.ui16_cmd = 2 # Turn left
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 2 # Turn left
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 2 # Turn left
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)
			self.turn_cmd.ui16_cmd = 1 # None
			self.turn_pub.publish(self.turn_cmd)
			rospy.sleep(0.5)

			self.turn_pub.publish(self.turn_cmd)


			self.rate.sleep

if __name__ == '__main__':
	rospy.init_node('sos_node', anonymous=True)
	node = Node()
	rospy.sleep(1)
	node.run()