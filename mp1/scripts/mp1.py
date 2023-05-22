#!/usr/bin/env python3

#==============================================================================
# File name          : mp1.py                                                                 
# Description        : MP1 for CS588                                                                                                                        
# Usage              : rosrun mp1 mp1.py                                                                                                                           
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
from zed_interfaces.msg import ObjectsStamped

class Node():

	def __init__(self):

		self.rate = rospy.Rate(0.1)
		self.od_flag = 0
		self.x_cam = -1
		self.y_cam = -1
		self.z_cam = -1

		# ZED2 object detector
		sub = rospy.Subscriber('/zed2/zed_node/obj_det/objects', ObjectsStamped, self.od_callback)

	    # GEM vehilce shift control
		self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
		self.gear_cmd = PacmodCmd()
		self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL
	
	    # GEM vehilce brake control
		self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
		self.brake_cmd = PacmodCmd()
		self.brake_cmd.enable = True
		self.brake_cmd.clear  = True
		self.brake_cmd.ignore = True

        # GEM vechile forward motion control
		self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
		self.accel_cmd = PacmodCmd()
		self.accel_cmd.enable = True
		self.accel_cmd.clear  = True
		self.accel_cmd.ignore = True
	
	    # GEM vechile steering wheel control
		self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
		self.steer_cmd = PositionWithSpeed()
		self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
		self.steer_cmd.angular_velocity_limit = 2.0 # radians/second

	def od_callback(self, objects_msg):
		# Process the objects in the message
		object_list = objects_msg.objects
		if object_list == []:
			self.od_flag = 0
		else:
			self.od_flag = 1
			for object in object_list:
				label = object.label # Object label
				position = object.position # Object centroid position
				#print(f"detected {label} at {position}")
				self.x_cam = position[0]
				self.y_cam = position[1]
				self.z_cam = position[2]



	def run(self):



		while not rospy.is_shutdown():

			self.gear_cmd.ui16_cmd = 3 # FORWARD_NEUTRAL
			self.gear_pub.publish(self.gear_cmd)

			self.steer_cmd.angular_position = 0.0
			
			if self.od_flag == 1 and self.y_cam < 3.5 and self.y_cam > -3.5 and self.x_cam < 20:
				# slow down
				print("detected human at: ", self.x_cam)
				self.accel_cmd.f64_cmd = 0.0
				self.brake_cmd.f64_cmd = 0.8

			else:
				# moving forward
				print("clear")
				self.accel_cmd.f64_cmd = 0.4
				self.brake_cmd.f64_cmd = 0.0			
		
			
			self.accel_pub.publish(self.accel_cmd)
			self.brake_pub.publish(self.brake_cmd)
			self.steer_pub.publish(self.steer_cmd)

			self.rate.sleep

if __name__ == '__main__':
	rospy.init_node('emergency_stop', anonymous=True)
	node = Node()
	node.run()