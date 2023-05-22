#!/usr/bin/env python3

#==============================================================================
# File name          : mp2.py                                                                 
# Description        : MP2 for CS588                                                                                                                        
# Usage              : rosrun mp2 mp2.py                                                                                                                           
#==============================================================================
from __future__ import print_function

#Python Headers
import math
import os

# ROS Headers
import rospy

# GEM PACMod Headers
from std_msgs.msg import Header, Float64
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt
from zed_interfaces.msg import ObjectsStamped



class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0

    def reset(self):
        self.iterm  = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de


class Node():

	def __init__(self):

		self.rate = rospy.Rate(0.1)
		self.od_flag = 0
                
		self.x_cam = -1
		self.y_cam = -1
		self.z_cam = -1
		self.x_radar = -1
		self.y_radar = -1
		self.z_radar = -1

		self.speed = 0

		# PID distance controller
		self.pid_distance = PID(0.1, 0.0, 0.3, wg=10)
		self.desired_dis = 5

		# ZED2 object detector
		sub = rospy.Subscriber('/zed2/zed_node/obj_det/objects', ObjectsStamped, self.od_callback)
		
		#GEM vehicle speed report
		self.speed_pub = rospy.Subscriber('as_tx/vehicle_speed', Float64, self.speed_callback)

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
				
				# Object in camera frame
				self.x_cam = position[0]
				self.y_cam = position[1]
				self.z_cam = position[2]

				# Object in radar frame            
				self.x_radar = self.x_cam - 1.5
				self.y_radar = self.y_cam
				self.z_radar = self.z_cam + 1.2

	def speed_callback(self, speed_msg):
		self.speed = speed_msg


	def run(self):

		while not rospy.is_shutdown():

			current_time = rospy.get_time()

			if self.od_flag == 1:
				print("person at : ", self.x_radar, "vehicle speed: ", self.speed)
				# person detected
				output_accel = self.pid_distance.get_control(current_time, self.desired_dis - self.x_radar)
				
				if self.speed <= 0.05 and self.x_radar < self.desired_dis:
					output_accel = 0.35 
				else:
					if output_accel > 0.4:
						output_accel = 0.4
				if output_accel < 0:
					output_accel = 0

			else:
				# no person found
				print("no person found", "vehicle speed: ", self.speed)
				output_accel = 0
			
			self.gear_cmd.ui16_cmd = 1 # SHIFT_BACKWARD
			self.steer_cmd.angular_position = 0.0
			self.brake_cmd.f64_cmd = 0.0			
			self.accel_cmd.f64_cmd = output_accel

			self.gear_pub.publish(self.gear_cmd)
			self.accel_pub.publish(self.accel_cmd)
			self.brake_pub.publish(self.brake_cmd)
			self.steer_pub.publish(self.steer_cmd)

			self.rate.sleep

if __name__ == '__main__':
	rospy.init_node('emergency_stop', anonymous=True)
	node = Node()
	node.run()