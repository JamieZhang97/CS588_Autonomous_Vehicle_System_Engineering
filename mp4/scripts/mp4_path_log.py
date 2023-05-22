#!/usr/bin/env python3

#==============================================================================
# File name          : mp4.py                                                                 
# Description        : MP4 for CS588                                                                                                                        
# Usage              : rosrun mp4 mp4_path_log.py                                                                                                                           
#==============================================================================
from __future__ import print_function

#Python Headers
import math
import os
import csv
import pandas as pd

# ROS Headers
import rospy
import alvinxy.alvinxy as axy # Import AlvinXY transformation module

# GEM PACMod Headers
from std_msgs.msg import Header 
#from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt
#from zed_interfaces.msg import ObjectsStamped

# GNSS-INS Headers
from novatel_gps_msgs.msg import Inspva

class Node():

	def __init__(self):

		self.rate = rospy.Rate(0.1)
		self.olat = 40.0928563
		self.olon = -88.2359994

		self.df = pd.DataFrame({'X':[], 'Y':[], "heading" :[]})

		# ins message
		self.gnss_sub = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

	def inspva_callback(self, inspva_msg):
		self.lat     = inspva_msg.latitude  # latitude
		self.lon     = inspva_msg.longitude # longitude
		self.heading = inspva_msg.azimuth   # heading in degrees
		self.x, self.y = axy.ll2xy(self.lat, self.lon, self.olat, self.olon)
		#print(self.x)
		df2 = {'X': self.x, 'Y':  self.y, "heading" :self.heading}
		#self.df.loc[len(self.df.index)] =[self.x, self.y, self.heading]
		self.df = self.df.append(df2, ignore_index = True)
		print(self.df)
	def run(self):

		while not rospy.is_shutdown():

			self.rate.sleep

		self.df.to_csv('mp4_demo.csv')

if __name__ == '__main__':
	rospy.init_node('gnss_log', anonymous=True)
	node = Node()
	node.run()
