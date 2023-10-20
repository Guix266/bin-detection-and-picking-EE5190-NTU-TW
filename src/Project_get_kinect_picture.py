#!/usr/bin/env python

from __future__ import division
#from function import *
from std_msgs.msg import String
import rospy
import threading
import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#from Project_running import *

global num
num=0
def kinect_callback(data):
	global num
	num+=1
	if num%30==0:
		rospy.loginfo(rospy.get_caller_id() + "I See.")
		cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
		cv2.imwrite("./tmp_img/capture.jpg".format(n= int(num/30)),cv_image)
		#cv2.imwrite("./kinect_img/image{n}.jpg".format(n= int(num/40)),cv_image)
		#rospy.sleep(3)
	#pkg = Project_running.Main(cv_image)
	#if pkg==False:
	#	print("Found Nothing")
	#else:
	#	print("Position at ({pos_x},{pos_y}), and angle = {angle}".format(pos_x = pkg[0], pos_y = pkg[1],angle=pkg[2]) )
	
def kinect_listener():
	rospy.init_node('Project_ai', anonymous=True)
	kinect_rgb = rospy.Subscriber("/camera/rgb/image_rect_color", Image, kinect_callback)
	#rospy.sleep(3)

if __name__ == '__main__':
	print("Starting program...")
	bridge = CvBridge()
	kinect_listener()
	rospy.spin()
