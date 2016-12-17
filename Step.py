#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, Vector3, Twist
#from kalman_zumy.srv import ImuSrv,ImuSrvResponse,NuSrv,NuSrvResponse
#import exp_quat_func as eqf
import time

class Step():
	def __init__(self, next_ar="ar_maker_0", direction_of_ur5=0, tilt_time=100.0):
		self.next_ar = next_ar
		self.direction_of_ur5 = direction_of_ur5
		self.tilt_time = tilt_time


	def __str__(self):
		return "next ar_tag name is: "+self.next_ar
		+"\nur5 should move to: "+str(self.direction_of_ur5)
		+"\ntilt time: "+str(self.tilt_time)
