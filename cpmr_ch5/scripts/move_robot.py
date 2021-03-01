#!/usr/bin/env python3

import rospy
import time
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray, String
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from cpmr_ch4.msg import float64

def callback(data):
#	print(data.data)
	publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	msg = Twist()
#	robot radius is 0.4m, so if the distance is less than 0.4 stop moving
	if (data.data[2] == 0.0 or data.data[2] < 0.4):
		print('stop robot')
		msg.linear.x = 0.0
		msg.angular.z = 0.0
	else: 
		angle = ((math.atan(data.data[0] / data.data[2]) * math.pi) / 180.0)
#		print('moving robot')
		print ("Distance: ", data.data[2])
		print ("Angle: ", angle)
		msg.linear.x = 0.3
		if (angle > 0):
			msg.angular.z = -0.1
		else:
			msg.angular.z = 0.1
			
	publisher.publish(msg)
    
if __name__ == '__main__':
	rospy.init_node('publisher')
	subscriber = rospy.Subscriber('/tVects', float64, callback)
	rospy.spin()
