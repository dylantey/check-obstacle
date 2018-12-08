#!/usr/bin/env python

import time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
	#print "[5]"
	#print msg.ranges[5]
	#print time.time()
	

	#if msg.ranges[0] > 1:
	#	print "Yes"
	#else:
		#print "No"
	global total
	global past
	if total > 5:
		print 'yes'
	else:	
		print total

	if msg.ranges[360] > 0.8:
		total += (time.time() - past)
	else:
		total = 0
	past = time.time()
	pub.publish(move)

rospy.init_node('check_obstacle')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)
move = Twist()
past = time.time()
total = 0

rospy.spin()

