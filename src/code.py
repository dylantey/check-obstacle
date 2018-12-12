#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time

""" Global Variables """
STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
SPEED = 0.2
FORWARD = Twist(linear=Vector3(SPEED,0.0,0.0), angular=Vector3(0.0,0.0,0.0))

class Parking(object):
    def __init__(self):
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Instance Variables
        self.timestamp1 = None
        self.timestamp2 = None
        self.dis2box = None
        self.widthofspot = None
        self.twist = None
        self.radius = None

        # adjustment
        self.adjustment = 0
        self.is_aligned = False

    def process_scan(self, m):
        """ Callback Function for laser subscriber """
	current_dist = m.ranges[10]
			
	if self.timestamp2 is None:
            self.twist = FORWARD
			
		# Set the distance to box
            if self.dis2box is None:
                self.dis2box = current_dist
                print "dis2box: ", self.dis2box
            
	    # Set the distance to wall when seeing empty space for the first time
	    if current_dist > (self.dis2box + 0.5):
                if self.timestamp1 is None:
			self.dis2wall = current_dist
                	self.radius = (self.dis2wall * math.sin(0.5) / 2.0)
                        self.timestamp1 = rospy.Time.now()
                        print "TIME1: ", self.timestamp1
			
	    # Almost past the parking space
            if abs(current_dist - self.dis2box) <= 0.02 and self.timestamp1 is not None:
                self.timestamp2 = rospy.Time.now()
                print "TIME2: ", self.timestamp2
                self.twist = STOP

        elif self.timestamp1 is not None and self.timestamp2 is not None:
            self.adjustment = 0.1
            self.widthofspot = SPEED * (self.timestamp2.secs - self.timestamp1.secs)

            if self.widthofspot >= 0.75 and not self.is_aligned:	
		self.warmup()
                self.park()
                rospy.signal_shutdown("Done parking.")
            else:
                print "box is too close to park, refinding parking space"
		self.timestamp1 = None
		self.timestamp2 = None

    def stop(self):
        self.publisher.publish(STOP)

    def drive_arc(self, omega, travel_time, sign):
        now = rospy.Time.now()
        while rospy.Time.now() - now <= travel_time:
            self.twist = Twist(linear = Vector3(sign*SPEED,0,0), angular=Vector3(0,0,omega))

    def warmup(self):
	d = self.dis2box * math.cos(0.5) + self.adjustment
	t = d / SPEED
	startTime = time.time()
	while (time.time() - startTime) <= t:
		self.twist = FORWARD
	self.twist = STOP
        self.is_aligned = True

    def park(self):
        # first arc
        omega = SPEED / self.radius
        travel_time = rospy.Duration(math.pi/2.0/omega)
        self.drive_arc(omega, travel_time, -1)

        # second drive_arc
        omega = SPEED / self.radius
        travel_time = rospy.Duration(math.pi/2.0/omega)
        self.drive_arc(-omega, travel_time, -1)

	# straight drive_arc
        omega = 0
        travel_time = rospy.Duration(2)
        self.drive_arc(omega, travel_time, 1)

        # stop
        self.twist = STOP

    def run(self):
        """ This is the main loop function """
        rospy.on_shutdown(self.stop)

        while not rospy.is_shutdown():
            if self.twist:
                self.publisher.publish(self.twist)
            self.r.sleep()

if __name__ == '__main__':
    rospy.init_node('parking')
    parking_node = Parking()
    parking_node.run()

