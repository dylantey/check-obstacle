#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math

""" Global Variables """
LENGTH_OF_SPOT = 0.5 # The parking spots are half a meter long.
STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
SPEED = 0.2
FORWARD = Twist(linear=Vector3(SPEED,0.0,0.0), angular=Vector3(0.0,0.0,0.0))
dis2box = 0

class Parking(object):
    def __init__(self):
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Instance Variables
        self.timestamp1 = None
        self.timestamp2 = None
        self.dist2wall = None
        self.dis2neato = None
        self.widthofspot = None
        self.twist = None
        self.radius = None

        # adjustment
        self.adjustment = 0
        self.is_aligned = False

    def process_scan(self, m):
        """ Callback Function for laser subscriber """
        #current_dist = m.ranges[270]  # CHANGE?
	current_dist = m.ranges[10]
        #print "Ranges[10]: ", current_dist
	if self.timestamp2 is None:
            self.twist = FORWARD

            if self.dis2neato is None:
                self.dis2neato = current_dist
                print "dis2neato: ", self.dis2neato
            elif current_dist > (self.dis2neato + LENGTH_OF_SPOT - 0.05):
                self.dis2wall = current_dist
                self.radius = (self.dis2wall / 2.0)
                if self.timestamp1 is None:
                    self.timestamp1 = rospy.Time.now()
                    print "TIME1: ", self.timestamp1

            if abs(current_dist - self.dis2neato) <= 0.2 and self.timestamp1 is not None:
                self.timestamp2 = rospy.Time.now()
                print "TIME2: ", self.timestamp2
                self.twist = STOP

        elif self.timestamp1 is not None and self.timestamp2 is not None:
            self.adjustment = 0.2
            self.widthofspot = SPEED * (self.timestamp2.secs - self.timestamp1.secs)
            if self.dis2neato >= 0.3 and not self.is_aligned:
                self.align_with_origin()
                self.park()
                rospy.signal_shutdown("Done parking.")
            elif self.dis2neato < 0.3:
                print "Neato was too close to park"

    def stop(self):
        self.publisher.publish(STOP)

    def align_with_origin(self):
        dist = self.radius - self.widthofspot/2.0 + self.adjustment
        now = rospy.Time.now()
        travel_time = dist/SPEED  # distance / speed = time
        while rospy.Time.now() - now <= rospy.Duration(travel_time):
            self.twist = FORWARD
        self.twist = STOP
        self.is_aligned = True

    def drive_arc(self, omega, travel_time, sign):
        now = rospy.Time.now()
        while rospy.Time.now() - now <= travel_time:
            self.twist = Twist(linear = Vector3(sign*SPEED,0,0), angular=Vector3(0,0,omega))

    def park(self):
        # first arc
        omega = SPEED / (self.radius + 0.25)
        travel_time = rospy.Duration(math.pi/2.0/omega)
        self.drive_arc(omega, travel_time, -1)

        # second drive_arc
        omega = SPEED / self.radius
        travel_time = rospy.Duration(math.pi/3.0/omega - 0.2)
        self.drive_arc(-omega, travel_time, -1)

        # third drive_arc
        omega = -0.4
        travel_time = rospy.Duration(1)
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

