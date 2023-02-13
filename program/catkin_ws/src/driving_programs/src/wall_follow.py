#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class Wall_follower:

    def __init__(self):
        self.sub_scanner = rospy.Subscriber("scan", LaserScan, scan_listener)
        self.pub_drive = rospy.Publisher("drive", AckermannDriveStamped, queue_size = 1)
        self.ack_data = AckermannDriveStamped()
        self.rate = rospy.Rate(500)
    
    def scan_listener(self, scan_data):


if __name__ == '__main__':
    print("program begin")
    rospy.init_node("wall_follower")
    Wall_follower()
    rospy.spin()