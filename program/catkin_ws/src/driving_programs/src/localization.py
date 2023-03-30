# #!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import sys

class Localization:
    
    def __init__(self):
        rospy.init_node("localization")
        #initialise subscribers
        sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_listen)
        sub_pose = rospy.Subscriber('/odom', Odometry, self.pose_listen)
        self.scan_ranges = None
        self.poses = None
    
    #get scan ranges and store in array
    def scan_listen(self, scan_data):
        self.scan_ranges = np.array(scan_data.ranges)

    #get car pose
    def pose_listen(self, msg):
        self.poses = msg.pose.pose
        self.locate()
    

    def locate(self):
        # takes closest obstacle range of scan
        closest_idx = np.argmin(self.scan_ranges)
        
        # calculates angle of closest obstacle
        angle = closest_idx * math.radians(0.25) # takes the radians of the degree difference between each scan

        # calculates x and y positioning of robot
        x = self.poses.position.x + self.scan_ranges[closest_idx] * math.cos(angle)
        y = self.poses.position.y + self.scan_ranges[closest_idx] * math.sin(angle)
        
        print ('x: ', x, 'y: ', y)


if __name__ == '__main__':
    print("program begin")
    localize = Localization()
    rospy.spin()