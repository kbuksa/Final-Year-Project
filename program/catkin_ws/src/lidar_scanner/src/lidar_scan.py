#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    print("Left of robot:", data.ranges[810]) #range value left of car
    print("In front of robot:", data.ranges[540]) #range value ahead of car
    print("Right of robot:", data.ranges[270]) #range value right of car

def lidar_scan_sub():
    #Name of node
    rospy.init_node("lidar_scanning")
    #Subscribe to scan
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    lidar_scan_sub()