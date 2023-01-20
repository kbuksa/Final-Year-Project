#!/usr/bin/env python
import rospy
from sensor_msgs import LaserScan
from std_msgs.msg import Float32

def callback(data):
    print("Left of robot:", data.ranges[0])
    print("In front of robot:", data.ranges[360])
    print("Right of robot:", data.ranges[719])

def lidar_scan_sub():
    #Name of node
    rospy.init_node("lidar_scanning")
    #Subscribe to scan
    rospy.Subscriber("scan", Float32, callback)
    rospy.spin()

# def publish_close():
#     publish_node = ("closest_point", Float64)
#     rospy.init_node("publish_close")

# def publish_furthest():
#     publish_node = ("furthest_point", Float64)
#     rospy.init_node("publish_far")

if __name__ == '__main__':
    lidar_scan_sub()
    # try:
    #     publish_close()
    #     publish_furthest()
        
    # except rospy.ROSInterruptException:
    #     pass