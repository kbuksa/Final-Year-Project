#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Auto_drive_test:

    def __init__(self):
        sub_scan = rospy.Subscriber("scan", LaserScan, self.scan_listener)
        #initialise Subscribers and Publishers
        self.pub_drive = rospy.Publisher("drive", AckermannDriveStamped, queue_size = 1)
        rate = rospy.Rate(500)

    def scan_listener(self, scan_data):
        print("distance to wall: ", scan_data.ranges[540])
        ack_data = AckermannDriveStamped()
        ack_data.drive.speed = 2 #keeps speed of car at 2m/s
        self.pub_drive.publish(ack_data)

if __name__ == '__main__':
    try:
        print("program begin")
        rospy.init_node("brake_car")
        Auto_drive_test()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass