#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Emergency_brake:

    def __init__(self):
        #initialise Subscribers and Publishers
        sub_scanner = rospy.Subscriber("scan", LaserScan, self.scan_listener)
        self.pub_drive = rospy.Publisher("drive", AckermannDriveStamped, queue_size = 1)
        self.ack_data = AckermannDriveStamped()
        self.rate = rospy.Rate(500)

    def scan_listener(self, scan_data):
        front_scan = scan_data.ranges[540] 
        if front_scan < 0.8: #condition on how close wall is
            self.ack_data.drive.speed = 0 #stops car if true
            print("Stop car")
        else:
            self.ack_data.drive.speed = 2 #keeps speed of car at 2m/s
        self.pub_drive.publish(self.ack_data) #publishes results

if __name__ == '__main__':
    try:
        print("program begin")
        rospy.init_node("brake_car")
        Emergency_brake()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass