#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

def scan_listener(scan_data):
    # print("distance to object: ", scan_data.ranges[360])
    if scan_data.ranges[360] < 0.8: #condition on how close wall is
        ack_data.drive.speed = 0 #stops car if true
        print("Stop car")
    else:
        ack_data.drive.speed = 2 #keeps speed of car at 2m/s
    pub_drive.publish(ack_data) #publishes results

if __name__ == '__main__':
    try:
        print("program begin")
        rospy.init_node("brake_car")
        #initialise Subscribers and Publishers
        sub_scanner = rospy.Subscriber("scan", LaserScan, scan_listener)
        pub_drive = rospy.Publisher("drive", AckermannDriveStamped, queue_size = 1)
        ack_data = AckermannDriveStamped()
        rate = rospy.Rate(500)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass