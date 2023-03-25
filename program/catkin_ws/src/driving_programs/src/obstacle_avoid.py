# #!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import sys

class Obstacle_avoidance:

    #Dictionary that stores the 3 scan data directions
    scan_dict = {
        "right": 0,
        "front": 0,
        "left": 0,}
    
    def __init__(self):
        sub = rospy.Subscriber('/scan', LaserScan, self.scan_listen)
        self.pub_drive = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.threshold = 0.75 #Max  distance acceptable to obstacle
        self.drive_mode = 0
        self.rate = rospy.Rate(20)

    def scan_listen(self, scan_data):
        global scan_dict

        #LidarScan works in 360 view starting from rear of car and going anti-clockwise
        scan_dict = {
            "right": min(scan_data.ranges[225:434]),
            "front": min(scan_data.ranges[435:644]),
            "left": min(scan_data.ranges[645:854]),}
        self.take_action()

    def take_action(self):
        global scan_dict

        if scan_dict['front'] > self.threshold and scan_dict['left'] > self.threshold and scan_dict['right'] > self.threshold:
            #close to no wall
            self.forward()

        elif scan_dict['front'] > self.threshold and scan_dict['left'] < self.threshold and scan_dict['right'] > self.threshold:
            #close to left wall
            self.turn_right()

        elif scan_dict['front'] > self.threshold and scan_dict['left'] < self.threshold and scan_dict['right'] < self.threshold:
            #close to left and right walls
            self.forward()

        elif scan_dict['front'] < self.threshold and scan_dict['left'] > self.threshold and scan_dict['right'] > self.threshold:
            #close to wall in front

            if scan_dict['left'] <= scan_dict['right']: #checks whether to turn left or right
                self.turn_right() #priority to turn right if bigger gap on right
            else:
                self.turn_left() #priority to turn right if bigger gap on right

        elif scan_dict['front'] < self.threshold and scan_dict['left'] > self.threshold and scan_dict['right'] < self.threshold:
            #close to front and right walls
            self.turn_left()

        elif scan_dict['front'] < self.threshold and scan_dict['left'] < self.threshold and scan_dict['right'] > self.threshold:
            #close to front and left walls
            self.turn_right()

        elif scan_dict['front'] < self.threshold and scan_dict['left'] < self.threshold and scan_dict['right'] < self.threshold:
            #close to front, left and right walls

            if scan_dict['left'] <= scan_dict['right']: #checks whether to turn left or right
                self.turn_right() #priority to turn right if bigger gap on right
            else:
                self.turn_left() #priority to turn right if bigger gap on right

        elif scan_dict['front'] > self.threshold and scan_dict['left'] > self.threshold and scan_dict['right'] < self.threshold:
            #close to right wall
            self.turn_left()

        else:
            print("No known state")
            sys.exit()

    def forward(self):
        ack_drive = AckermannDriveStamped()
        self.drive_mode = 1
        ack_drive.drive.speed = 1
        self.pub_drive.publish(ack_drive)

    def turn_left(self):
        ack_drive = AckermannDriveStamped()
        self.drive_mode = 2
        ack_drive.drive.speed = 0.4
        ack_drive.drive.steering_angle = 1
        self.pub_drive.publish(ack_drive)

    def turn_right(self):
        ack_drive = AckermannDriveStamped()
        self.drive_mode = 3
        ack_drive.drive.speed = 0.4
        ack_drive.drive.steering_angle = -1
        self.pub_drive.publish(ack_drive)

if __name__ == '__main__':
    try:
        print("program begin")
        rospy.init_node("obstacle_avoid")
        obs_avoid = Obstacle_avoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass