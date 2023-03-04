# #!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import sys

#Dictionary that stores the 3 scan data directions
scan_dict = {
    "right": 0,
    "front": 0,
    "left": 0,}

def scan_listen(scan_data):
    global scan_dict

    #LidarScan works in 360 view starting from rear of car and going anti-clockwise
    scan_dict = {
        "right": min(scan_data.ranges[225:434]),
        "front": min(scan_data.ranges[435:644]),
        "left": min(scan_data.ranges[645:854]),}
    take_action()

def take_action():
    global scan_dict
    threshold = 0.75 #Max  distance acceptable to obstacle

    if scan_dict['front'] > threshold and scan_dict['left'] > threshold and scan_dict['right'] > threshold:
        #close to no wall
        forward()

    elif scan_dict['front'] > threshold and scan_dict['left'] < threshold and scan_dict['right'] > threshold:
        #close to left wall
        turn_right()

    elif scan_dict['front'] > threshold and scan_dict['left'] < threshold and scan_dict['right'] < threshold:
        #close to left and right walls
        forward()

    elif scan_dict['front'] < threshold and scan_dict['left'] > threshold and scan_dict['right'] > threshold:
        #close to wall in front

        if scan_dict['left'] <= scan_dict['right']: #checks whether to turn left or right
            turn_right() #priority to turn right if bigger gap on right
        else:
            turn_left() #priority to turn right if bigger gap on right

    elif scan_dict['front'] < threshold and scan_dict['left'] > threshold and scan_dict['right'] < threshold:
        #close to front and right walls
        turn_left()

    elif scan_dict['front'] < threshold and scan_dict['left'] < threshold and scan_dict['right'] > threshold:
        #close to front and left walls
        turn_right()

    elif scan_dict['front'] < threshold and scan_dict['left'] < threshold and scan_dict['right'] < threshold:
        #close to front, left and right walls
        turn_left()

    elif scan_dict['front'] > threshold and scan_dict['left'] > threshold and scan_dict['right'] < threshold:
        #close to right wall

        if scan_dict['left'] <= scan_dict['right']: #checks whether to turn left or right
            turn_right() #priority to turn right if bigger gap on right
        else:
            turn_left() #priority to turn right if bigger gap on right

    else:
        print("No known state")
        sys.exit()

def forward():
    ack_drive = AckermannDriveStamped()
    ack_drive.drive.speed = 1
    pub_drive.publish(ack_drive)

def turn_left():
    ack_drive = AckermannDriveStamped()
    ack_drive.drive.speed = 0.4
    ack_drive.drive.steering_angle = 1
    pub_drive.publish(ack_drive)

def turn_right():
    ack_drive = AckermannDriveStamped()
    ack_drive.drive.speed = 0.4
    ack_drive.drive.steering_angle = -1
    pub_drive.publish(ack_drive)

if __name__ == '__main__':
    try:
        print("program begin")
        rospy.init_node("obstacle_avoid")
        sub = rospy.Subscriber('/scan', LaserScan, scan_listen)
        pub_drive = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        rate = rospy.Rate(20)
        ack_drive = AckermannDriveStamped()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass