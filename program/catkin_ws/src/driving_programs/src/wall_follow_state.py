# #!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import math
import time

#keeping class empty if I implement program as class
class Wall_follow:
    pass

#Dictionary that stores the 3 scan data directions
scan_dict = {
    "right": 0,
    "front": 0,
    "left": 0,}

#Dictionary that stores the 3 states of the program
current_state = 0
state_dict = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',}

#Changes program' state
def state_change(state):
    global state_dict, current_state
    if state is not current_state:
        print("New state: ", state_dict[state]) 
        current_state = state
        

#Stores scan data in scan dictionary
def scan_listen(scan_data):
    # print("range length", len(scan_data.ranges))
    # print("range 000 (rear?)", scan_data.ranges[0])
    # print("range 090", scan_data.ranges[90])
    # print("range 180", scan_data.ranges[180])
    # print("range 270", scan_data.ranges[270])
    # print("range 360", scan_data.ranges[360])
    # print("range 450", scan_data.ranges[450])
    # print("range 540 (front?)", scan_data.ranges[540])
    # print("range 630", scan_data.ranges[630])
    # print("range 720", scan_data.ranges[720])
    global scan_dict

    #LidarScan works in 360 view starting from rear of car and going anti-clockwise
    scan_dict = {
        "right": min(scan_data.ranges[225:434]),
        "front": min(scan_data.ranges[435:644]),
        "left": min(scan_data.ranges[645:854]),}
    # print(scan_dict)
    follow_action()

def follow_action():
    global scan_dict
    threshold = 0.8 #Max acceptable distance to wall

    #Cases
    #Cases 0 - find wall
    if scan_dict['front'] > threshold and scan_dict['left'] > threshold and scan_dict['right'] > threshold:
        #close to no wall
        state_change(0)
    elif scan_dict['front'] > threshold and scan_dict['left'] < threshold and scan_dict['right'] > threshold:
        #close to left wall
        state_change(0)
    elif scan_dict['front'] > threshold and scan_dict['left'] < threshold and scan_dict['right'] < threshold:
        #close to left and right walls
        state_change(0)
    #Cases 1 - turn left 
    elif scan_dict['front'] < threshold and scan_dict['left'] > threshold and scan_dict['right'] > threshold:
        #close to wall in front
        state_change(1)
    elif scan_dict['front'] < threshold and scan_dict['left'] > threshold and scan_dict['right'] < threshold:
        #close to front and right walls
        state_change(1)
    elif scan_dict['front'] < threshold and scan_dict['left'] < threshold and scan_dict['right'] > threshold:
        #close to front and left walls
        state_change(1)
    elif scan_dict['front'] < threshold and scan_dict['left'] < threshold and scan_dict['right'] < threshold:
        #close to front, left and right walls
        state_change(1)
    #Cases 2 - follow wall
    elif scan_dict['front'] > threshold and scan_dict['left'] > threshold and scan_dict['right'] < threshold:
        #close to right wall
        state_change(2)
    else:
        print("No known state")
        sys.exit()
    
    if current_state == 0:
        find_wall()
    elif current_state == 1:
        turn_left()
    elif current_state == 2:
        follow_wall()
        pass
    else:
        rospy.logerr('Unknown state!')

def find_wall():
    # print("find wall")
    ack_drive = AckermannDriveStamped()
    ack_drive.drive.speed = 0.5
    pub_drive.publish(ack_drive)


def turn_left():
    # print("turn left")
    ack_drive = AckermannDriveStamped()
    ack_drive.drive.speed = 0.2
    ack_drive.drive.steering_angle = 0.6
    pub_drive.publish(ack_drive)

def follow_wall():
    # print("following")
    ack_drive = AckermannDriveStamped()
    ack_drive.drive.steering_angle = 0
    ack_drive.drive.speed = 0.3
    pub_drive.publish(ack_drive)

if __name__ == '__main__':
    try:
        print("program begin")
        rospy.init_node("wall_follow_state")
        sub = rospy.Subscriber('/scan', LaserScan, scan_listen)
        pub_drive = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        rate = rospy.Rate(20)
        ack_drive = AckermannDriveStamped()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass