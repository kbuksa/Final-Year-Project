# #!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import sys

class Wall_follow:

    #Dictionary that stores the 3 scan data directions
    scan_dict = {
        "right": 0,
        "front": 0,
        "left": 0,}

    #stores current state program is in
    
    
    def __init__(self):
        sub = rospy.Subscriber('/scan', LaserScan, self.scan_listen)
        self.pub_drive = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.rate = rospy.Rate(20)
        #Dictionary that stores the 3 states of the program
        self.switch_case = -1 #initialise check with what case was taken up in if-then-elif statements
        self.current_state = 0 #start state at find wall
        self.threshold = 0.8 #Max acceptable distance to wall
        self.state_dict = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',}
            

    #Changes program' state
    def state_change(self, state):
        if state is not self.current_state:
            print("New state: ", self.state_dict[state]) 
            self.current_state = state
            

    #Stores scan data in scan dictionary
    def scan_listen(self, scan_data):

        global scan_dict

        #LidarScan works in 360 view starting from rear of car and going anti-clockwise
        scan_dict = {
            "right": min(scan_data.ranges[225:434]),
            "front": min(scan_data.ranges[435:644]),
            "left": min(scan_data.ranges[645:854]),}
        # print(scan_dict)
        self.follow_action()

    def follow_action(self):
        global scan_dict

        #Cases
        #Cases 0 - find wall
        if scan_dict['front'] > self.threshold and scan_dict['left'] > self.threshold and scan_dict['right'] > self.threshold:
            #close to no wall
            self.switch_case = 1
            self.state_change(0)
        elif scan_dict['front'] > self.threshold and scan_dict['left'] < self.threshold and scan_dict['right'] > self.threshold:
            #close to left wall
            self.switch_case = 2
            self.state_change(0)
        elif scan_dict['front'] > self.threshold and scan_dict['left'] < self.threshold and scan_dict['right'] < self.threshold:
            #close to left and right walls
            self.switch_case = 3
            self.state_change(0)
        #Cases 1 - turn left 
        elif scan_dict['front'] < self.threshold and scan_dict['left'] > self.threshold and scan_dict['right'] > self.threshold:
            #close to wall in front
            self.switch_case = 4
            self.state_change(1)
        elif scan_dict['front'] < self.threshold and scan_dict['left'] > self.threshold and scan_dict['right'] < self.threshold:
            #close to front and right walls
            self.switch_case = 5
            self.state_change(1)
        elif scan_dict['front'] < self.threshold and scan_dict['left'] < self.threshold and scan_dict['right'] > self.threshold:
            #close to front and left walls
            self.switch_case = 6
            self.state_change(1)
        elif scan_dict['front'] < self.threshold and scan_dict['left'] < self.threshold and scan_dict['right'] < self.threshold:
            #close to front, left and right walls
            self.switch_case = 7
            self.state_change(1)
        #Cases 2 - follow wall
        elif scan_dict['front'] > self.threshold and scan_dict['left'] > self.threshold and scan_dict['right'] < self.threshold:
            #close to right wall
            self.switch_case = 8
            self.state_change(2)
        else:
            print("No known state")
            sys.exit()
        
        if self.current_state == 0:
            self.find_wall()
        elif self.current_state == 1:
            self.turn_left()
        elif self.current_state == 2:
            self.follow_wall()
        else:
            rospy.logerr('Unknown state!')

    def find_wall(self):
        # print("find wall")
        ack_drive = AckermannDriveStamped()
        # ack_drive.drive.steering_angle = -0.05
        ack_drive.drive.speed = 0.8
        self.pub_drive.publish(ack_drive)


    def turn_left(self):
        # print("turn left")
        ack_drive = AckermannDriveStamped()
        ack_drive.drive.speed = 0.5
        ack_drive.drive.steering_angle = 0.8
        self.pub_drive.publish(ack_drive)

    def follow_wall(self):
        # print("following")
        ack_drive = AckermannDriveStamped()
        ack_drive.drive.speed = 0.8
        ack_drive.drive.steering_angle = -0.04
        self.pub_drive.publish(ack_drive)

if __name__ == '__main__':
    try:
        print("program begin")
        rospy.init_node("wall_follow_state")
        wall_follow = Wall_follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass