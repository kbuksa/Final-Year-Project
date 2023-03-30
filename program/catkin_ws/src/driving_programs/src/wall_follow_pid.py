#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
import time

#constants
PI = 3.141592653589793
DESIRED_DIST = 0.8

#PID control constants
K_PROP = 1
K_DERIV = 0.1
K_INTEG = 0.5


class Wall_follower:

    prev_error = 0.0
    prev_t = time.time() #previous time - init as current time
    error_t = 0.0 #function e(t) used in equation
    time_delay = 0.0 #time delay between calculation of distance
    integration = 0.0 #used with K_INTEG

    def __init__(self):
        #initialising subs and pubs
        
        self.sub_scanner = rospy.Subscriber("scan", LaserScan, self.scan_listener)
        self.pub_drive = rospy.Publisher("drive", AckermannDriveStamped, queue_size = 1)
        self.rate = rospy.Rate(500)
        self.ack_data = AckermannDriveStamped()

    def scan_listener(self, scan_data):
        angle_b = 90.0 / 180.0 * PI #getting angle 90 degrees (rad) of car 
        angle_a = 45.0 / 180 * PI #getting angle 45 degrees (rad) of car

        #calculating which element from LidarScan ranges[] to use to scan distance to wall for side a & b
        index_a = int(math.floor((angle_a - scan_data.angle_min) / scan_data.angle_increment))
        index_b = int(math.floor((angle_b - scan_data.angle_min) / scan_data.angle_increment))
        print(index_a, index_b)
        range_a = scan_data.ranges[index_a]
        range_b = scan_data.ranges[index_b]

        #using distances a & b to calculate angle alpha between car's x axis and the right wall
        angle_alpha = math.atan((range_a * math.cos(angle_a) - range_b) / range_a * math.sin(angle_a))

        #using alpha angle at current time, now find the current distance (Dt) to wall from car
        dist_t = range_b * math.cos(angle_alpha)

        #using alpha angle at current time, now find the future distance (Dt+1) to wall from car after travelling 1m
        dist_t1 = dist_t + (1.0 * math.cos(angle_alpha))
        # self.pid_control(dist_t1)
        self.test()

    def test(self):
        print("publish")
        self.ack_data.drive.speed = 1.5
        self.pub_drive.publish(self.ack_data)

    def pid_control(self, dist_t1):
        self.error_t = DESIRED_DIST - dist_t1 #calculating error from desired distance
        moment_now = time.time() #getting new current time reading 
        self.time_delay = moment_now - self.prev_t
        self.integration = self.integration + (self.prev_error)
        (K_PROP * self.error_t + K_DERIV * (self.error_t - self.prev_error) / self.time_delay + K_INTEG * self.integration)
        self.prev_t = moment_now #sets current time as previous time for future calc
        self.prev_error = self.error_t #sets current error as previous error for future calc
        print("check")
        if abs(self.ack_data.drive.steering_angle) > 20.0 / 180.0 * PI:
            print("speed: 0.5")
            self.ack_data.drive.speed = 0.5
        elif abs(self.ack_data.drive.steering_angle) > 10.0 / 180.0 * PI:
            print("speed: 1.0")
            self.ack_data.drive.speed = 1.0
        else:
            print("speed: 1.5")
            self.ack_data.drive.speed = 1.5
        self.pub_drive.publish(self.ack_data)

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    print("program begin")
    Wall_follower()
    rospy.spin()