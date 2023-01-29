#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math

#Program is inspired from lecture and lab sheet from f1tenth documentation regarding AEB
class Safety:
    def init(self):
        self.car_speed = 0 #initialise car speed var
        self.collision_limit = 0.8 #threshold time for collision
        #subscribe to scan & odometry
        self.sub_scanner = rospy.Subscriber("scan", LaserScan, self.laser_callback)
        self.sub_drive = rospy.Subscriber("odom", Odometry, self.odom_callback)
        #publishers to decide if robot needs to brake
        self.publish_brake = rospy.Publisher("brake", AckermannDriveStamped)
        #init car velocity
        self.ackermann_brake = AckermannDriveStamped
        self.rate = rospy.Rate(60)
        

    def laser_callback(self, laser_data):
        min_ttc = max() #init minimum TTC
        angle = laser_data.angle_min #sets angle value to min
        ranges_read = laser_data.ranges #sets range array to var
        print("calculating TTC")
        for angle_scan in range(len(ranges_read)): #for each angle of lidar scan (0 - 180 degrees)
            #formula for TTC found from lab sheet and lecture 
            ttc = angle_scan / max(self.car_speed * math.cos(angle)) #calculate TTC using distance_to_wall / time_derivative_of_object
            if (ttc < min_ttc):
                min_ttc = ttc
            angle = angle + (angle_scan * laser_data.angle_increment) #increments angle used for calulations
        
        if (min_ttc < self.collision_limit): #check if TTC lower than set time #publish brake as True
            self.ackermann_brake.header = laser_data.header
            self.ackermann_brake.drive.speed = 0.0 #set speed to 0.0 which brakes car
            print("Emergency Brake: On")
            self.publish_brake.publish(self.ackermann_brake) #publish brake
        else:
            print("Emergency Brake: Off")



    def odom_callback(self, odom_data):
        #calculating relative speed of the car
        self.car_speed = odom_data.twist.twist.linear.x #odometry uses twist.twist to get the linear x-coord of car

def main():
    #name of node
    rospy.init_node("stop_car")
    Safety()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass