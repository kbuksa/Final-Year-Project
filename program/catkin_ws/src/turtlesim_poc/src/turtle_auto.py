#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

#Code inspired by http://wiki.ros.org/turtlesim/Tutorials/Moving%20in%20a%20Straight%20Line

def auto_move():
    #name of node
    rospy.init_node('turtle_move')
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #moving only on x axis at speed 1
    vel_msg.linear.x = 1

    #since moving only horizontally, y and z are 0 and no angles at x so also 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():
        #constantly publish the travel distance of going forward in x axis
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        auto_move()
    except rospy.ROSInterruptException: pass