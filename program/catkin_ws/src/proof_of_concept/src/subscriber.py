#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

#Program inspired by ros.org tutorial

#outputs subscribed message
def terminal_output(data):
    rospy.loginfo(rospy.get_caller_id() + "Message received, Number is: %s", data.data)

#Creates new subscriber node and loops subscriber output
def subscriber():
    #Name of node
    rospy.init_node('listener')
    #declares node subscribing to 'num_output' topic.
    #Will be able to see msgs published from publisher
    rospy.Subscriber("num_output", Int16, terminal_output)
    #Keeps python from exiting until shutdown
    rospy.spin()

if __name__ == '__main__':
    subscriber()