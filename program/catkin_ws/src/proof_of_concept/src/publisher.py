#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16
import random

#Simple function to publish number to be read by subscriber
def num_publish():
    #Declaring node publising to 'num_output' topic with message type of 16-bit int
    #queue_size limits number of queued msgs if publishing/subscriber is slow
    publish_node = rospy.Publisher('num_output', Int16, queue_size=10)
    #Name of node
    rospy.init_node('publisher_int16')
    rate = rospy.Rate(10) # Loops msgs at 10hz p/s
    #Runs program until ctrl+C to shutdown command
    while not rospy.is_shutdown():
        #Creates a message outputting random number
        msg_int = random.randint(0,99)
        print("Number is: ", msg_int)
        publish_node.publish(msg_int)
        rate.sleep()

if __name__ == '__main__':
    try:
        num_publish()
    except rospy.ROSInterruptException:
        pass