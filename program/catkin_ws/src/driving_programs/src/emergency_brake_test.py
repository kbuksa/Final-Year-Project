#!/usr/bin/env python
import emergency_brake
import rospy
import unittest
import rostest
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Emergency_brake_test(unittest.TestCase): #class for unit tests

    def setUp(self): #set up test environment from Emergency_brake class
        self.brake = emergency_brake.Emergency_brake()

    def tearDown(self):
        pass
    
    def test_scan_less_than_threshold(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges to have these values
        scan_data.ranges = [0.4, 0.8, 1.2, 1.7, 2.3] #0.4 < threshold so program should brake
        self.brake.scan_listener(scan_data) #sends simulated values to Emergency_brake's scan_listener method

        #assertion test
        self.assertLess(min(scan_data.ranges), self.brake.threshold, "Smallest scan_data range not smaller than threshold!")

    def test_scan_greater_than_threshold(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges to have these values
        scan_data.ranges = [0.6, 0.8, 1.2, 1.7, 2.3] #all values > threshold so program should keep car driving
        self.brake.scan_listener(scan_data) #sends simulated values to Emergency_brake's scan_listener method

        #assertion test
        self.assertGreater(min(scan_data.ranges), self.brake.threshold, "Smallest scan_data range not larger than threshold!")

    def test_brake(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges to have these values
        scan_data.ranges = [0.4, 0.8, 1.2, 1.7, 2.3] #0.4 < threshold so program should brake
        self.brake.scan_listener(scan_data) #sends simulated values to Emergency_brake's scan_listener method

        #assertion test
        self.assertLess(min(scan_data.ranges), self.brake.threshold, "Smallest scan_data range not smaller than threshold!")
        self.assertEqual(self.brake.ack_data.drive.speed, 0.0, "Speed != 0.0, car has not stopped!")

    def test_no_brake(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges to have these values
        scan_data.ranges = [0.6, 0.8, 1.2, 1.7, 2.3] #all values > threshold so program should keep car driving
        self.brake.scan_listener(scan_data) #sends simulated values to Emergency_brake's scan_listener method

        #assertion test
        self.assertEqual(self.brake.ack_data.drive.speed, 2.0, "Speed != 2.0, car has different speed/stopped!")
        self.assertGreater(min(scan_data.ranges), self.brake.threshold, "Smallest scan_data range not larger than threshold!")

if __name__ == '__main__':
    rospy.init_node("brake_test")
    rostest.rosrun("driving_programs", "emergency_brake_test", Emergency_brake_test)