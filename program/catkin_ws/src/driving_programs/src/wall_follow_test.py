#!/usr/bin/env python
import wall_follow_state
import rospy
import unittest
import rostest
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

'''
This is the dictionary tested, 
values in ranges are used to simulate numbers in the scan_data range variables

scan_dict = {
            "right": min(scan_data.ranges[225:434]),
            "front": min(scan_data.ranges[435:644]),
            "left": min(scan_data.ranges[645:854]),}'''

class Wall_follow_test(unittest.TestCase): #class for unit tests

    def setUp(self): #set up test environment from Emergency_brake class
        self.wall_follow = wall_follow_state.Wall_follow()

    def tearDown(self):
        pass

    #test to find wall & test if case 'no wall'
    def test_scan_state_find_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 1080 #adds value 1.2 to all range elements (ranges has 1080 elements in)
        self.wall_follow.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertGreater(min(self.wall_follow.scan_dict), self.wall_follow.threshold, "Smallest scan_data range not smaller than threshold!")
        self.assertEqual(self.wall_follow.current_state, 0, "Current state is not in find wall")
        self.assertEqual(self.wall_follow.switch_case, 1, "not in 'no wall' case")
        
    
    #test to turn left & test if case 'wall front'
    def test_scan_state_turn_left(self): 
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 580 + [0.4] + [1.2] * 500  #add an element < threshold to be in "front" range and fill rest with values > threshold
        self.wall_follow.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertEqual(self.wall_follow.current_state, 1, "Current state is not in turn left")
        self.assertEqual(self.wall_follow.switch_case, 4, "not in 'wall front' case")

    #test to follow wall & test if case 'wall right'
    def test_scan_state_follow_wall(self): 
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.4] + [1.2] * 800 #add an element < threshold to be in "right" range and fill rest with values > threshold
        self.wall_follow.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertEqual(self.wall_follow.current_state, 2, "Current state is not in follow wall")
        self.assertEqual(self.wall_follow.switch_case, 8, "not in 'wall right' case")
    
    #test if case 'left wall'
    def test_close_left_wall(self): 
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 680 + [0.4] + [1.2] * 400 #add an element < threshold to be in "left" range and fill rest with values > threshold
        self.wall_follow.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertEqual(self.wall_follow.current_state, 0, "Current state is not in find wall")
        self.assertEqual(self.wall_follow.switch_case, 2, "not in 'wall left' case")
    
    #test if case 'left & right walls'
    def test_close_left_right_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.4] + [1.2] * 400 + [0.4] + [1.2] * 400 #add an element < threshold to be in "right" and "left" ranges and fill rest with values > threshold
        self.wall_follow.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertEqual(self.wall_follow.current_state, 0, "Current state is not in find wall")
        self.assertEqual(self.wall_follow.switch_case, 3, "not in 'wall left & right' case")

    #test if case 'front & right walls'
    def test_close_front_right_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.4] + [1.2] * 300 + [0.4] + [1.2] * 500 #add an element < threshold to be in "right" and "front" ranges and fill rest with values > threshold
        self.wall_follow.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertEqual(self.wall_follow.current_state, 1, "Current state is not in find wall")
        self.assertEqual(self.wall_follow.switch_case, 5, "not in 'wall front & right' case")

    #test if case 'front & left walls'
    def test_close_front_left_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 580 + [0.4] + [1.2] * 100 + [0.4] + [1.2] * 400 #add an element < threshold to be in "right" and "left" ranges and fill rest with values > threshold
        self.wall_follow.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertEqual(self.wall_follow.current_state, 1, "Current state is not in find wall")
        self.assertEqual(self.wall_follow.switch_case, 6, "not in 'wall front & left' case")
    
    #test if case 'front, left & right walls'
    def test_close_front_left_right_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.4] + [1.2] * 200 + [0.4] + [1.2] * 200 + [0.4] + [1.2] * 400 #add an element < threshold to be in "right", "left" and "front" ranges and fill rest with values > threshold
        self.wall_follow.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertEqual(self.wall_follow.current_state, 1, "Current state is not in find wall")
        self.assertEqual(self.wall_follow.switch_case, 7, "not in 'wall front & left & right' case")


if __name__ == '__main__':
    rospy.init_node("wall_follow_test")
    rostest.rosrun("driving_programs", "wall_follow_test", Wall_follow_test)