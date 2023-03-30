#!/usr/bin/env python
import obstacle_avoid
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

class Obstacle_avoid_test(unittest.TestCase): #class for unit tests


    def setUp(self): #set up test environment from Obstacle avoid class
        self.obs_avoid = obstacle_avoid.Obstacle_avoidance()

    def tearDown(self):
        pass


    #test if case 'no wall'
    def test_close_no_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 1080 #adds value 1.2 to all range elements (ranges has 1080 elements in)
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertGreater(min(self.obs_avoid.scan_dict), self.obs_avoid.threshold, "Smallest scan_data range not smaller than threshold!")
        self.assertEqual(self.obs_avoid.drive_mode, 1, "not in 'no wall' - forward mode")
        # self.assertEqual(self.obs_avoid.switch_case, 1, "not in 'no wall' case")


    #test if case 'left wall'
    def test_close_left_wall(self): 
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 680 + [0.4] + [1.2] * 400 #add an element < threshold to be in "left" range and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 3, "not in 'left wall' - turn right mode")
        # self.assertEqual(self.obs_avoid.switch_case, 2, "not in 'wall left' case")
    

    #test if case 'left & right walls'
    def test_close_left_right_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.4] + [1.2] * 400 + [0.4] + [1.2] * 400 #add an element < threshold to be in "right" and "left" ranges and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to Wall_follow's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 1, "not in 'left & right wall' - forward mode")
        # self.assertEqual(self.obs_avoid.switch_case, 3, "not in 'wall left & right' case")


    #test if case 'wall front', with left side having less space
    def test_close_front_wall_left_turn(self): 
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 580 + [0.4] + [1.3] * 500  #add an element < threshold to be in "front" range and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 2, "not in 'front wall' - turn left mode. Right side has less turn space")
        # self.assertEqual(self.obs_avoid.switch_case, 4, "not in 'wall front' case")
    

    #test if case 'wall front', with right side having less space
    def test_close_front_wall_right_turn(self): 
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.3] * 580 + [0.4] + [1.2] * 500  #add an element < threshold to be in "front" range and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 3, "not in 'front wall' - turn right mode. Left side has less turn space")
        # self.assertEqual(self.obs_avoid.switch_case, 4, "not in 'wall front' case")


    #test if case 'front & right walls'
    def test_close_front_right_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.4] + [1.2] * 300 + [0.4] + [1.2] * 500 #add an element < threshold to be in "right" and "front" ranges and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 2, "not in 'front & right wall' - turn left mode")
        # self.assertEqual(self.obs_avoid.switch_case, 5, "not in 'wall front & right' case")


    #test if case 'front & left walls'
    def test_close_front_left_wall(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 580 + [0.4] + [1.2] * 100 + [0.4] + [1.2] * 400 #add an element < threshold to be in "right" and "left" ranges and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 3, "not in 'front & left wall' - turn right mode")
        # self.assertEqual(self.obs_avoid.switch_case, 6, "not in 'wall front & left' case")
    

    #test if case 'front, left & right walls', with right side having less space
    def test_close_all_walls_left_turn(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.3] + [1.2] * 200 + [0.4] + [1.2] * 200 + [0.4] + [1.2] * 400 #add an element < threshold to be in "right", "left" and "front" ranges and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 2, "not in 'front & left wall' - turn left mode. Right side has less turn space")
        # self.assertEqual(self.obs_avoid.switch_case, 7, "not in 'wall front & left & right' case")


        #test if case 'front, left & right walls', with right side having less space
    def test_close_all_walls_right_turn(self):
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.4] + [1.2] * 200 + [0.4] + [1.2] * 200 + [0.3] + [1.2] * 400 #add an element < threshold to be in "right", "left" and "front" ranges and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 3, "not in 'front & left wall' - turn right mode. Left side has less turn space")
        # self.assertEqual(self.obs_avoid.switch_case, 7, "not in 'wall front & left & right' case")
    

    #test if case 'wall right'
    def test_close_right_wall(self): 
        scan_data = LaserScan()
        #simulates scan_data.ranges for all regions
        scan_data.ranges = [1.2] * 280 + [0.4] + [1.2] * 800 #add an element < threshold to be in "right" range and fill rest with values > threshold
        self.obs_avoid.scan_listen(scan_data) #sends simulated values to obs_avoid's scan_listener method

        #assertion tests
        self.assertEqual(self.obs_avoid.drive_mode, 2, "not in 'right wall' - turn left mode")
        # self.assertEqual(self.obs_avoid.switch_case, 8, "not in 'wall right' case")

if __name__ == '__main__':
    rospy.init_node("obstacle_avoid_test")
    rostest.rosrun("driving_programs", "obstacle_avoid_test", Obstacle_avoid_test)