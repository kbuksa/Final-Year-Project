# reports

Holds all documentation and reports written for this project.

# program/catkin_ws

`build` & `devel` directories hold configured files created from Catkin workspace to allow use of ROS and F1Tenth.

`src` directory holds the core files used to run ROS programs and initialise simulator.

## src 

Core folders and files used to run ROS programs and initialise simulator.

### f1tenth_simulator

Holds the simulator itself, alongside with the fundamental car capabilities, its programs and map for visualiser.

### lidar_scanner/src

`lidar_scanner.py` - program used to show use of LidarScan, outputting car's scans of left, front and right.

### proof_of_concept/src

`publisher.py` - program used to show how a simple publisher program publishes to topic random integers.

`subscriber.py` - program used to show how a simple subscriber program will listen to topic that publisher.py publishes. Outputs these messages.

### turtlesim_poc/src

`turtle_auto.py` - proof of concept program to show how ROS works in a GUI using built-in ROS robot for auto drive.

### driving program/src

`auto_drive_test.py` - program that uses F1Tenth plugin to drive robot forwards automatically.

`emergency_brake.py` - program that uses F1Tenth plugin to drive robot forwards automatically. Car stops if it detects a wall too close.

`emergency_brake_test.py` - program that tests `emergency_brake.py` using unit tests.

`localization.py` - program that uses F1Tenth plugin to check the x-coordinates & y-coordinates of the car.

`wall_follow_pid.py` - program prototype that uses F1Tenth plugin to run a wall following algorithm using mathematical approach.

`wall_follow_state.py` - program that uses F1Tenth plugin to run a wall following algorithm using logical approach via states and condition meeting.

`wall_follow_test.py` - program that tests `wall_follow_state.py` using unit tests.

`obstacle_avoid.py` - program that uses F1Tenth plugin to make the car drive through a course to avoid walls and obstacles.

`obstacle_avoid_test.py` - program that tests `obstacle_avoid.py` using unit tests.
