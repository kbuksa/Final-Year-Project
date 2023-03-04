# Term 1 Week 03/10/22 - 09/10/22

## 03/10

Completed overview of project, explained motivation behind doing this and how it can be solved using F1Tenth ROS simulator.

## 04/10

Created a planned timeline on what I will work on during this project. Described tasks and length of time each will take.

Completed risk assessment on the project and how it will be mitigated.

## 06/10

First meeting with supervisor, questions answered about where the simulator will be (installed in Ubuntu). Got comments about re-arranging parts of abstract. Comments on re-wording risks to make them more targeted for this project.

Project plan submitted after corrections were made from the meeting

# Week 10/10/22 - 16/10/22

Focus was shifted to catching up with other modules on the course.

Some attention put on checking how to set up ROS and F1Tenth simulator. 

# Week 17/10/22 - 23/10/22

Goals for this week:

Set up ROS on computer (via Ubuntu)

Begin researching and learning documentation of ROS.

## 17/10

Started with downloading Ubuntu desktop to work with simulator

## 18/10

Begun reading lecture notes that are accessible on F1Tenth website (https://f1tenth.org/learn.html).

## 23/10

Began writing report on the F1Tenth hardware, sensors and communications. Created reports and programs folders.

# Week 24/10/22 - 30/10/22

Goals for this week:

Complete research on hardware, sensors and communication of the F1Tenth car

## 25/10

Continued research on hardware of the F1Tenth. Researched Chassis, NVIDIA Jetson NX, Lidar sensor. Found this to be important for the report due to these parts of the car being primary components that make it work.

## 29/10

Continued hardware report write-up. Completed writing majority of F1Tenth phyical car's hardware. Plan to start writing how aspects link to simulator soon. Hopefully can finish report by mid next week to organise meeting with supervisor.

# Week 31/9/22 - 06/11/22

Week focusing on courseworks for other modules. Plans to fully initialise Ubuntu.

# Week 07/11/22 - 13/11/22

Week focusing on finishing report on hardware, potentially finish set up for F1Tenth simulator and ROS system

## 07/11

Downloaded Ubuntu. Began setup of ROS-Melodic but encountered problems. Meeting with supervisor on Wednesday will discuss this problem

## 08/11

Finished Hardware communication with ROS section of hardware report.

# Week 14/11/22 - 20/11/22

## 15/11

Discovered that ROS Melodic works most stable on Ubuntu v18.04. Downloaded this version, ROS and simulator. Can now start working on writing code.

# Week 21/11/22 - 27/11/22

## 22/11

Began writing configuration and installation of ROS environment and F1Tenth simulator report

## 23/11

Completed configuration and installation of ROS environment and F1Tenth simulator report

## 24/11

Saved simulator and files in the repository under 'program' package.

# Week 28/11/22 - 04/12/22

## 29/11

Researched how to use ROS in more detail. Created 2 simple programs for POC on ROS. 'Publisher.py' will 'publish' a random number in the ROS system at a constant rate. 'Subscriber.py' will read and display such information published from 'Publisher.py' 

## 30/11

Interim report created. Added introduction talking about why reasoning behind the project, how I am going to overcome and prove this concept.

## 01/12

Added aims and objectives of the project. Further research on algorithms (Bug algorithms, Monte Carlo Localisation, Ackermann Steering Geometry).

## 02/12

Added Installation & Config report to interim report. Added F1Tenth hardware report. Talked about message passing, PID control and how it is used in simulator and physical car.

## 04/12

Created simple turtlesim robot program to show automatic forward movement.

# Week 05/12/22 - 11/12/22

## 07/12

Talked about proof of concept of message passing (publisher-subscriber), importance in ROS. Talked about turtlesim program. Completed recordings of proof of concept demo.

# Term 2: Week 23/01/23 - 29/01/23

## 23/01

After Winter holidays and getting back into university work/life styles, I have picked back up on the project, starting off with setting up a meeting on 24/01 with supervisor.

Main parts to talk about with supervisor:
* Interim report grades
* Feedback on code & report
* Doubts about how to run code with the simulator
* Potentially creating own launcher to show ROS working

## 24/01

After meeting with supervisor, marks have been received. This term's main focus is on creating code to show F1Tenth's capabilities. 

Although receiving the green light on working to create a new launcher, I will give one final attempt on trying to run a program with the simulator.

Began writing a program which will attempt to read the LidarScanner readings. These readings are read through a subscription to LidarScan topic, by using the range variable to output left, right and front readings of the robot.

## 25/01

Completed writing the lidar scan program, added 3 print statements in the subscriber function to read range[0, 360, 719] readings.

Instead of trying to run the program by entering its details into the launcher file, mux, behaviour, etc., I have attempted to run the program by running the simulator, to create a ROS master_node connection. Once done, run the lidar scan program on a separate terminal using ```python <file name>``` command. This has successfully run the program, with it subscribing to the car's LidarScan and showing readings.

Tested the success by dropping car in different locations to see Lidar readings change.

Messaged supervisor to scrap the new launcher idea.

## 28/01

After discovering how to run programs with the simulator, the next step is to start creating an emergency brake program. This will cause the car to brake if it reaches too close to the wall.

Following the F1/10 lab sheet, the goal is to subscribe to LidarScan and Odometry topics, while publishing to AckermannDriveStamped (Ackermann). Using LidarScan readings & Odometry's coordinates, we calculate the time to collision (TTC) with wall and publishing a brake to Ackermann topic.

# Week 30/01/23 - 05/02/23

## 30/01

When attempting to calculate the TTC, the program seemed to be stuck in a loop, with the car also not moving at all. This could have been a result of topics overwriting data while trying to scan. 

As a result, I have decided to subscribe only to LidarScan and publish to Ackermann, where the car will scan what is in front of itself, and if the scan goes past set threshold, program will publish to Ackermann to brake.

I later have encountered multiple errors where when program is executed, an error occurs where reading topic's variables are not found.

# 31/01

After multiple attempts to debug the code, I decided to delete it and start from scratch. This has ended in success, where the problem was that when attempting to change topic values for speed, the variables were being declared in the wrong order.

When run, the car will drive forward until reaching near the wall, where car will brake and output a message of "stop car" to confirm that the program does work.

# Week 13/02/23 - 19/02/23

## 13/02

Begun writing code for a wall follower by using the F1Tenth lab sheets for inspiration. The plan is to implement it via use of PID control to calculate the error distance from the followed wall so car drives in parallel to wall at specific distance. 

The work will be done by subscribing to LidarScan to get laser scan of car-to-wall distance. Also will be publishing to AckermannDriveStamped to change speed and steering angle of car from scan readings.

## 15/02

Majority of wall follower code is completed. Use of constants and initialised variables to store data necessary for calculations. Main ones being calculating current and previous time between readings, calculation of cars current and prediction of future location. 

With this information, can calculate the distance from the wall and the car's steering angle, with specific conditions for each, changing the speed and steering of the car when needed.

The main issue met today is that ROS time is not working with the initialisation of the node. As a result, I will attempt to change the time from using the ROS library to the Time python library.

## 16/02

After changing the time from ROS time to Time library, there are no more errors appearing. However, when running the program, it does not seem to publish the change of speed to the car, which as a result causes the car to be forever stationary.

Tomorrow will attempt to completely redo the wall follower program, without PID changes, instead using changing of states depending on what command should be done (eg: follow wall, find wall, etc.).

Plan will be to show use of PID control in a separate program which I will think about in the later stage.

# Week 20/02/23 - 26/02/23

## 20/02

Today took a detour to create a quick and brief proof-of-concept to show how OOP can be used with ROS, by using a class to handle the program. The program itself shows a car that can drive forward automatically, with scanner message showing distance to object ahead. 

This is achieved by subscribing to LidarScan to get the range ahead of car, and publishing the speed to AckermannDriveStamped.

With this done, I will now focus on creating a new prototype for wall-follower, using states and OOP class method this week.

## 26/02

I have now began creating the new prototype to show how a wall-following algorithm would work with the F1Tenth car. This program uses states to cycle between, which are stored in a dictionary. The states are:

* 0 : Find a wall
* 1 : Turn left if wall found
* 2 : Follow wall

The reason why I decided for this approach is that it requires much less calculations, while just performing actions based on the distance of the car to walls. This also would allow the program to be easier to understand as it runs.

Both prototypes subscribe to LidarScan and publish to AckermannDriveStamped. 

LidarScan uses a dictionary variable that holds the scans for the left, front and right of the car, at 60 degree ranges and will store the lowest value in each of the ranges (distance closest to wall for each direction).

* Right [121:280]
* Front [281:440]
* Left [441:600]

I have then stated different cases for the car using series of if...then...else statements for each of the measurements. For example:

* If car's reading for left, front and right values are > threshold, change state to 0 (find wall)
* If readings for the front of car are < threshold while readings for left & right > threshold, change state to 1 (turn left)
* If readings for right of car < threshold and while readings for front & left > threshold, change state to 2 (follow wall)

These cases take into account all scenarios the car may be in, and will switch to appropriate case. 

Once a state is declared, this will then run the appropriate code for that state

* 0 will drive car forward, looking for wall
* 1 will turn the car slowly to the left
* 2 once car is perpendicular to right wall, it stops turning and drives forwards (follows wall)

Currently, the car is not able to publish the Ackermann values, but when the car is moved around the map, depending on how close/far to the wall it is, the states are shown to change through terminal output.

# Week 27/02/23 - 05/03/23

## 28/02

I have found the solution to my problem with no publishing of Ackermann values. This was resolved when I moved the code that calls on functions for state actions from the main function into the action decider function with all cases.

I have also reduced the threshold for scan range from 1 to 0.8

Currently, the car is able to drive to find a wall and turn left if wall is found. However, the car tends to always overturn therefore never going into the follow wall state. It seems that the condition is never entered into.

# 02/03

After constant trial and error with the scan ranges, I have discovered that I have been using LaserScan ranges wrong, with multiple sources using different scan ranges. For the F1Tenth, it does a full 360 degrees scan, starting at the rear and going anti-clockwise. I discovered that:

* scan.ranges[0] = 0 degrees from bottom (rear)
* scan.ranges[270] = 90 degrees (right)
* scan.ranges[540] = 180 degrees (front)
* scan.ranges[810] = 270 degrees (left)
* scan.ranges[1019] = 360 degrees (rear)

With this discovery, I have now changed the range scan dictionary for left, front and right values:

* right: scan_data.ranges[225:434]
* front: scan_data.ranges[435:644]
* left: scan_data.ranges[645:854]

This now causes each area to have a scan range of 70 degrees per region.

I have also adjusted the speed and steering angle to make the program much easier to notice changes, however I may change the speed to be faster when making demo video.


On top of this, with the discovery of LaserScan ranges, I have now returned to my previous programs of lidar_scan and emergency_brake and changed the ranges values to the appropriate ones. 

As a result, emergency brake now works correctly and can be visible to stop before touching the wall.

The next plan is to create another prototype for wall follower, which will include a new state that allows the car to follow the left wall, which will as a result keep the car driving within the track at all times without relying on a single wall.

# 04/03

Created program for the car to avoid obstacles using similar logic to the wall follower program. I have used a case system (if statements) to take apprioriate action. However instead of using states, I made the appropriate function to be called within the case. By using measurements:

* right: scan_data.ranges[225:434]
* front: scan_data.ranges[435:644]
* left: scan_data.ranges[645:854]

I was able to create cases where the car will turn left, right or drive forwards depending on what is in front of the car.

When case to drive forward, Ackermann's speed is updated only. When case to turn left, Ackermann's speed is reduced and steering angle is set to 1 to turn left. When case to turn right, Ackermann's speed is also reduced and steering angle is set to -1 to turn right. 

As a result, the car is now able to complete a track without touching any walls, turning in time to not crash into a wall, while also centering itself to be in the middle of the track for safety. When adding obstacle points into the track, the car will take appropriate evasive action to avoid the obstacle depending on what the scan read shows. Afterwards it will then re-centre on the track again.

The car
