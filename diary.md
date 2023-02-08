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
