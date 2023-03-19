# Project-4

## User Interface

### camera_ws
#### This folder is used during patrol mode to take screenshots of the a human if they are detected in the library

### watchdog
#### This folder is used during patrol mode to see if there are any pictures taken of the detected human and a log is created in patrollog.log file which can be viewed by the security guards

### webpage
#### This folder includes all the databases and webpages used to create the webpage for user interaction with the robot during assist and patrol mode

## Navigation Commander

### turtlebot3_commander
#### This package is used to navigate the robot to the different locations based on the information received through the different topics from the crowd detection cameras, webpage and the cameras on the robot.

## Human Detection & Crowd Detecton
### To run the detection scripts, first create a venv, enter the env, then run the get_pi_requirements.sh file to download all necessary libraries.
#### These 2 folders contain scripts that will publish and subscribe to diffferent ros2 nodes if humans are detected.
