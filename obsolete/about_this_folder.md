Author: Sean Frett
February 24, 2025

# Introduction

This is a folder for experiments we did with technologies that we ended up dropping from the robot,
such as SLAM Toolbox. I was about to delete some of these files to reduce clutter, but there is 
just enough of a chance that they could be useful again some day that I decided to keep them around
in a more accessible place than the commit history. Anything that gets put in here should have a
brief description in this file explaining why it isn't used anymore and how it might become important
again.

## slam_lidar.launch
Launch file for launching both sick_scan_xd and slam_toolbox. I think this was adapted from our ROS 1
launch file, which used Hector SLAM, but it may also have been built from the ground up. May become
relevant if we decide to use SLAM again for something.

## slam_toolbox_test.py
Python launch file for just starting SLAM toolbox. Similar situation to slam_lidar.launch.

## slam.launch.py
Another attempt at a Python file for starting SLAM toolbox. We had a lot of trouble getting this stuff
to work. Our main problem was that we could not get SLAM toolbox to understand our transform tree.

## transform_publisher.py
A simple script to create a static transform publisher from the scan frame to the map frame. Was used 
in an attempt to make SLAM toolbox work.

## UnifiedArduino/UnifiedArduino.ino
An arduino script that was meant to be used with one arduino that would control the motors and get data
from the IMU. Formerly known by the much less sensible name of IMUOUT.ino. We discontinued this because
sending data to and receiving data from the same serial port in parallel without causing problems proved
to be more difficult than expected.