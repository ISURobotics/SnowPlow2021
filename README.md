Snowplow project for 2020-2021.

To run SICK-LMS-511 with ROS and jetson nano:  
1. ensure Jetson connection to router via ethernet with DHCP, disable Jetson wifi  
2. cd into catkin_ws  
3. run "roscore" in a different tab  
4. cd into catkin_ws  
5. source install/setup.bash  
6. roslaunch sick_scan sick_lms_5xx.launch hostname:=192.168.1.8 (IP may change)  

Hector SLAM testing:  
1. cd into catkin_ws  
2. source install/setup.bash  
3. roslaunch sick_scan test_005_hector.launch hostname:=192.168.1.8  
4. rviz will open. Select Global Options>Fixed Frame > laser_POS_000_DIST (May take some time to appear)  
5. rostopic echo /slam_out_pose (for best robot position and orientation info. Also may take some time)  
6. rostopic echo /tf (for a flood of data)  

General Notes:
Using map for localization
Check live data (/scan) for actual current obstacles
arduino github: https://github.com/FRC4014/SRTester
