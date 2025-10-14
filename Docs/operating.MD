## The Jetson Nano has a number of bash scripts which can be executed from anywhere via alias:  
```initplow``` : initializes ROS core, ROS lidar functionality/Hector SLAM, and rosserial for arduino communication.  
```startplow``` : executes ```main.py``` in this directory's ```src``` folder.  
```runplow``` : executes the above 2 functions in sequence, initializes then starts the plow's movement.  
```stopplow``` : ```pkill``` all the processes spawned by the above scripts.

## To run SICK-LMS-511 with ROS and jetson nano:  
1. ensure Jetson connection to router via ethernet with DHCP, disable Jetson wifi  
2. cd into catkin_ws  
3. run ```roscore``` in a different tab  
4. cd into catkin_ws  
5. ```source install/setup.bash```  
6. ```roslaunch sick_scan sick_lms_5xx.launch hostname:=192.168.1.2``` (IP may change - set IP with SOPAS-ET for Windows)  
7. ```rviz rviz``` to open the rviz visualizer for the data  

## Hector SLAM testing:  
1. cd into catkin_ws  
2. ```source install/setup.bash```  
3. ```roslaunch sick_scan test_005_hector.launch hostname:=192.168.1.2```  
4. rviz will open. Select Global Options>Fixed Frame > laser_POS_000_DIST (May take some time to appear)  
5. ```rostopic echo /slam_out_pose``` (for best robot position and orientation info. Also may take some time)
7. ```rostopic echo /scan``` (for a nice array of distances to objects for each half degree)  

## General Notes:  
Using map for localization  
Check live data (/scan) for actual current obstacles  

ros_numpy geometry: https://github.com/eric-wieser/ros_numpy/blob/master/src/ros_numpy/geometry.py  

To change which git user is used on the nano:  
```git config user.email "you@example.com"```  
```git config user.name "Your Name"```  
Remember to update ssh keys as well  

Lidar IPs:  
SICK LMS511: 192.168.1.2  
SICK TiM551: 192.168.1.3



Last communication from previous team is: We have the shell script that launches all the processes we need normally, but a ROS launch file is the "correct" way to do this. Once we have this , we should be able to run "ros2 launch SnowPlow2021" and everything will start.

