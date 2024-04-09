#!/bin/bash
# initializes ROS, lidar launch, and arduino serial
# source /home/snowclone/catkin_ws/install/setup.bash
source /opt/ros/humble/setup.bash
# run core ros service
# roscore &

# echo "Waiting on roscore"
# sleep 2
# wait for roscore to complete
# while ! rostopic list | grep "^/rosout$" >/dev/null; do
# 	echo -n "."	
# 	sleep 2
# done

echo "Starting Lidar"

# boot up the lidar listener
#ros2 run sick_scan_xd sick_generic_caller /home/snowclone/colcon_ws/src/sick_scan_xd/launch/sick_lms_5xx.launch hostname:=192.168.1.2 &

ros2 run sick_scan_xd sick_generic_caller slam_lidar.launch &
ros2 run slam_toolbox async_slam_toolbox_node slam_lidar.launch &
sleep 3
while ! ros2 topic list | grep "^/pose$" >/dev/null; do
        echo -n "."     
        sleep 3
done

echo "***Hector SLAM Initialized. Setting up arduino serial node..."

# boot up the rosserial node for arduino communication
ros2 run rosserial_python serial_node.py __name:="arduino1" /dev/ttyACM0 &
ros2 run rosserial_python serial_node.py __name:="arduino2" /dev/ttyACM1 &
echo "$!" > ~/ino_pid.txt 

sleep 5


echo "All done!"
