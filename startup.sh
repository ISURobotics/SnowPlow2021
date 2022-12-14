#!/bin/bash
#runs the basic laserscan with slami
# wait for "started core service [/rosout]
source install/setup.bash
output=$( rosrun )
until [ $output="Node listening" ]
do
    echo $output
done

roslaunch slam_lidar.launch &


