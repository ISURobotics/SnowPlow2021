#!/bin/bash
#runs the basic laserscan with slami
# wait for "started core service [/rosout]
source install/setup.bash
# output=$( roscore )
# until [ $output="Node listening" ]
# do
#    echo $output
# done

# run core ros service
roscore &

echo "Waiting on roscore"
# wait for roscore to complete
while ! rostopic list | grep "^/rosout$" >/dev/null; do
	echo -n "."	
	sleep 10
done

echo "roscore started. Starting Lidar"

# boot up the lidar listener
roslaunch slam_lidar.launch hostname:=192.168.1.2 &

while ! rostopic list | grep "^/slam_out_pose$" >/dev/null; do
        echo -n "."     
        sleep 10
done

echo "all done!"
