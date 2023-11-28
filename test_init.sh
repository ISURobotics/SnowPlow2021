#!/bin/bash
# initializes ROS and arduino serial. We might need more than one arduino serial line in here
source /home/karterk/catkin_ws/install/setup.bash

# run core ros service
roscore &

echo "Waiting on roscore"
sleep 2
# wait for roscore to complete
while ! rostopic list | grep "^/rosout$" >/dev/null; do
	echo -n "."	
	sleep 2
done

echo "roscore started. Starting arduino serial node..."

# boot up the rosserial node for arduino communication
rosrun rosserial_python serial_node.py __name:="arduino1" /dev/ttyACM0 &
rosrun rosserial_python serial_node.py __name:="arduino2" /dev/ttyACM1 &
echo "$!" > ~/ino_pid.txt 

sleep 5


echo "All done!"
