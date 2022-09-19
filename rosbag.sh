#!/bin/bash
#/(\/scan|\/slam_out_pose)/gm
echo "Enter number of topics to record:"
read NUM_TOPICS -r
REG="/"
while [ "$NUM_TOPICS" -ge 1 ]
do
    echo "Enter topic to record(no slashes):"
    read TOPIC -r
    REG+=$TOPIC
    REG+=" /"
    ((NUM_TOPICS--))
done
REG+="null"
echo "Enter duration to record in seconds: "
read DUR_SECS -r
echo "Enter name for bag file(no extension): "
read FILE_NAME -r
rosbag record -O "$FILE_NAME".bag --duration="$DUR_SECS" $REG






