#!/bin/bash
#/(\/scan|\/slam_out_pose)/gm
echo "Enter number of topics to record:"
read NUM_TOPICS
i=0
REG="/(\\/"
while [ $NUM_TOPICS -ge 1 ]
do
    echo "Enter topic to record(no slashes):"
    read TOPIC
    REG+=$TOPIC
    REG+="|\/"
    ((NUM_TOPICS--))
done
REG+="blahhh)/gm"
echo "Enter duration to record in seconds: "
read DUR_SECS
echo "Enter name for bag file(no extension): "
read FILE_NAME
rosbag record -O $FILE_NAME.bag --duration=$DUR_SECS -e \"$REG\"






