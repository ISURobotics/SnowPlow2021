#!/bin/bash
# shut down robot programs
pkill -9 hector_mapping
pkill -9 sick_generic_caller
pkill -9 roslaunch
pkill -9 roscore
pkill -9 rosout
pkill -9 rosmaster

#get the arduino pid
kill -9 $(cat ~/ino_pid.txt)

echo "Shutdown completed"
