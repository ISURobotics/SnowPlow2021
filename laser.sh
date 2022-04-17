#!/bin/bash
#runs the basic laserscan
source install/setup.bash
roslaunch sick_scan sick_lms_5xx.launch hostname:=192.168.1.2
