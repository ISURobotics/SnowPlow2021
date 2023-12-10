#!/bin/bash
#runs the basic laserscan with slam
source install/setup.bash
roslaunch sick_scan test_005_hector.launch hostname:=192.168.1.2
