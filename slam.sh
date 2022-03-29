 #!/bin/sh
 #runs the basic laserscan
 source install/setup.bash
 roslaunch sick_scan test_005_hector.launch hostname:=192.168.1.2
