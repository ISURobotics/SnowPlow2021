# Autonomous Snowplow - Pathing
## About Lidar

Pathing utilizes almost all files in the src folder. For convenience, they are located below.

Maybe we can review these as a group:
[Arduino_Node.py](../src/Arduino_Node.py)
[Axes.py](../src/Axes.py)
[Func_Generator.py](../src/Func_Generator.py)
[Motor.py](../src/Motor.py)
[Movement_Threshold.py](../src/Movement_Threshold.py)
[Path_Executor.py](../src/Path_Executor.py)
[Path_Finder.py](../src/Path_Finder.py)
[Robot.py](../src/Robot.py)
[RobotGPS.py](../src/RobotGPS.py)
[Robot_Mover.py](../src/Robot_Mover.py)
[Serial_Detect.py](../src/Serial_Detect.py)
[main.py](../src/main.py)
[utils.py](../src/utils.py)

Which is dependent on data recieved from:
[Sensors.py](../src/Sensors.py)
[GPS_Node.py](../src/GPS_Node.py)
[IMU.py](../src/IMU.py)
[Lidar.py](../src/Lidar.py)

And ultimately drives:
[Motor.py](../src/Motor.py)

Last communication from previous team was:

This was assigned to Tony, but he quit and his current code doesn't really work with our sensor listener system

Basically, we need to add a new function to RobotMover.py that makes the robot move toward a specific point on the field and correct its path by trying to point towards that point while moving forward

Currently, our correction algorithm just tries to keep the robot pointed in the direction it was in when it started moving forward, which isn't always the exact direction we need.

This is partially implemented at the moment, but it still needs to be finished and tested. Whoever ends up doing this, get familiar with the publisher-subscriber pattern, as our "movement threshold" system is an implementation of it

(The current implementation is on the Point-following branch)
