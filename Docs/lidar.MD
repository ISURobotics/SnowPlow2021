# Autonomous Snowplow - Lidar
## About Lidar

The lidar used for this project is:
- [Product Specs](https://www.sick.com/ag/en/products/lidar-and-radar-sensors/lidar-sensors/lms5xx/lms511-10100-pro/p/p215941)

There are provided code examples:
- [Sick Code Examples](https://support.sick.com/sick-knowledgebase/article/?code=KA-09838)

Ros1 and Ros2 Driver:
- [ROS Driver](https://github.com/SICKAG/sick_scan_xd)

PicoScan:
- [PicoScan](https://support.sick.com/sick-knowledgebase/article/?code=KA-09379)

Our implementation:
- [Lidar.py](../src/Lidar.py)


The last communication from the previous team is this: 

So, right at the end of the spring 2025 semester, the Jetson's filesystem got corrupted, and we had to reflash the operating system. I reinstalled all the existing packages and copied the entirely of the old /home directory over to the new filesystem (which includes the colcon workspace and stuff like the contents of the .bash_history and .bashrc files), but some stuff still isn't working. Specifically, the Jetson does not want to talk to the Lidar, and we're not sure what we're missing.

Also, note that even though this is something that needs to work before the competition, this isn't a whole-team job or a blocking point for other developments. Other team members can still get things done while 1-2 people are on device-troubleshooting duty.

