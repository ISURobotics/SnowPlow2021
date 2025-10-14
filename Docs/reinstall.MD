Installation:

In the event of a need for a complete reinstallation, the following process will allow for faster status updates.
At the time of this writing, 10/2/25, Humble Hawksbill was a supported ROS2 installation on Ubuntu 22.04. IMPORTANT : This changes regularly and care 
should be taken when selecting ROS2 version to ensure compatibility with the latest version of Lidar and Jetson Ubuntu version. Installation of an 
unsupported version can result in significant time delays. 22.04 with humble was used in all examples contained within this repository.


The Jetson Nano getting started guide is located here: 

- [Getting Started with the Jetson Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

Ros2 Installation:

Upon successful installation of 22.04 for Jetson, proceed to the ROS2 humble installation:

- [ROS2](https://docs.ros.org/en/humble/Installation.html)

I feel like we should expand on this and share what we know collectively. The above is the process for installation of ROS2. Generally, after installation you can go through a tutorial series which would familiarize you with colcon build. I am wondering if we got that far after installation. This is detailed here and will likely help close one issue. 

- [Colcon Build](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)


Vision Systems:

Sick Lidar w/ imu
- [Getting Started with the Sick 511 Video Series](https://support.sick.com/sick-knowledgebase/article/?code=KA-09385)
- [Lidar](/Docs/lidar.MD)

Path Planning:
Path planning is facilitated by the previous generations git repo: 

- [SnowPlow2021](https://github.com/ISURobotics/SnowPlow2021)


Data Collection:
Data collection is facilitated by Rosbag.
Moving forward it likely makes sense to use MCAP and transition any existing Rosbags. If any data needs to be converted from ROS1:

- [Rosbag conversion ROS to ROS2](https://docs.ros.org/en/noetic/api/ov_core/html/dev-ros1-to-ros2.html)


Upon successful installation, continue to :

- [Operating Procedure](/Docs/operating.MD)
