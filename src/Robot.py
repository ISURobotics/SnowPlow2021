# from turtle import right
# from matplotlib.pyplot import axis
import time

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from Movement_Threshold import Movement_Threshold
import ros_numpy
import matplotlib.pyplot as plt
import utils
import numpy as np
import Motor
# import Lidar

# Note: Before running this file, be sure to start roscore and rosrun the rosserial_python
# >> rosrun rosserial_python serial_node.py
from typing import List


class Robot:
    def __init__(self, rospy_init=True):
        if rospy_init:
            rospy.init_node('Robot', anonymous=False)
        self.left = Motor('left_motor')
        self.right = Motor('right_motor')
        # self.lidar = Lidar()

    def stop(self):
        self.left.set_speed(0)
        self.right.set_speed(0)

    def set_speed(self, speed):
        self.left.set_speed(speed)
        self.right.set_speed(speed)
        print "speed set"

    def set_speeds(self, leftSpeed, rightSpeed):
        self.left.set_speed(leftSpeed)
        self.right.set_speed(rightSpeed)
        print "speeds set"

    def get_speeds(self):
        """
            Returns (left speed, right speed)
        """
        return (self.left.speed, self.right.speed)

    def wait_for_pub(self):
	print "Waiting for publishers.."
	topics = rospy.get_published_topics()
	print topics
	while not (['/left_motor/speed', 'std_msgs/Int8'] in topics) or not(['/right_motor/speed', 'std_msgs/Int8'] in topics):
	    topics = rospy.get_published_topics()
	    time.sleep(1)
	time.sleep(20)
        print "Publishers active."
	return
