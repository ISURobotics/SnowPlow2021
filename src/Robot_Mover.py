import time
import rospy
import sys, os
import Robot

from sensor_msgs.msg import Image, PointCloud2, LaserScan
from std_msgs.msg import Int16, Int32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Transform, Vector3, Quaternion, Point, Pose, PoseStamped
import ros_numpy
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt

last_loop = time.time()


# https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2/20401#20401


class RobotMover:
    movingMeters = 0  # Meters to move from start before stopping
    rotating = 0  # -1 for left/counterclockwise, 0 for not rotating, 1 for right/clockwise
    rotatingRadians = 0  # Radians to rotate from start before stopping
    moving = 0  # -1 for backward, 0 for not moving, 1 for forward

    def __init__(self, robot):
        """
        :param robot: A Robot object. The functions to set motor speeds will be run on it.
        """
        self.robot = robot

    def move_forward(self, pose, meters):
        """
            Starts moving forward
            pose: The pose object produced by the slam_out_pose topic
            meters: the number of meters to move before stopping
        """
        starting_pose = pose
        moving = 1
        moving_meters = meters
        self.robot.set_speeds(5)  # probably not right value

    def move_backward(self, pose, meters):
        """
            Starts moving backward
            pose: The pose object produced by the slam_out_pose topic
            meters: the number of meters to move before stopping
        """
        starting_pose = pose
        moving = -1
        moving_meters = meters
        self.robot.set_speeds(-5)  # probably not right value

    def rotate_left(self, pose, degrees):
        """
            Starts a left/counterclockwise rotation
            pose: The pose object produced by the slam_out_pose topic
            degrees: The number of degrees to turn from the current pose before stopping
        """
        starting_pose = pose
        rotating = -1
        rotating_radians = np.radians(degrees)
        self.robot.set_speed(-5, 5)

    def rotate_right(self, pose, degrees):
        """
            Starts a right/clockwise rotation
            pose: The pose object produced by the slam_out_pose topic
            degrees: The number of degrees to turn from the current pose before stopping
        """
        starting_pose = pose
        rotating = 1
        rotating_radians = np.radians(degrees)
        self.robot.set_speed(5, -5)

    def stop(self):
        """
            Stop all motion of the robot
        :return:
        """
        rotating = 0
        moving = 0
        self.robot.stop()


