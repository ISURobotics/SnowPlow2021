# from multiprocessing.resource_sharer import stop
import time
from matplotlib.hatch import SouthEastHatch
import rospy
import sys, os
from Movement_Threshold import Movement_Threshold
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
import utils

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

    def move_forward(self, lidar, meters):
        """
            Starts moving forward
            lidar: a lidar object to add listeners to
            meters: the number of meters to move before stopping
        """
        print "Moving forward " + str(meters)
        pose = lidar.get_pose()
        print pose
        angle = utils.quaternion_to_euler(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)[2]
        thres = None
        if (angle >= np.pi * -(1 / 4) and angle < np.pi * (1 / 4)): # Moving in roughly positive x direction
            delta = meters * np.cos(angle)
            thres = Movement_Threshold(Movement_Threshold.X_AXIS, True, pose.position.x + delta, lambda: self.stop())
        elif (angle >= np.pi * (1 / 4) and angle < np.pi * (3 / 4)): # Roughly positive y
            delta = meters * np.sin(angle)
            thres = Movement_Threshold(Movement_Threshold.Y_AXIS, True, pose.position.y + delta, lambda: self.stop())

        elif (angle >= np.pi * (3 / 4) or angle < np.pi * -(3 / 4)): # Roughly negative x
            delta = meters * np.cos(angle) # This will be negative
            thres = Movement_Threshold(Movement_Threshold.X_AXIS, False, pose.position.x + delta, lambda: self.stop())

        elif (angle < np.pi * -(1 / 4) and angle > np.pi * -(3 / 4)): # Roughly negative y
            delta = meters * np.sin(angle) # Negative
            thres = Movement_Threshold(Movement_Threshold.Y_AXIS, False, pose.position.y + delta, lambda: self.stop())

        assert thres != None
        lidar.add_listener(thres)

        # finished here
        # moving = 1
        # moving_meters = meters
        self.robot.set_speeds(5)  # probably not right value

    def move_backward(self, lidar, meters):
        """
            Starts moving backward
            lidar: a lidar object to add listeners to
            meters: the number of meters to move before stopping
        """

        print "Moving backward " + str(meters)
        pose = lidar.get_pose()
        angle = utils.quaternion_to_euler(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)[2]
        thres = None

        if (angle >= np.pi * -(1 / 4) and angle < np.pi * (1 / 4)): # Moving in roughly positive x direction
            delta = meters * np.cos(angle)
            thres = Movement_Threshold(Movement_Threshold.X_AXIS, False, pose.position.x + delta, lambda: self.stop())
        elif (angle >= np.pi * (1 / 4) and angle < np.pi * (3 / 4)): # Roughly positive y
            delta = meters * np.sin(angle)
            thres = Movement_Threshold(Movement_Threshold.Y_AXIS, False, pose.position.y + delta, lambda: self.stop())

        elif (angle >= np.pi * (3 / 4) or angle < np.pi * -(3 / 4)): # Roughly negative x
            delta = meters * np.cos(angle) # This will be negative
            thres = Movement_Threshold(Movement_Threshold.X_AXIS, True, pose.position.x + delta, lambda: self.stop())

        elif (angle < np.pi * -(1 / 4) and angle > np.pi * -(3 / 4)): # Roughly negative y
            delta = meters * np.sin(angle) # Negative
            thres = Movement_Threshold(Movement_Threshold.Y_AXIS, True, pose.position.y + delta, lambda: self.stop())

        assert thres != None
        lidar.add_listener(thres)
        # starting_pose = pose
        # moving = -1
        # moving_meters = meters
        self.robot.set_speeds(-5)  # probably not right value

    def rotate_left(self, lidar, degrees):
        """
            Starts a left/counterclockwise rotation
            lidar: a lidar object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        print "Rotating left " + degrees
        pose = lidar.get_pose()
        angle = utils.quaternion_to_euler(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)[2]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = deltaRadians + angle
        if targetRadians > np.pi:
            targetRadians -= 2 * np.pi # Going from positive angle to negative

        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, targetRadians, lambda: self.stop())

        lidar.add_listener(thres)

        self.robot.set_speed(-5, 5)


    def rotate_right(self, lidar, degrees):
        """
            Starts a right/clockwise rotation
            lidar: a lidar object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        print "Rotating right " + degrees
        pose = lidar.get_pose()
        angle = utils.quaternion_to_euler(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)[2]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = deltaRadians - angle
        if targetRadians < np.pi:
            targetRadians += 2 * np.pi # Going from positive angle to negative

        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, targetRadians, lambda: self.stop())

        lidar.add_listener(thres)
        
        self.robot.set_speed(5, -5)

    def stop(self):
        """
            Stop all motion of the robot
        :return:
        """
        print "STOPSTOPSTOPSTOP"
        # rotating = 0
        # moving = 0
        self.robot.stop()


