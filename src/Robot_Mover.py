# from multiprocessing.resource_sharer import stop
import time
# from matplotlib.hatch import SouthEastHatch
# import rospy
# import sys, os
from Movement_Threshold import Movement_Threshold
# import Robot

# from sensor_msgs.msg import Image, PointCloud2, LaserScan
# from std_msgs.msg import Int16, Int32
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import PointCloud2, LaserScan
# from sensor_msgs import point_cloud2
# from geometry_msgs.msg import Transform, Vector3, Quaternion, Point, Pose, PoseStamped
# import ros_numpy
# import cv2
import numpy as np
# import math
# from matplotlib import pyplot as plt
import utils

last_loop = time.time()


# https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2/20401#20401


class RobotMover:

    def __init__(self, robot):
        """
        :param robot: A Robot object. The functions to set motor speeds will be run on it.
        """
        self.robot = robot
        self.finish_listeners = [] # list of lambda functions
        self.maintain_angle = 0
        self.correction_mult = 0.7
        self.correction_thres = 0.03 # radians off of maintain_angle (1 rad = around 58 degrees)
        self.slow_thres = 0.5
        self.slow_mult = 0.7
        self.slow_angle_thres = 0.05
        self.slow_angle_mult = 0.9
        self.correction_overshoot = 0.01 # To get closer to the original course, correct until we're this many radians past the correct angle

    def add_finish_listener(self, func):
        self.finish_listeners.append(func)

    def move_forward(self, meters):
        """
            Starts moving forward
            lidar: a lidar object to add listeners to
            meters: the number of meters to move before stopping
        """
        lidar = self.robot.lidar
        print "Moving forward " + str(meters)
        pose = lidar.get_pose()
        print pose
        angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        self.maintain_angle = angle
        thres = None
        slow = None
        if (angle >= np.pi * -(1.0 / 4) and angle < np.pi * (1.0 / 4)): # Moving in roughly positive x direction
            delta = meters * np.cos(angle)
            slow_delta = (meters - self.slow_thres) * np.cos(angle)
            if self.slow_thres > meters:
                slow_delta = (meters / 2) * np.cos(angle)
            thres = Movement_Threshold(Movement_Threshold.X_AXIS, True, pose.position.x + delta, lambda: self.finish_step(lidar), "move")
            slow = Movement_Threshold(Movement_Threshold.X_AXIS, True, pose.position.x + slow_delta, lambda: self.slow_movement(lidar), "slow")
            print "moving in +x"
        elif (angle >= np.pi * (1.0 / 4) and angle < np.pi * (3.0 / 4)): # Roughly positive y
            delta = meters * np.sin(angle)
            slow_delta = (meters - self.slow_thres) * np.sin(angle)
            if self.slow_thres > meters:
                slow_delta = (meters / 2) * np.sin(angle)
            thres = Movement_Threshold(Movement_Threshold.Y_AXIS, True, pose.position.y + delta, lambda: self.finish_step(lidar), "move")
            slow = Movement_Threshold(Movement_Threshold.Y_AXIS, True, pose.position.y + slow_delta, lambda: self.slow_movement(lidar), "slow")
            print "moving in +y"
        elif (angle >= np.pi * (3.0 / 4) or angle < np.pi * -(3.0 / 4)): # Roughly negative x
            delta = meters * np.cos(angle) # This will be negative
            slow_delta = (meters - self.slow_thres) * np.cos(angle)
            if self.slow_thres > meters:
                slow_delta = (meters / 2) * np.cos(angle)
            thres = Movement_Threshold(Movement_Threshold.X_AXIS, False, pose.position.x + delta, lambda: self.finish_step(lidar), "move")
            slow = Movement_Threshold(Movement_Threshold.X_AXIS, False, pose.position.x + slow_delta, lambda: self.slow_movement(lidar), "slow")
            print "moving in -x"
        elif (angle < np.pi * -(1.0 / 4) and angle > np.pi * -(3.0 / 4)): # Roughly negative y
            delta = meters * np.sin(angle) # Negative
            slow_delta = (meters - self.slow_thres) * np.sin(angle)
            if self.slow_thres > meters:
                slow_delta = (meters / 2) * np.sin(angle)
            thres = Movement_Threshold(Movement_Threshold.Y_AXIS, False, pose.position.y + delta, lambda: self.finish_step(lidar), "move")
            slow = Movement_Threshold(Movement_Threshold.Y_AXIS, False, pose.position.y + slow_delta, lambda: self.slow_movement(lidar), "slow")
            print "moving in -y"
        assert thres != None
        lidar.add_listener(thres)
        lidar.add_listener(slow)
        low_correct = angle - self.correction_thres
        if (low_correct < -np.pi):
            low_correct += 2 * np.pi
        high_correct = angle + self.correction_thres
        if (high_correct > np.pi):
            high_correct -= 2 * np.pi
        # For correction
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, low_correct, lambda: self.correct_left(False), "correct")
        lidar.add_listener(thres)
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, high_correct, lambda: self.correct_right(False), "correct")
        lidar.add_listener(thres)

        # finished here
        # moving = 1
        # moving_meters = meters
        self.robot.set_speed(25)  # probably not right value

    def move_backward(self, meters):
        """
            Starts moving backward
            lidar: a lidar object to add listeners to
            meters: the number of meters to move before stopping
        """
        lidar = self.robot.lidar
        print "Moving backward " + str(meters)
        pose = lidar.get_pose()
        angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        self.maintain_angle = angle
        thres = None
        slow = None

        if (angle >= np.pi * -(1.0 / 4) and angle < np.pi * (1.0 / 4)): # Pointing in roughly positive x direction
            delta = -meters * np.cos(angle)
            slow_delta = -(meters - self.slow_thres) * np.cos(angle)
            if self.slow_thres > meters:
                slow_delta = -(meters / 2) * np.cos(angle)
            thres = Movement_Threshold(Movement_Threshold.X_AXIS, False, pose.position.x + delta, lambda: self.finish_step(lidar), "move")
            slow = Movement_Threshold(Movement_Threshold.X_AXIS, False, pose.position.x + slow_delta, lambda: self.slow_movement(lidar), "slow")
            print "positive x"
        elif (angle >= np.pi * (1.0 / 4) and angle < np.pi * (3.0 / 4)): # Roughly positive y
            delta = -meters * np.sin(angle)
            slow_delta = -(meters - self.slow_thres) * np.sin(angle)
            if self.slow_thres > meters:
                slow_delta = -(meters / 2) * np.sin(angle)
            thres = Movement_Threshold(Movement_Threshold.Y_AXIS, False, pose.position.y + delta, lambda: self.finish_step(lidar), "move")
            slow = Movement_Threshold(Movement_Threshold.Y_AXIS, False, pose.position.y + slow_delta, lambda: self.slow_movement(lidar), "slow")

        elif (angle >= np.pi * (3.0 / 4) or angle < np.pi * -(3.0 / 4)): # Roughly negative x
            delta = -meters * np.cos(angle)
            slow_delta = -(meters - self.slow_thres) * np.cos(angle)
            if self.slow_thres > meters:
                slow_delta = -(meters / 2) * np.cos(angle)
            thres = Movement_Threshold(Movement_Threshold.X_AXIS, True, pose.position.x + delta, lambda: self.finish_step(lidar), "move")
            slow = Movement_Threshold(Movement_Threshold.X_AXIS, True, pose.position.x + slow_delta, lambda: self.slow_movement(lidar), "slow")

        elif (angle < np.pi * -(1.0 / 4) and angle > np.pi * -(3.0 / 4)): # Roughly negative y
            delta = -meters * np.sin(angle)
            slow_delta = -(meters - self.slow_thres) * np.sin(angle)
            if self.slow_thres > meters:
                slow_delta = -(meters / 2) * np.sin(angle)
            thres = Movement_Threshold(Movement_Threshold.Y_AXIS, True, pose.position.y + delta, lambda: self.finish_step(lidar), "move")
            slow = Movement_Threshold(Movement_Threshold.Y_AXIS, True, pose.position.y + slow_delta, lambda: self.slow_movement(lidar), "slow")

        assert thres != None
        lidar.add_listener(thres)
        lidar.add_listener(slow)

        low_correct = angle - self.correction_thres # 1 degree
        if (low_correct < -np.pi):
            low_correct += 2 * np.pi
        high_correct = angle + self.correction_thres
        if (high_correct > np.pi):
            high_correct -= 2 * np.pi
        # For correction
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, low_correct, lambda: self.correct_left(True), "correct")
        lidar.add_listener(thres)
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, high_correct, lambda: self.correct_right(True), "correct")
        lidar.add_listener(thres)
        # starting_pose = pose
        # moving = -1
        # moving_meters = meters
        self.robot.set_speed(-25)  # probably not right value

    def rotate_left(self, degrees):
        """
            Starts a left/counterclockwise rotation
            lidar: a lidar object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        lidar = self.robot.lidar
        print "Rotating left " + str(degrees)
        pose = lidar.get_pose()
        angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = angle + deltaRadians
        if targetRadians > np.pi:
            targetRadians -= 2 * np.pi # Going from positive angle to negative
        slowRadians = targetRadians - self.slow_angle_thres
        if slowRadians < -np.pi:
            slowRadians += 2 * np.pi

        print "Current: " + str(angle)
        print "Target: " + str(targetRadians)
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, targetRadians, lambda: self.finish_step(lidar), "rotate")
        slow = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, slowRadians, lambda: self.slow_rotation(lidar), "slow")

        lidar.add_listener(thres)
        lidar.add_listener(slow)

        self.robot.set_speeds(-25, 25)

    def rotate_left_imu(self, degrees):
        """
            Starts a left/counterclockwise rotation
            lidar: a lidar object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        imu = self.robot.imu
        print "Rotating left " + str(degrees)
        #pose = lidar.get_pose()
        #angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        angle = imu.get_euler()[0]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = angle + deltaRadians
        if targetRadians > np.pi:
            targetRadians -= 2 * np.pi # Going from positive angle to negative
        slowRadians = targetRadians - self.slow_angle_thres
        if slowRadians < -np.pi:
            slowRadians += 2 * np.pi

        print "Current: " + str(angle)
        print "Target: " + str(targetRadians)
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, targetRadians, lambda: self.finish_step(imu), "rotate")
        slow = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, slowRadians, lambda: self.slow_rotation(imu), "slow")

        imu.add_listener(thres)
        imu.add_listener(slow)

        self.robot.set_speeds(-25, 25)

    def rotate_right(self, degrees):
        """
            Starts a right/clockwise rotation
            lidar: a lidar object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        lidar = self.robot.lidar
        print "Rotating right " + str(degrees)
        pose = lidar.get_pose()
        angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = angle - deltaRadians
        if targetRadians < -np.pi:
            targetRadians += 2 * np.pi # Going from negative angle to positive
        slowRadians = targetRadians + self.slow_angle_thres
        if slowRadians > np.pi:
            slowRadians -= 2 * np.pi

        print "Current: " + str(angle)
        print "Target: " + str(targetRadians)
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, targetRadians, lambda: self.finish_step(lidar), "rotate")
        slow = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, slowRadians, lambda: self.slow_rotation(lidar), "slow")

        lidar.add_listener(thres)
        lidar.add_listener(slow)
        
        self.robot.set_speeds(25, -25)

    def rotate_right_imu(self, degrees):
        """
            Starts a right/clockwise rotation
            lidar: a lidar object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        imu = self.robot.imu
        print "Rotating right " + str(degrees)
        angle = imu.get_euler()[0]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = angle - deltaRadians
        if targetRadians < -np.pi:
            targetRadians += 2 * np.pi # Going from negative angle to positive
        slowRadians = targetRadians + self.slow_angle_thres
        if slowRadians > np.pi:
            slowRadians -= 2 * np.pi

        print "Current: " + str(angle)
        print "Target: " + str(targetRadians)
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, targetRadians, lambda: self.finish_step(imu), "rotate")
        slow = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, slowRadians, lambda: self.slow_rotation(imu), "slow")

        imu.add_listener(thres)
        imu.add_listener(slow)
        
        self.robot.set_speeds(25, -25)

    def correct_right(self, backing_up):
        """
            To use when rotation starts drifting left
        """
        lidar = self.robot.lidar
        print "correcting to the right"
        targetAngle = self.maintain_angle - self.correction_overshoot
        if targetAngle < -np.pi:
            targetAngle += 2 * np.pi
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, targetAngle, lambda: self.stop_correcting(True, backing_up), "stop correct")
        if backing_up:
            self.robot.set_speeds(self.robot.get_speeds()[0] * self.correction_mult, self.robot.get_speeds()[1])
        else:
            self.robot.set_speeds(self.robot.get_speeds()[0], self.robot.get_speeds()[1] * self.correction_mult)
        lidar.remove_listeners('correct')
        lidar.add_listener(thres)

    def correct_left(self, backing_up):
        lidar = self.robot.lidar
        print "correcting to the left"
        targetAngle = self.maintain_angle + self.correction_overshoot
        if targetAngle > np.pi:
            targetAngle -= 2 * np.pi
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, targetAngle, lambda: self.stop_correcting(False, backing_up), "stop correct")
        if backing_up:
            self.robot.set_speeds(self.robot.get_speeds()[0], self.robot.get_speeds()[1] * self.correction_mult)
        else:
            self.robot.set_speeds(self.robot.get_speeds()[0] * self.correction_mult, self.robot.get_speeds()[1])
        lidar.remove_listeners('correct')
        lidar.add_listener(thres)

    def stop_correcting(self, correcting_right, backing_up):
        lidar = self.robot.lidar
        print "back to straight"
        lidar.remove_listeners('stop correct')
        if correcting_right:
            self.robot.set_speeds(self.robot.get_speeds()[0], self.robot.get_speeds()[1] / self.correction_mult)
        else:
            self.robot.set_speeds(self.robot.get_speeds()[0] / self.correction_mult, self.robot.get_speeds()[1])

        low_correct = self.maintain_angle - self.correction_thres
        if (low_correct < -np.pi):
            low_correct += 2 * np.pi
        high_correct = self.maintain_angle + self.correction_thres
        if (high_correct > np.pi):
            high_correct -= 2 * np.pi
        # For correction
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, False, low_correct, lambda: self.correct_left(backing_up), "correct")
        lidar.add_listener(thres)
        thres = Movement_Threshold(Movement_Threshold.Z_ROTATION, True, high_correct, lambda: self.correct_right(backing_up), "correct")
        lidar.add_listener(thres)

    def finish_step(self, listening_object):
        """
            Removes all listeners from the given object, stops all motion, and runs the functions in finish_listeners
        """
        self.robot.stop()
        print "Step done"
        listening_object.remove_listeners('move')
        listening_object.remove_listeners('rotate')
        listening_object.remove_listeners('correct')
        listening_object.remove_listeners('stop correct')
        listening_object.remove_listeners('slow')
        for l in self.finish_listeners:
            l()

    def slow_movement(self, listening_object):
        self.robot.set_speeds(self.robot.get_speeds()[0] * self.slow_mult, self.robot.get_speeds()[1] * self.slow_mult)
        listening_object.remove_listeners("slow")

    def slow_rotation(self, listening_object):
        self.robot.set_speeds(self.robot.get_speeds()[0] * self.slow_angle_mult, self.robot.get_speeds()[1] * self.slow_angle_mult)
        listening_object.remove_listeners("slow")
    
    def stop(self):
        """
            Stop all motion of the robot
        :return:
        """
        print "STOPSTOPSTOPSTOP"
        # rotating = 0
        # moving = 0
        self.robot.stop()
