import time
from Movement_Threshold import Movement_Threshold
import numpy as np
import Axes
import utils
from Robot import Robot

last_loop = time.time()


class Robot_Mover:

    def __init__(self, robot: Robot):
        """
        :param robot: A Robot object. The functions to set motor speeds will be run on it.
        """
        self.robot = robot
        self.finish_listeners: list[function] = [] # list of lambda functions
        self.maintain_angle = 0
        self.correction_mult = 0.6
        self.correction_thres = 0.015 # radians off of maintain_angle (1 rad = around 58 degrees)
        self.slow_thres = 1
        self.slow_mult = 0.7
        self.slow_angle_thres = 0.05
        self.slow_angle_mult = 0.9
        self.correction_overshoot = 0.01 # To get closer to the original course, correct until we're this many radians past the correct angle
        self.left_turn_offset = -0.006 # When turning left, increase our rotation by this value to counteract constant errors
        self.right_turn_offset = 0.013 # When turning right, increase our rotation by this value to counteract errors
        self.base_speed = 25

    def add_finish_listener(self, func):
        """
            Adds a lambda function to the list of things to do when the current step is finished.
            This does not apply to corrections, as they use a different stopping function from the
            movement functions.
        """
        self.finish_listeners.append(func)

    def move_forward(self, meters: float):
        """
            Starts moving forward and creates a listener to stop at a certain distance
            sensors: a sensors object to add listeners to
            meters: the number of meters to move before stopping
        """
        sensors = self.robot.sensors
        print ("Moving forward " + str(meters))
        pose = sensors.get_lidar_pose()
        angle = sensors.get_euler()[0]
        print (pose)
        #angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        self.maintain_angle = angle
        self.maintain_angle/=(np.pi/2)
        self.maintain_angle=round(self.maintain_angle)
        self.maintain_angle*=(np.pi/2)
        thres = None
        slow = None
        if (angle >= np.pi * -(1.0 / 4) and angle < np.pi * (1.0 / 4)): # Moving in roughly positive x direction
            delta = meters * np.cos(angle)
            slow_delta = (meters - self.slow_thres) * np.cos(angle)
            if self.slow_thres > meters:
                slow_delta = (meters / 2) * np.cos(angle)
            thres = Movement_Threshold(Axes.LIDAR_X, True, pose.position.x + delta, lambda: self.finish_step(), "move")
            slow = Movement_Threshold(Axes.LIDAR_X, True, pose.position.x + slow_delta, lambda: self.slow_movement(), "slow")
            print ("moving in +x")
        elif (angle >= np.pi * (1.0 / 4) and angle < np.pi * (3.0 / 4)): # Roughly positive y
            delta = meters * np.sin(angle)
            slow_delta = (meters - self.slow_thres) * np.sin(angle)
            if self.slow_thres > meters:
                slow_delta = (meters / 2) * np.sin(angle)
            thres = Movement_Threshold(Axes.LIDAR_Y, True, pose.position.y + delta, lambda: self.finish_step(), "move")
            slow = Movement_Threshold(Axes.LIDAR_Y, True, pose.position.y + slow_delta, lambda: self.slow_movement(), "slow")
            print ("moving in +y")
        elif (angle >= np.pi * (3.0 / 4) or angle < np.pi * -(3.0 / 4)): # Roughly negative x
            delta = meters * np.cos(angle) # This will be negative
            slow_delta = (meters - self.slow_thres) * np.cos(angle)
            if self.slow_thres > meters:
                slow_delta = (meters / 2) * np.cos(angle)
            thres = Movement_Threshold(Axes.LIDAR_X, False, pose.position.x + delta, lambda: self.finish_step(), "move")
            slow = Movement_Threshold(Axes.LIDAR_X, False, pose.position.x + slow_delta, lambda: self.slow_movement(), "slow")
            print ("moving in -x")
        elif (angle < np.pi * -(1.0 / 4) and angle > np.pi * -(3.0 / 4)): # Roughly negative y
            delta = meters * np.sin(angle) # Negative
            slow_delta = (meters - self.slow_thres) * np.sin(angle)
            if self.slow_thres > meters:
                slow_delta = (meters / 2) * np.sin(angle)
            thres = Movement_Threshold(Axes.LIDAR_Y, False, pose.position.y + delta, lambda: self.finish_step(), "move")
            slow = Movement_Threshold(Axes.LIDAR_Y, False, pose.position.y + slow_delta, lambda: self.slow_movement(), "slow")
            print ("moving in -y")
        assert thres != None
        sensors.add_listener(thres)
        sensors.add_listener(slow)
        low_correct = angle - self.correction_thres
        if (low_correct < -np.pi):
            low_correct += 2 * np.pi
        high_correct = angle + self.correction_thres
        if (high_correct > np.pi):
            high_correct -= 2 * np.pi
        # For correction
        thres = Movement_Threshold(Axes.IMU_ROT, False, low_correct, lambda: self.correct_left(False), "correct")
        sensors.add_listener(thres)
        thres = Movement_Threshold(Axes.IMU_ROT, True, high_correct, lambda: self.correct_right(False), "correct")
        sensors.add_listener(thres)
        self.robot.set_speed(self.base_speed)  # probably not right value

    def move_backward(self, meters: float):
        """
            Starts moving backward and creates a listener to stop at a certain distance
            sensors: a sensors object to add listeners to
            meters: the number of meters to move before stopping
        """
        sensors = self.robot.sensors
        print ("Moving backward " + str(meters))
        pose = sensors.get_lidar_pose()
        angle = sensors.get_euler()[0]
        #angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        self.maintain_angle = angle
        thres = None
        slow = None

        if (angle >= np.pi * -(1.0 / 4) and angle < np.pi * (1.0 / 4)): # Pointing in roughly positive x direction
            delta = -meters * np.cos(angle)
            slow_delta = -(meters - self.slow_thres) * np.cos(angle)
            if self.slow_thres > meters:
                slow_delta = -(meters / 2) * np.cos(angle)
            thres = Movement_Threshold(Axes.LIDAR_X, False, pose.position.x + delta, lambda: self.finish_step(), "move")
            slow = Movement_Threshold(Axes.LIDAR_X, False, pose.position.x + slow_delta, lambda: self.slow_movement(), "slow")
            print ("positive x")
        elif (angle >= np.pi * (1.0 / 4) and angle < np.pi * (3.0 / 4)): # Roughly positive y
            delta = -meters * np.sin(angle)
            slow_delta = -(meters - self.slow_thres) * np.sin(angle)
            if self.slow_thres > meters:
                slow_delta = -(meters / 2) * np.sin(angle)
            thres = Movement_Threshold(Axes.LIDAR_Y, False, pose.position.y + delta, lambda: self.finish_step(), "move")
            slow = Movement_Threshold(Axes.LIDAR_Y, False, pose.position.y + slow_delta, lambda: self.slow_movement(), "slow")

        elif (angle >= np.pi * (3.0 / 4) or angle < np.pi * -(3.0 / 4)): # Roughly negative x
            delta = -meters * np.cos(angle)
            slow_delta = -(meters - self.slow_thres) * np.cos(angle)
            if self.slow_thres > meters:
                slow_delta = -(meters / 2) * np.cos(angle)
            thres = Movement_Threshold(Axes.LIDAR_X, True, pose.position.x + delta, lambda: self.finish_step(), "move")
            slow = Movement_Threshold(Axes.LIDAR_X, True, pose.position.x + slow_delta, lambda: self.slow_movement(), "slow")

        elif (angle < np.pi * -(1.0 / 4) and angle > np.pi * -(3.0 / 4)): # Roughly negative y
            delta = -meters * np.sin(angle)
            slow_delta = -(meters - self.slow_thres) * np.sin(angle)
            if self.slow_thres > meters:
                slow_delta = -(meters / 2) * np.sin(angle)
            thres = Movement_Threshold(Axes.LIDAR_Y, True, pose.position.y + delta, lambda: self.finish_step(), "move")
            slow = Movement_Threshold(Axes.LIDAR_Y, True, pose.position.y + slow_delta, lambda: self.slow_movement(), "slow")

        assert thres != None
        sensors.add_listener(thres)
        sensors.add_listener(slow)

        low_correct = angle - self.correction_thres # 1 degree
        if (low_correct < -np.pi):
            low_correct += 2 * np.pi
        high_correct = angle + self.correction_thres
        if (high_correct > np.pi):
            high_correct -= 2 * np.pi
        # For correction
        thres = Movement_Threshold(Axes.IMU_ROT, False, low_correct, lambda: self.correct_left(True), "correct")
        sensors.add_listener(thres)
        thres = Movement_Threshold(Axes.IMU_ROT, True, high_correct, lambda: self.correct_right(True), "correct")
        sensors.add_listener(thres)
        self.robot.set_speed(-self.base_speed)  # probably not right value

    def rotate_left(self, degrees: float):
        """
            Starts a left/counterclockwise rotation and creates a listener to stop at a certain angle
            sensors: a sensors object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        sensors = self.robot.sensors
        print("Rotating left " + str(degrees))
        pose = sensors.get_pose()
        angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        thres = None
        deltaRadians = degrees * (np.pi / 180) + self.left_turn_offset
        targetRadians = angle + deltaRadians
        targetRadians /= (np.pi / 2)
        targetRadians = round(targetRadians)
        targetRadians *= (np.pi / 2)
        targetRadians += self.left_turn_offset
        if targetRadians > np.pi:
            targetRadians -= 2 * np.pi # Going from positive angle to negative
        slowRadians = targetRadians - self.slow_angle_thres
        if slowRadians < -np.pi:
            slowRadians += 2 * np.pi

        print("Current: " + str(angle))
        print("Target: " + str(targetRadians))
        thres = Movement_Threshold(Axes.LIDAR_ROT, True, targetRadians, lambda: self.finish_step(), "rotate")
        slow = Movement_Threshold(Axes.LIDAR_ROT, True, slowRadians, lambda: self.slow_rotation(), "slow")

        sensors.add_listener(thres)
        sensors.add_listener(slow)

        self.robot.set_speeds(-self.base_speed, self.base_speed)

    def rotate_left_imu(self, degrees:float):
        """
            Starts a left/counterclockwise rotation
            lidar: a lidar object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        sensors = self.robot.sensors
        print("Rotating left " + str(degrees))
        #pose = lidar.get_pose()
        #angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        angle = sensors.get_euler()[0]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = angle + deltaRadians
        targetRadians /= (np.pi / 2)
        targetRadians = round(targetRadians)
        targetRadians *= (np.pi / 2)
        targetRadians += self.left_turn_offset
        if targetRadians > np.pi:
            targetRadians -= 2 * np.pi # Going from positive angle to negative
        slowRadians = targetRadians - self.slow_angle_thres
        if slowRadians < -np.pi:
            slowRadians += 2 * np.pi

        print("Current: " + str(angle))
        print("Target: " + str(targetRadians))
        thres = Movement_Threshold(Axes.IMU_ROT, True, targetRadians, lambda: self.finish_step(), "rotate")
        slow = Movement_Threshold(Axes.IMU_ROT, True, slowRadians, lambda: self.slow_rotation(), "slow")

        sensors.add_listener(thres)
        sensors.add_listener(slow)

        self.robot.set_speeds(-self.base_speed, self.base_speed)

    def rotate_right(self, degrees: float):
        """
            Starts a right/clockwise rotation and creates a listener to stop at a certain angle
            sensors: a sensors object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        sensors = self.robot.sensors
        print("Rotating right " + str(degrees))
        pose = sensors.get_pose()
        angle = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = angle - deltaRadians
        targetRadians /= (np.pi / 2)
        targetRadians = round(targetRadians)
        targetRadians *= (np.pi / 2)
        targetRadians -= self.right_turn_offset
        if targetRadians < -np.pi:
            targetRadians += 2 * np.pi # Going from negative angle to positive
        slowRadians = targetRadians + self.slow_angle_thres
        if slowRadians > np.pi:
            slowRadians -= 2 * np.pi
        print("Current: " + str(angle))
        print("Target: " + str(targetRadians))
        thres = Movement_Threshold(Axes.LIDAR_ROT, False, targetRadians, lambda: self.finish_step(), "rotate")
        slow = Movement_Threshold(Axes.LIDAR_ROT, False, slowRadians, lambda: self.slow_rotation(), "slow")

        sensors.add_listener(thres)
        sensors.add_listener(slow)
        
        self.robot.set_speeds(self.base_speed, -self.base_speed)

    def rotate_right_imu(self, degrees: float):
        """
            Starts a right/clockwise rotation
            lidar: a lidar object to add listeners to
            degrees: The number of degrees to turn from the current pose before stopping
        """
        sensors = self.robot.sensors
        print("Rotating right " + str(degrees))
        angle = sensors.get_euler()[0]
        thres = None
        deltaRadians = degrees * (np.pi / 180)
        targetRadians = angle - deltaRadians
        targetRadians /= (np.pi / 2)
        targetRadians = round(targetRadians)
        targetRadians *= (np.pi / 2)
        targetRadians -= self.right_turn_offset
        if targetRadians < -np.pi:
            targetRadians += 2 * np.pi # Going from negative angle to positive
        slowRadians = targetRadians + self.slow_angle_thres
        if slowRadians > np.pi:
            slowRadians -= 2 * np.pi

        print("Current: " + str(angle))
        print("Target: " + str(targetRadians))
        thres = Movement_Threshold(Axes.IMU_ROT, False, targetRadians, lambda: self.finish_step(), "rotate")
        slow = Movement_Threshold(Axes.IMU_ROT, False, slowRadians, lambda: self.slow_rotation(), "slow")

        sensors.add_listener(thres)
        sensors.add_listener(slow)
        self.robot.set_speeds(self.base_speed, -self.base_speed)

    def correct_right(self, backing_up: bool):
        """
            To use when the robot starts drifting left. Slows down the right wheels until the angle is back to straight
        """
        sensors = self.robot.sensors
        print("correcting to the right")
        targetAngle = self.maintain_angle - self.correction_overshoot
        if targetAngle < -np.pi:
            targetAngle += 2 * np.pi
        thres = Movement_Threshold(Axes.IMU_ROT, False, targetAngle, lambda: self.stop_correcting(True, backing_up), "stop correct")
        if backing_up:
            self.robot.set_speeds(self.robot.get_speeds()[0] * self.correction_mult, self.robot.get_speeds()[1])
        else:
            self.robot.set_speeds(self.robot.get_speeds()[0], self.robot.get_speeds()[1] * self.correction_mult)
        sensors.remove_listeners('correct')
        sensors.add_listener(thres)

    def correct_left(self, backing_up: bool):
        """
            To use when the robot starts drifting right. Slows down the left wheels until the angle is back to straight
        """
        print("correcting to the left")
        sensors = self.robot.sensors
        targetAngle = self.maintain_angle + self.correction_overshoot
        if targetAngle > np.pi:
            targetAngle -= 2 * np.pi
        thres = Movement_Threshold(Axes.IMU_ROT, True, targetAngle, lambda: self.stop_correcting(False, backing_up), "stop correct")
        if backing_up:
            self.robot.set_speeds(self.robot.get_speeds()[0], self.robot.get_speeds()[1] * self.correction_mult)
        else:
            self.robot.set_speeds(self.robot.get_speeds()[0] * self.correction_mult, self.robot.get_speeds()[1])
        sensors.remove_listeners('correct')
        sensors.add_listener(thres)

    def stop_correcting(self, correcting_right: bool, backing_up: bool):
        """
            Re-equalizes the speed of the wheels. Use when the robot is back to straight after correcting
        """
        print("back to straight")
        sensors = self.robot.sensors
        sensors.remove_listeners('stop correct')
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
        thres = Movement_Threshold(Axes.IMU_ROT, False, low_correct, lambda: self.correct_left(backing_up), "correct")
        sensors.add_listener(thres)
        thres = Movement_Threshold(Axes.IMU_ROT, True, high_correct, lambda: self.correct_right(backing_up), "correct")
        sensors.add_listener(thres)

    def finish_step(self):
        """
            Stops all wheel motion and removes all movement listeners. Runs finish listeners.
        """
        sensors = self.robot.sensors
        self.robot.stop()
        print("Step done")
        sensors.remove_listeners('move')
        sensors.remove_listeners('rotate')
        sensors.remove_listeners('correct')
        sensors.remove_listeners('stop correct')
        sensors.remove_listeners('slow')
        for l in self.finish_listeners:
            l()

    def slow_movement(self):
        """
            Multiplies both wheel speeds by slow_mult. Use this when close to the goal for a forward or backward movement.
        """
        sensors = self.robot.sensors
        self.robot.set_speeds(self.robot.get_speeds()[0] * self.slow_mult, self.robot.get_speeds()[1] * self.slow_mult)
        sensors.remove_listeners("slow")

    def slow_rotation(self):
        """
            Multiplies both wheel speeds by slow_angle_mult. Use this when close to the goal for a rotation
        """
        sensors = self.robot.sensors
        self.robot.set_speeds(self.robot.get_speeds()[0] * self.slow_angle_mult, self.robot.get_speeds()[1] * self.slow_angle_mult)
        sensors.remove_listeners("slow")
    
    def stop(self):
        """
            Stop all motion of the robot
        """
        print("STOPSTOPSTOPSTOP")
        # rotating = 0
        # moving = 0
        self.robot.stop()
