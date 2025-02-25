from Lidar import *
from IMU import *
from RobotGPS import *

from Movement_Threshold import Movement_Threshold

class Sensors:
    """
        This class acts as a facade for the sensors on the robot, currently including the Lidar and IMU.
        It also stores all active listeners for sensor values.
    """
    def __init__(self, node):
        self.lidar = Lidar(self, node)
        self.imu = IMU(self, node)
        self.gps = RobotGPS(self, node)
        self.thresholds = []

    def init_lidar(self, node):
        self.lidar.wait_for_pose_set(node)

    def init_imu(self):
        pass

    def init_gps(self):
        pass
    
    def callback_sensor_data(self):
        """
            To be run every time any sensor recieves new data.
            This will check some sensors that haven't been updated since the last time this was checked, but that shouldn't be a huge deal.
        """
        self.check_thresholds()

    def check_thresholds(self):
        """
            To be run every time we get new data from a sensor. Iterates through the list of thresholds and runs their function if their
            axis_func returns true
        """
        
        for i in range(len(self.thresholds) - 1, -1, -1): # Gotta iterate backwards as stuff might get removed from the list
            t: Movement_Threshold = self.thresholds[i]
            triggered = t.axis_func(self, t)
            if triggered:
                self.thresholds.pop(i)
                t.function() # run the lamba associated with the threshold

    def add_listener(self, threshold: Movement_Threshold):
        self.thresholds.append(threshold)

    def remove_listeners(self, tag: str):
        """
            Removes all listeners with the given tag
        """
        for i in range(len(self.thresholds) - 1, -1, -1):
            if self.thresholds[i].tag == tag:
                self.thresholds.pop(i)

    def get_pose(self):
        """
            Uses our current method to get the robot's position.
            Currently, we use the GPS
        """
        return self.gps.get_pose()
    
    def get_obstacle_points(self):
        return self.lidar.prepare_obstacle_points()
    
    def get_movement_points(self, points_list):
        return self.lidar.prepare_movement_points(points_list)
    
    def get_euler(self):
        return self.imu.get_euler()
