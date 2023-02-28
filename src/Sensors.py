import Lidar
import IMU

from Movement_Threshold import Movement_Threshold
import numpy as np
import utils

class Sensors:

    def __init__(self):
        self.lidar = Lidar(self)
        self.imu = IMU(self)
        self.lidar_pose = None
        self.thresholds = []

    def init_lidar(self):
        self.lidar.wait_for_pose_set()

    def init_imu(self):
        pass
    
    def callback_lidar_pose(self, pose):
        self.lidar_pose = pose
        self.check_threshold()

    def check_thresholds(self):
        for i in range(len(self.thresholds) - 1, -1, -1): # Gotta iterate backwards as stuff might get removed from the list
            t = self.thresholds[i]
            triggered = False
            if (t.axis == Movement_Threshold.X_AXIS):
                triggered = self.lidar_x_axis(t)
            elif (t.axis == Movement_Threshold.Y_AXIS):
                triggered = self.lidar_y_axis(t)
            elif (t.axis == Movement_Threshold.Z_ROTATION):
                triggered = self.lidar_z_rotation(t)
            if triggered:
                self.thresholds.pop(i)
                t.function() # run the lamba associated with the threshold

    def add_listener(self, threshold):
        self.thresholds.append(threshold)

    def remove_listeners(self, tag):
        """
            Removes all listeners with the given tag
        """
        for i in range(len(self.thresholds) - 1, -1, -1):
            if self.thresholds[i].tag == tag:
                self.thresholds.pop(i)

    def get_lidar_pose(self):
        return self.lidar.get_pose()
    
    #--------------------------------------------------------------------------------------------------------------------------
    # These functions are for checking movement thresholds. Maybe we should move this stuff to another file, but ehhhhhh...
    #--------------------------------------------------------------------------------------------------------------------------

    def lidar_x_axis(self, t):
        measured_val = self.lidar_pose.position.x
        above_thres = (measured_val >= t.value)
        return (above_thres == t.trigger_when_above)
    
    def lidar_y_axis(self, t):
        measured_val = self.lidar_pose.position.y
        above_thres = (measured_val >= t.value)
        return (above_thres == t.trigger_when_above)
    
    def lidar_z_rotation(self, t):
        eulers = utils.quaternion_to_euler(self.lidar_pose.orientation.x, self.lidar_pose.orientation.y, self.lidar_pose.orientation.z, self.lidar_pose.orientation.w)
        measured_val = eulers[2] # z rotation or yaw
        print "measured: " + str(measured_val)
        if t.trigger_when_above:
            if (t.value > np.pi / 2):
                above_thres = (measured_val >= t.value) or (measured_val < t.value - 3 * np.pi / 2) # target is close to 180 degrees (pi radians)
            else:
                above_thres = (measured_val >= t.value) and (measured_val < t.value + np.pi / 4)
        else:
            if (t.value < np.pi / 2):
                above_thres = (measured_val > t.value) and (measured_val <= t.value + 3 * np.pi / 2)
            else:
                above_thres = (measured_val > t.value) or (measured_val <= t.value - np.pi / 4)
        return (above_thres == t.trigger_when_above)
    
    def imu_z_rotation(self, t):
        # TODO
        return False