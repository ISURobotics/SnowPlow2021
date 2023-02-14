import Lidar

from Movement_Threshold import Movement_Threshold
import numpy as np
import utils

class Sensors:

    def __init__(self):
        self.lidar = Lidar(self)
        self.thresholds = []

    def init_lidar(self):
        self.lidar.wait_for_pose_set()
    
    def callback_lidar_pose(self, pose):
        for i in range(len(self.thresholds) - 1, -1, -1): # Gotta iterate backwards as stuff might get removed from the list
            t = self.thresholds[i]
            measured_val = 0
            above_thres = False
            if (t.axis == Movement_Threshold.X_AXIS):
                measured_val = pose.position.x
                #print t.value
                #print measured_val
                above_thres = (measured_val >= t.value)
            elif (t.axis == Movement_Threshold.Y_AXIS):
                measured_val = pose.position.y
                above_thres = (measured_val >= t.value)
            elif (t.axis == Movement_Threshold.Z_ROTATION):
                eulers = utils.quaternion_to_euler(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                measured_val = eulers[2] # z rotation or yaw
                # NEEDS TESTING. LOTS OF TESTING.
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
                #print above_thres
                #print t.trigger_when_above
                #print ""
            if above_thres == t.trigger_when_above:
                self.thresholds.pop(i)
                t.function() # run the lamba associated with the threshold

    def get_lidar_pose(self):
        return self.lidar.get_pose()
    
    def add_listener(self, threshold):
        self.thresholds.append(threshold)

    def remove_listeners(self, tag):
        """
            Removes all listeners with the given tag
        """
        for i in range(len(self.thresholds) - 1, -1, -1):
            if self.thresholds[i].tag == tag:
                self.thresholds.pop(i)