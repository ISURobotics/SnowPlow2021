import utils
import numpy as np
from Sensors import Sensors
from Movement_Threshold import Movement_Threshold

"""
    These lambdas are to be used as part of a strategy pattern for checking whether a movement threshold has been reached.
    When we add a new axis that we might keep track of when moving, just implement the function in this file and add the 
    short name to the list of constants below. This cuts down on code duplication and makes adding new sensors much easier,
    which wouldn't be possible without a good architecture, Sergio.
"""

LIDAR_X = lambda p_sensors, p_threshold: x_axis(p_sensors, p_threshold)
LIDAR_Y = lambda p_sensors, p_threshold: y_axis(p_sensors, p_threshold)
LIDAR_ROT = lambda p_sensors, p_threshold: lidar_z_rotation(p_sensors, p_threshold)
IMU_ROT = lambda p_sensors, p_threshold: imu_z_rotation(p_sensors, p_threshold)

"""
    These functions return true if the threshold t has been reached given the data accessed using the sensors facade
    and false if it hasn't.

    The data stored in t is defined in Movement_Threshold.py. The properties relevant to these functions are
    t.value and t.trigger_when_above
"""

def x_axis(sensors: Sensors, t: Movement_Threshold):
    """
        For movement along the x axis using the GPS (formerly the Lidar)
    """
    pose = sensors.get_pose()
    measured_val = pose.position.x
    print("measured x position: ", measured_val)
    print("target x position: ", t.value)
    above_thres = (measured_val >= t.value)
    return (above_thres == t.trigger_when_above)

def y_axis(sensors: Sensors, t: Movement_Threshold):
    """
        For movement along the y axis using the GPS (formerly the Lidar)
    """
    pose = sensors.get_pose()
    measured_val = pose.position.y
    print("measured y position: ", measured_val)
    print("target y position: ", t.value)
    above_thres = (measured_val >= t.value)
    return (above_thres == t.trigger_when_above)

def lidar_z_rotation(sensors: Sensors, t: Movement_Threshold):
    """
        For rotation using the Lidar and SLAM. Not used since the IMU is more reliable
        and Hector SLAM never got ported to ROS 2
    """
    lidar_pose = sensors.get_pose()
    eulers = utils.quaternion_to_euler(lidar_pose.orientation.x, lidar_pose.orientation.y, lidar_pose.orientation.z, lidar_pose.orientation.w)
    measured_val = eulers[2] # z rotation or yaw
    print ("measured: " + str(measured_val))
    # I tested and found some bugs with this implementation. Fixes are implemented
    # if t.trigger_when_above:
    #     if (t.value > np.pi / 2):
    #         above_thres = (measured_val >= t.value) or (measured_val < t.value - 3 * np.pi / 2) # target is close to 180 degrees (pi radians)
    #     else:
    #         above_thres = (measured_val >= t.value) and (measured_val < t.value + np.pi / 4)
    # else:
    #     if (t.value < np.pi / 2):
    #         above_thres = (measured_val > t.value) and (measured_val <= t.value + 3 * np.pi / 2)
    #     else:
    #         above_thres = (measured_val > t.value) or (measured_val <= t.value - np.pi / 4)
    if t.trigger_when_above:
        if (t.value > np.pi / 2):
            above_thres = (measured_val >= t.value) or (measured_val < t.value - 3 * np.pi / 2) # target is close to 180 degrees (pi radians)
        else:
            above_thres = (measured_val >= t.value) and (measured_val < t.value + np.pi / 2)
    else:
        if (t.value > np.pi / 2):
            above_thres = (measured_val > t.value) or (measured_val <= t.value - np.pi / 2)
        else:
            above_thres = (measured_val > t.value) and (measured_val <= t.value + 3 * np.pi / 2)
    return (above_thres == t.trigger_when_above)

def imu_z_rotation(sensors: Sensors, t: Movement_Threshold):
    """
        For rotation using the IMU's gyroscope and magnetometer
    """
    measured_val = 0
    above_thres = False
    eulers = sensors.get_euler()
    measured_val = eulers[0] # The IMU is oriented so that its x axis is up and down, so we need that
    print ("measured: " + str(measured_val))
    print ("target: " + str(t.value))
    if t.trigger_when_above:
        if (t.value > np.pi / 2):
            above_thres = (measured_val >= t.value) or (measured_val < t.value - 3 * np.pi / 2) # target is close to 180 degrees (pi radians)
        else:
            above_thres = (measured_val >= t.value) and (measured_val < t.value + np.pi / 2)
    else:
        if (t.value > -np.pi / 2):
            above_thres = (measured_val > t.value) or (measured_val <= t.value - np.pi / 2)
        else:
            above_thres = (measured_val > t.value) and (measured_val <= t.value + 3 * np.pi / 2)

    print (above_thres)
    print (t.trigger_when_above)
    print ("")

    return (above_thres == t.trigger_when_above)
