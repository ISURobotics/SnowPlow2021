import utils
import numpy as np

LIDAR_X = lambda p_pose, p_threshold: lidar_x_axis(p_pose, p_threshold)
LIDAR_Y = lambda p_pose, p_threshold: lidar_y_axis(p_pose, p_threshold)
LIDAR_ROT = lambda p_pose, p_threshold: lidar_z_rotation(p_pose, p_threshold)
IMU_ROT = lambda p_pose, p_threshold: imu_z_rotation(p_pose, p_threshold)

def lidar_x_axis(lidar_pose, t):
    measured_val = lidar_pose.position.x
    above_thres = (measured_val >= t.value)
    return (above_thres == t.trigger_when_above)

def lidar_y_axis(lidar_pose, t):
    measured_val = lidar_pose.position.y
    above_thres = (measured_val >= t.value)
    return (above_thres == t.trigger_when_above)

def lidar_z_rotation(lidar_pose, t):
    eulers = utils.quaternion_to_euler(lidar_pose.orientation.x, lidar_pose.orientation.y, lidar_pose.orientation.z, lidar_pose.orientation.w)
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