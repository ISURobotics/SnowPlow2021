from turtle import right
from matplotlib.pyplot import axis
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from Movement_Threshold import Movement_Threshold
import ros_numpy
import matplotlib as plt
import utils
import numpy as np

# Note: Before running this file, be sure to start roscore and rosrun the rosserial_python
# >> rosrun rosserial_python serial_node.py
from typing import List


class Motor:
    def __init__(self, path):
        self._speed_pub = rospy.Publisher("{}/speed".format(path), Int8, queue_size=1)
        self.speed = 0

    def set_speed(self, speed):
        assert -100 < speed < 100
        value = Int8()
        value.data = speed
        self.speed = speed
        self._speed_pub.publish(value)


class Lidar:
    def __init__(self):
        self.points = []
        self.pose = None
        self._sub_points = rospy.Subscriber("/cloud", PointCloud2, self.callback_pointcloud)
        self._sub_pose = rospy.Subscriber("/slam_out_pose", PoseStamped, self.callback_slam_pose)
        self.thresholds = []
        self.pose_set = False

    def callback_pointcloud(self, data):
        self.points = ros_numpy.point_cloud2.pointcloud2_to_array(data)  # type: List[tuple]
        # points data is returned as (x, y, color)

    def callback_slam_pose(self, data):
        print "poseee"
        if not self.pose_set:
            self.pose = data
            self.pose_set = True
        for i in range(len(self.thresholds) - 1, 0, -1): # Gotta iterate backwards as stuff might get removed from the list
            t = self.thresholds[i]
            measured_val = 0
            above_thres = False
            if (t.axis == Movement_Threshold.X_AXIS):
                measured_val = data.position.x
                above_thres = (measured_val >= t.value)
            elif (t.axis == Movement_Threshold.Y_AXIS):
                measured_val = data.position.y
                above_thres = (measured_val >= t.value)
            elif (t.axis == Movement_Threshold.Z_ROTATION):
                eulers = utils.quaternion_to_euler(data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w)
                measured_val = eulers[2] # z rotation or yaw
                # NEEDS TESTING. LOTS OF TESTING.
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
            print above_thres
            if above_thres == t.trigger_when_above:
                self.thresholds.remove(i)
                t.function() # run the lamba associated with the threshold

    def get_points(self):
        return self.points  # points data is returned as (x, y, color)

    def get_pose(self):
        return self.pose

    # def plot_points(self):
    #     points = self.points
    #     x_pts = [pt[0] for pt in points]
    #     y_pts = [pt[1] for pt in points]
    #     col = [pt[3] for pt in points]

    #     plt.figure()
    #     plt.scatter(x_pts, y_pts, c=col)
    #     # plt.axes([0, 10, 0, 10])
    #     plt.ylim(-15, 15)
    #     plt.xlim(0, 15)
    #     # plt.axes(xlim=(-5, 5), ylim=(0, 3.5))
    #     plt.show()

    def add_listener(self, threshold):
        self.thresholds.append(threshold)



class Robot:
    def __init__(self, rospy_init=True):
        if rospy_init:
            rospy.init_node('Robot', anonymous=False)
        self.left = Motor('left_motor')
        self.right = Motor('right_motor')
        self.lidar = Lidar()

    def stop(self):
        self.left.set_speed(0)
        self.right.set_speed(0)

    def set_speeds(self, speed):
        self.left.set_speed(speed)
        self.right.set_speed(speed)

    def set_speed(self, leftSpeed, rightSpeed):
        self.left.set_speed(leftSpeed)
        self.right.set_speed(rightSpeed)
