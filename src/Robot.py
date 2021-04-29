import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import PointCloud2
import ros_numpy
import matplotlib as plt

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
        self.points = None
        self._sub_points = rospy.Subscriber("/cloud", PointCloud2, self.callback_pointcloud)

    def callback_pointcloud(self, data):
        self.points = ros_numpy.point_cloud2.pointcloud2_to_array(data)  # type: List[tuple]
        # points data is returned as (x, y, color)

    def get_points(self):
        return self.points  # points data is returned as (x, y, color)

    def plot_points(self):
        points = self.points
        x_pts = [pt[0] for pt in points]
        y_pts = [pt[1] for pt in points]
        col = [pt[3] for pt in points]

        plt.figure()
        plt.scatter(x_pts, y_pts, c=col)
        # plt.axes([0, 10, 0, 10])
        plt.ylim(-15, 15)
        plt.xlim(0, 15)
        # plt.axes(xlim=(-5, 5), ylim=(0, 3.5))
        plt.show()


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
