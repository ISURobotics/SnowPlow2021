import time
import rclpy
import sys, os

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Int16, Int32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import ros_numpy
import cv2
import numpy as np
from queue import Queue
from matplotlib import pyplot as plt
point_cloud = None
last_loop = time.time()

#https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2/20401#20401


def callback_pointcloud(data):
    global last_loop
    assert isinstance(data, PointCloud2)

    if time.time() - last_loop < .5:
        return

    points = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    # for pt in sorted(points, key=lambda x: x[3]):
    #     print(pt)
    # points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False)
    global point_cloud
    # print(points.shape)
    point_cloud = points

    x_pts = [pt[0] for pt in points]
    y_pts = [pt[1] for pt in points]
    col = [pt[3] for pt in points]

    plt.figure()
    plt.scatter(x_pts, y_pts, c=col)
    #plt.axes([0, 10, 0, 10])
    plt.ylim(-15, 15)
    plt.xlim(0, 15)
    # plt.axes(xlim=(-5, 5), ylim=(0, 3.5))
    plt.show()

    # gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=False)
    # np_arr = np.array([p for p in gen])
    # np_arr = np_arr.reshape((data.height, data.width, 3))
    # print(np_arr.shape)
    # print(points[0, 0, :], points[0, 1, :])
    # print(time.time() - last_loop)
    last_loop = time.time()
    # for p in gen:
    #     print(" x : %.3f  y: %.3f  z: %.3f" % (p[0], p[1], p[2]))


rclpy.init_node('Testing', anonymous=False)
# bridge = CvBridge()

# sub_image = rclpy.Subscriber("/camera/color/image_raw", Image, image_callback)
sub_points = rclpy.Subscriber("/cloud", PointCloud2, callback_pointcloud)

rclpy.spin()
while not rclpy.is_shutdown():
    #print(point_cloud)
    # time.sleep()
    # print('loop')
    pass
