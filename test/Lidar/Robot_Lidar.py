import time
import rclpy
import sys, os

from sensor_msgs.msg import Image, PointCloud2, LaserScan
from std_msgs.msg import Int16, Int32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Transform, Vector3, Quaternion, Point, Pose, PoseStamped
import ros_numpy
import cv2
import numpy as np
from matplotlib import pyplot as plt

point_cloud = None
last_loop = time.time()
'''
    FOR TESTING ONLY
'''

# https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2/20401#20401

oldMeters = 0
movingForward = False

# NOT IN USE ANY LONGER!!!

def callback_slam_pose(data):
    # global last_loop
    p = data.pose
    assert isinstance(p, Pose)

    # if time.time() - last_loop < .5:
    #   return
    print(p)
    print(p.position.x)

    p = ros_numpy.geometry.pose_to_numpy(p)

    print(p)
    """ old
    if movingForward:
        dist = Math.sqrt(
            Math.pow(p.position.x - oldPos.position.x, 2) + Math.pow(p.position.y - oldPos.position.y, 2) + Math.pow(
                p.position.z - oldPos.position.z, 2))
        if dist >= oldMeters:
            motor.setSpeed(0)
            movingForward = false
    """

    last_loop = time.time()


def laser_input(data):
    # global last_loop
    assert isinstance(data, LaserScan)
    s = data.ranges
    i = data.intensities

    # if time.time() - last_loop < .5:
    #   return
    print(s[190])
    print(i[190])


rclpy.init_node('Testing', anonymous=False)
# bridge = CvBridge()

# sub_image = rclpy.Subscriber("/camera/color/image_raw", Image, image_callback)

curr_pose = rclpy.Subscriber("/slam_out_pose", PoseStamped, callback_slam_pose)
# laser_points = rclpy.Subscriber("/scan", LaserScan, laser_input)

rclpy.spin()
while not rclpy.is_shutdown():
    # print(point_cloud)
    # time.sleep()
    # print('loop')
    pass
