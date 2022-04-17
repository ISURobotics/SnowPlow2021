import time
import rospy
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

def callback_slam_pose(data):
    #global last_loop
    p = data.pose
    assert isinstance(p, Pose)

    #if time.time() - last_loop < .5:
     #   return
    print(p)
    print(p.position.x)

    p = ros_numpy.geometry.pose_to_numpy(p)

    print(p)

    # for pt in sorted(points, key=lambda x: x[3]):
    #     print(pt)
    # points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(p, remove_nans=False)
    #global pose
    # print(points.shape)
    #pose = points

    #x_pts = [pt[0] for pt in points]
    #y_pts = [pt[1] for pt in points]
    #col = [pt[3] for pt in points]

    #plt.figure()
    #plt.scatter(x_pts, y_pts, c=col)
    #plt.axes([0, 10, 0, 10])
    #plt.ylim(-15, 15)
    #plt.xlim(0, 15)
    # plt.axes(xlim=(-5, 5), ylim=(0, 3.5))
    #plt.show()

    # gen = point_cloud2.read_points(p, field_names=("x", "y", "z"), skip_nans=False)
    # np_arr = np.array([p for p in gen])
    # np_arr = np_arr.reshape((p.height, p.width, 3))
    # print(np_arr.shape)
    # print(points[0, 0, :], points[0, 1, :])
    # print(time.time() - last_loop)
    last_loop = time.time()
    # for p in gen:
    #     print(" x : %.3f  y: %.3f  z: %.3f" % (p[0], p[1], p[2]))
def laser_input(data):
    #global last_loop
    assert isinstance(data, LaserScan)
    s = data.ranges
    i = data.intensities

    #if time.time() - last_loop < .5:
     #   return
    print(s[190])
    print(i[190])

rospy.init_node('Testing', anonymous=False)
# bridge = CvBridge()

# sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

#sub_points = rospy.Subscriber("/slam_out_pose", PoseStamped, callback_slam_pose)
laser_points = rospy.Subscriber("/scan", LaserScan, laser_input)

rospy.spin()
while not rospy.is_shutdown():
    #print(point_cloud)
    # time.sleep()
    # print('loop')
    pass

def move_forward(pose, meters):
    pass
