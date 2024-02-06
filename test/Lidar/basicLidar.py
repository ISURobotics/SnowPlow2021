import time
import rospy
import sys, os
import Robot

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

'''
    NOT USED ANYMORE!!!!
'''

movingMeters = 0 # Meters to move from start before stopping
rotating = 0 # -1 for left/counterclockwise, 0 for not rotating, 1 for right/clockwise
rotatingRadians = 0 # Radians to rotate from start before stopping
moving = 0 # -1 for backward, 0 for not moving, 1 for forward
startingPose = None # Pose object for moving and rotating

# NOT IN USE ANY LONGER!!!

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
    # plt.axes([0, 10, 0, 10])
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

    #p = ros_numpy.geometry.pose_to_numpy(p)

    print(p)

    #move forward and backward
    if moving != 0:
        d = dist(p, startingPose)
        if dist >= movingMeters:
            motor.setSpeed(0)
            moving = 0

    # rotate left and right
    if rotating != 0:
        a = angleBetween(p, startingPose)
        if a >= rotatingRadians:
            motor.left.setSpeed(0)
            motor.right.setSpeed(0)
            rotating = 0

    last_loop = time.time()
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

    curr_pose = rospy.Subscriber("/slam_out_pose", PoseStamped, callback_slam_pose)
    #laser_points = rospy.Subscriber("/scan", LaserScan, laser_input)

    rospy.spin()
while not rospy.is_shutdown():
    #print(point_cloud)
    # time.sleep()
    # print('loop')
    pass

def move_forward(pose, meters):
    '''
        Starts moving forward
        pose: The pose object produced by the slam_out_pose topic
        meters: the number of meters to move before stopping
    '''
    startingPose = pose
    moving = 1
    movingMeters = meters
    motor.setSpeed(5) # probably not right value
def move_backward(pose, meters):
    '''
        Starts moving backward
        pose: The pose object produced by the slam_out_pose topic
        meters: the number of meters to move before stopping
    '''
    startingPose = pose
    moving = -1
    movingMeters = meters
    motor.setSpeed(-5) # probably not right value

def rotate_left(pose, degrees)
    '''
        Starts a left/counterclockwise rotation
        pose: The pose object produced by the slam_out_pose topic
        degrees: The number of degrees to turn from the current pose before stopping
    '''
    startingPose = pose
    rotating = -1
    rotatingRadians = np.radians(degrees)
    motor.left.setSpeed(-5) # probably not right value
    motor.right.setSpeed(5)

def rotate_right(pose, degrees)
    '''
        Starts a right/clockwise rotation
        pose: The pose object produced by the slam_out_pose topic
        degrees: The number of degrees to turn from the current pose before stopping
    '''
    startingPose = pose
    rotating = 1
    rotatingRadians = np.radians(degrees)
    motor.left.setSpeed(5) # probably not right value
    motor.right.setSpeed(-5)

def dist(p, oldPos):
    ''' 
        Returns the distance between two pose objects
        p and oldPos are both pose objects
    '''
    return Math.sqrt(Math.pow(p.position.x - oldPos.position.x, 2) + Math.pow(p.position.y - oldPos.position.y, 2) + Math.pow(p.position.z - oldPos.position.z, 2))

def angleBetween(pose1, pose2):
    '''
        Returns the angle between two poses' orientations in radians
        Return value will be in the range [0, pi]
        pose1 and pose2 are both pose objects
        TODO: Test
    '''
    rad1 = 2 * np.arctan(pose1.orientation.z/pose1.orientation.w)
    rad2 = 2 * np.arctan(pose2.orientation.z/pose2.orientation.w)
    return np.minimum(rad1 - rad2, rad2 - rad1) # TODO: Get these to work when on opposite sides of 180
