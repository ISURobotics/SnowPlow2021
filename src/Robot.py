# from turtle import right
# from matplotlib.pyplot import axis
import time

import rclpy
from rclpy.node import Node
# from std_msgs.msg import Int8
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import PointCloud2
# from geometry_msgs.msg import PoseStamped
# from Movement_Threshold import Movement_Threshold
# import ros_numpy
# import matplotlib.pyplot as plt
# import utils
# import numpy as np
from Motor import *
from Sensors import *
import rclpy.topic_endpoint_info
# import Lidar
# from typing import List

class Robot(Node):
    """
        This class controls the motors and keeps track of the sensors module. 
        Passing a robot into a function allows the function to get sensor data and run the speed functions
    """
    def __init__(self, rclpy_init=True):
        rclpy.init()
        super().__init__("Robot")
        self.left = Motor(self, 'left_motor')
        self.right = Motor(self, 'right_motor')
        self.sensors = Sensors(self)
        # self.lidar = Lidar()

    def stop(self):
        self.left.set_speed(0)
        self.right.set_speed(0)

    def set_speed(self, speed):
        self.left.set_speed(speed)
        self.right.set_speed(speed)
        print("speed set")

    def set_speeds(self, leftSpeed, rightSpeed):
        self.left.set_speed(leftSpeed)
        self.right.set_speed(rightSpeed)
        print("speeds set")

    def get_speeds(self):
        """
            Returns (left speed, right speed)
        """
        return (self.left.speed, self.right.speed)

    def wait_for_pub(self):
        print("Waiting for publishers..")
        topics = self.get_topic_names_and_types()
        while not (('/left_motor/speed', ['std_msgs/msg/Int8']) in topics) or not(('/right_motor/speed', ['std_msgs/msg/Int8']) in topics):
            topics = self.get_topic_names_and_types()
            print(topics)
            time.sleep(1)
        print("waiting...")
        time.sleep(2)
        print("Publishers active.")
        return




# This stuff has been copied and edited to work in separate files; keeping the old code commented in case we need it
# class Motor:
#     def __init__(self, path):
#         self._speed_pub = rclpy.Publisher("{}/speed".format(path), Int8, queue_size=5)
#         self.speed = 0

#     def set_speed(self, speed):
#         assert -100 < speed < 100
#         value = Int8()
#         value.data = speed
#         self.speed = speed
#         self._speed_pub.publish(value)
#         print "speed published " + str(value)


# class Lidar:
#     def __init__(self):
#         self.points = []
#         self.pose = None
#         self._sub_points = rclpy.Subscriber("/cloud", PointCloud2, self.callback_pointcloud)
#         self._sub_pose = rclpy.Subscriber("/slam_out_pose", PoseStamped, self.callback_slam_pose)
#         self.thresholds = []
#         self.pose_set = False

#     def callback_pointcloud(self, data):
#         self.points = ros_numpy.point_cloud2.pointcloud2_to_array(data)  # type: List[tuple]
#         # points data is returned as (x, y, color)

#     def callback_slam_pose(self, data):
#         if not self.pose_set:
#             print "Ready"
#             self.pose_set = True
#         self.pose = data.pose
#         for i in range(len(self.thresholds) - 1, -1, -1): # Gotta iterate backwards as stuff might get removed from the list
#             t = self.thresholds[i]
#             measured_val = 0
#             above_thres = False
#             if (t.axis == Movement_Threshold.X_AXIS):
#                 measured_val = self.pose.position.x
#                 #print t.value
#                 #print measured_val
#                 above_thres = (measured_val >= t.value)
#             elif (t.axis == Movement_Threshold.Y_AXIS):
#                 measured_val = self.pose.position.y
#                 above_thres = (measured_val >= t.value)
#             elif (t.axis == Movement_Threshold.Z_ROTATION):
#                 eulers = utils.quaternion_to_euler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
#                 measured_val = eulers[2] # z rotation or yaw
#                 # NEEDS TESTING. LOTS OF TESTING.
#                 print "measured: " + str(measured_val)
#                 if t.trigger_when_above:
#                     if (t.value > np.pi / 2):
#                         above_thres = (measured_val >= t.value) or (measured_val < t.value - 3 * np.pi / 2) # target is close to 180 degrees (pi radians)
#                     else:
#                         above_thres = (measured_val >= t.value) and (measured_val < t.value + np.pi / 4)
#                 else:
#                     if (t.value < np.pi / 2):
#                         above_thres = (measured_val > t.value) and (measured_val <= t.value + 3 * np.pi / 2)
#                     else:
#                         above_thres = (measured_val > t.value) or (measured_val <= t.value - np.pi / 4)
#                 #print above_thres
#                 #print t.trigger_when_above
#                 #print ""
#             if above_thres == t.trigger_when_above:
#                 self.thresholds.pop(i)
#                 t.function() # run the lamba associated with the threshold

#     def get_points(self):
#         return self.points  # points data is returned as (x, y, color)

#     def get_pose(self):
#         return self.pose

#     def wait_for_pose_set(self):
# 	print "Waiting for lidar data..."
#         while not self.pose_set:
#             time.sleep(1)
# 	print "Lidar data received."
#         return

#     def plot_points(self):
#         points = self.points
#         x_pts = [pt[0] for pt in points]
#         y_pts = [pt[1] for pt in points]
#         col = [pt[3] for pt in points]

#         plt.figure()
#         plt.scatter(x_pts, y_pts, c=col)
#         # plt.axes([0, 10, 0, 10])
#         plt.ylim(-15, 15)
#         plt.xlim(0, 15)
#         # plt.axes(xlim=(-5, 5), ylim=(0, 3.5))
#         plt.show()

#     def prepare_obstacle_points(self):
#         #We only care about obstacles in the range of: (may need to be changed)
#         #Y: .2 < y < 4.2
#         #X: -4 < x < 6
#         #To convert: 
#         #Multiply values by 4
#         #Subtract y value from 28
#         #Add 26 to x value

#         #Get points to prepare
#         point_list = self.points
#         #FIlter points to only use those in the range of the field
#         new_list = []
#         for pt in point_list:
#             if 4.0 > pt[0] > 1.1 and 7.25 > pt[1] > -7.75:
#             # if 4.9 > pt[0] > .9 and 7.25 > pt[1] > 0:
#                 new_list.append(pt)
# 		print pt
#         #Convert points to correspond with the pathfinding grid
#         final_list = []
#         for pt in new_list:
#             final_pt = []
#             final_pt.append(int(round(20 - (pt[0] * 4)))) # 20 because the robot lidar starts 2 meters from the bottom of the possible area
#             final_pt.append(int(round(26 + (pt[1] * (-4)))))
#             final_list.append(final_pt)
#         #Return list of points
#         return final_list

#     def prepare_movement_points(self, points_list):
#         #Reverse from prepare obstacle points
#         final_list = []
#         for pt in points_list:
#             final_pt = []
#             final_pt.append((pt[0] - 20) / 4.0)  # robot starts by looking in the negative x direction
#             final_pt.append((pt[1] - 26) / 4.0)  # left of robot is negative y direction
#             final_list.append(final_pt)
#         return final_list

#     def add_listener(self, threshold):
#         self.thresholds.append(threshold)

#     def remove_listeners(self, tag):
#         """
#             Removes all listeners with the given tag
#         """
#         for i in range(len(self.thresholds) - 1, -1, -1):
#             if self.thresholds[i].tag == tag:
#                 self.thresholds.pop(i)

# class IMU:
#     def __init__(self):
#         print("IMU sub activating...")
#         self._sub_euler = rclpy.Subscriber("/imu_euler", Float64MultiArray, self.callback_euler)
#         self.euler = [0, 0, 0]
#         self.thresholds = []
        
#         print("IMU sub active")

#     def callback_euler(self, data):
#         print "new imu data"
#         self.euler = [data.data[0] * np.pi / 180, data.data[1] * np.pi / 180, data.data[2] * np.pi / 180]
#         for i in range(len(self.thresholds) - 1, -1, -1): # Gotta iterate backwards as stuff might get removed from the list
#             t = self.thresholds[i]
#             measured_val = 0
#             above_thres = False
#             if (t.axis == Movement_Threshold.Z_ROTATION):
#                 eulers = self.euler
#                 measured_val = eulers[0] # z rotation or yaw
#                 # NEEDS TESTING. LOTS OF TESTING.
#                 print "measured: " + str(measured_val)
#                 if t.trigger_when_above:
#                     if (t.value > np.pi / 2):
#                         above_thres = (measured_val >= t.value) or (measured_val < t.value - 3 * np.pi / 2) # target is close to 180 degrees (pi radians)
#                     else:
#                         above_thres = (measured_val >= t.value) and (measured_val < t.value + np.pi / 4)
#                 else:
#                     if (t.value < np.pi / 2):
#                         above_thres = (measured_val > t.value) and (measured_val <= t.value + 3 * np.pi / 2)
#                     else:
#                         above_thres = (measured_val > t.value) or (measured_val <= t.value - np.pi / 4)
#                 #print above_thres
#                 #print t.trigger_when_above
#                 #print ""
#             if above_thres == t.trigger_when_above:
#                 self.thresholds.pop(i)
#                 t.function() # run the lamba associated with the threshold
#         #print(data.data[0], data.data[1], data.data[2])

#     def get_euler(self):
#         return self.euler

#     def add_listener(self, threshold):
#         self.thresholds.append(threshold)

#     def remove_listeners(self, tag):
#         """
#             Removes all listeners with the given tag
#         """
#         for i in range(len(self.thresholds) - 1, -1, -1):
#             if self.thresholds[i].tag == tag:
#                 self.thresholds.pop(i)
