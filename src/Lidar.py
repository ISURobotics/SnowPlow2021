import rclpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import utils
# from Movement_Threshold import Movement_Threshold
import ros2_numpy

class Lidar:
    def __init__(self, sensors, node):
        self.sensors = sensors
        self.points = []
        self.pose = None
        self._sub_points = node.create_subscription(PointCloud2, "/cloud", self.callback_pointcloud, 10)
        self.grid_res = 100     # Must be a multiple of 4
        assert (self.grid_res % 4 == 0)
        # self._sub_pose = node.create_subscription(Odometry, "/odom_rf2o", self.callback_slam_pose, 10)
        # self._sub_pose = rclpy.Subscriber("/slam_out_pose", PoseStamped, self.callback_slam_pose)
        self.points_set = False

    def extract_pose(self, msg_object: Odometry):
        '''
            If we just use laser odometry without SLAM (unlikely),
            we need to convert the PoseWithCovariance message to just a pose
        '''
        # ret = PoseStamped()
        # ret.pose.position = msg_object.pose.position
        # ret.pose.orientation = msg_object.pose.pose.orientation
        return msg_object.pose

    def callback_pointcloud(self, data):
        # print(ros2_numpy.point_cloud2.point_cloud2_to_array(data))
        self.points = ros2_numpy.point_cloud2.point_cloud2_to_array(data)  # type: List[tuple]
        # points data is returned as (x, y, color)
        self.points_set = True

    # def callback_slam_pose(self, data):
    #     if not self.points_set:
    #         print("Ready")
    #         self.points_set = True
    #     self.pose = self.extract_pose(data.pose)
    #     self.sensors.callback_sensor_data()

    def get_points(self):
        return self.points  # points data is returned as (x, y, color)

    def get_pose(self):
        return self.pose

    def wait_for_pose_set(self, node):
        print("Waiting for Lidar data...")
        while not self.points_set:
            rclpy.spin_once(node)
        print("Lidar data received.")
        return

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

    def prepare_obstacle_points(self):
        #We only care about obstacles in the range of: (may need to be changed)
        #Y: .2 < y < 4.2
        #X: -4 < x < 6
        #To convert: 
        #Multiply values by 4
        #Subtract y value from 28
        #Add 26 to x value

        #Get points to prepare
        point_list = self.points['xyz']
        #FIlter points to only use those in the range of the field
        new_list = []
        for pt in point_list:
            # print(pt)
            if 4.9 > pt[0] > .9 and 7.25 > pt[1] > -7.75:
            # if 4.9 > pt[0] > .9 and 7.25 > pt[1] > 0:
                new_list.append(pt)
            print(pt)
        #Convert points to correspond with the pathfinding grid
        final_list = []
        for pt in new_list:
            final_pt = []
            final_pt.append(int(round((5*self.grid_res) - (pt[0] * self.grid_res)))) # 20 because the robot lidar starts 2 meters from the bottom of the possible area
            final_pt.append(int(round((6.5*self.grid_res) + (pt[1] * (-(self.grid_res))))))
            final_list.append(final_pt)
        #Return list of points
        return final_list

    def prepare_movement_points(self, points_list):
        #Reverse from prepare obstacle points
        final_list = []
        for pt in points_list:
            final_pt = []
            final_pt.append((pt[0] - (5*self.grid_res)) / self.grid_res)  # robot starts by looking in the negative x direction
            final_pt.append((pt[1] - (6.5*self.grid_res)) / self.grid_res)  # left of robot is negative y direction
            final_list.append(final_pt)
        return final_list
