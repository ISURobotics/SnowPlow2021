from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
# from Sensors import Sensors
# from Movement_Threshold import Movement_Threshold
import ros2_numpy

class Lidar:
    """
        This class handles the Lidar subscription on the Robot node.
        It subscribes to the /cloud topic (sick_scan publishes to it)
        The Sensors class automatically creates one of these when instantiated
    """
    def __init__(self, sensors, node: Node):
        self.sensors = sensors
        self.points: list[tuple] = []
        self.pose = None
        self._sub_points = node.create_subscription(PointCloud2, "/cloud", self.callback_pointcloud, 10)
        self.grid_res = 4     # Must be a multiple of 4
        assert (self.grid_res % 4 == 0)
        # self._sub_pose = node.create_subscription(Odometry, "/odom_rf2o", self.callback_slam_pose, 10)
        # self._sub_pose = rclpy.Subscriber("/slam_out_pose", PoseStamped, self.callback_slam_pose)
        self.points_set = False

    def extract_pose(self, msg_object: Odometry):
        """
            If we try laser odometry without SLAM (unlikely),
            we need to convert the PoseWithCovariance message to just a pose.

            Since we currently use GPS to get our position witout using SLAM or laser odometery, this is unused.
        """
        # ret = PoseStamped()
        # ret.pose.position = msg_object.pose.position
        # ret.pose.orientation = msg_object.pose.pose.orientation
        return msg_object.pose

    def callback_pointcloud(self, data: PointCloud2):
        """
            Callback function for the /pointcloud topic;
            it stores the data in the self.points list.
            To get the points, run get_points
        """

        self.points = ros2_numpy.point_cloud2.point_cloud2_to_array(data)
        # points data is returned as (x, y, color)
        self.points_set = True

    # def callback_slam_pose(self, data):
    #     if not self.points_set:
    #         print("Ready")
    #         self.points_set = True
    #     self.pose = self.extract_pose(data.pose)
    #     self.sensors.callback_sensor_data()

    def get_points(self) -> list[tuple]:
        """
            Gets the most recent available points, for spotting obstacles
        """
        return self.points  # points data is returned as (x, y, color)

    # def get_pose(self) -> PoseStamped:
    #     """
    #         Returns the estimated pose based on the SLAM algorithm
    #         Currently, we don't use SLAM and the library we used for it (Hector SLAM)
    #         is not available in ROS 2, so this is unused and nonfunctional
    #     """
    #     return self.pose

    # def wait_for_pose_set(self):
    #     """
    #         Stops execution until the first SLAM data message is received.
    #         Currently nonfunctional as we don't use SLAM right now.
    #         Also, I think time.sleep prevents us from receiving subscriptions in ROS 2
    #     """
    #     print ("Waiting for SLAM data...")
    #     while not self.pose_set:
    #         time.sleep(1)
    #     print ("SLAM data received.")
    #     return

    def prepare_obstacle_points(self) -> list[tuple]:
        """
            Converts the self.points array into a list of obsacle points.
            This IS used in normal runs; Sensors.get_obstacle_points calls it.
        """
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

    def prepare_movement_points(self, points_list: list[tuple]) -> list[tuple]:
        """
            This takes a list of points on a grid from Path_Finder.path_generator 
            and converts the distances into meters.
            Points_list is a list of (x, y) tuples representing the grid spaces we want to visit

            To be honest, I don't think this function belongs in this file...
        """
        final_list = []
        for pt in points_list:
            final_pt = []
            final_pt.append((pt[0] - (5*self.grid_res)) / self.grid_res)  # robot starts by looking in the negative x direction
            final_pt.append((pt[1] - (6.5*self.grid_res)) / self.grid_res)  # left of robot is negative y direction
            final_list.append(final_pt)
        return final_list
