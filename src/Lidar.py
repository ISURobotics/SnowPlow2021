import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import time
# from Movement_Threshold import Movement_Threshold
import ros_numpy

class Lidar:
    def __init__(self, sensors):
        self.sensors = sensors
        self.points = []
        self.pose = None
        self._sub_points = rospy.Subscriber("/cloud", PointCloud2, self.callback_pointcloud)
        self._sub_pose = rospy.Subscriber("/slam_out_pose", PoseStamped, self.callback_slam_pose)
        self.pose_set = False

    def callback_pointcloud(self, data):
        self.points = ros_numpy.point_cloud2.pointcloud2_to_array(data)  # type: List[tuple]
        # points data is returned as (x, y, color)

    def callback_slam_pose(self, data):
        if not self.pose_set:
            print "Ready"
            self.pose_set = True
        self.pose = data.pose
        self.sensors.callback_lidar_pose(self.pose)

    def get_points(self):
        return self.points  # points data is returned as (x, y, color)

    def get_pose(self):
        return self.pose

    def wait_for_pose_set(self):
        print "Waiting for lidar data..."
        while not self.pose_set:
            time.sleep(1)
        print "Lidar data received."
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
        point_list = self.points
        #FIlter points to only use those in the range of the field
        new_list = []
        for pt in point_list:
            if 4.9 > pt[0] > .9 and 7.25 > pt[1] > -7.75:
            # if 4.9 > pt[0] > .9 and 7.25 > pt[1] > 0:
                new_list.append(pt)
        print pt
        #Convert points to correspond with the pathfinding grid
        final_list = []
        for pt in new_list:
            final_pt = []
            final_pt.append(int(round(20 - (pt[0] * 4)))) # 20 because the robot lidar starts 2 meters from the bottom of the possible area
            final_pt.append(int(round(26 + (pt[1] * (-4)))))
            final_list.append(final_pt)
        #Return list of points
        return final_list

    def prepare_movement_points(self, points_list):
        #Reverse from prepare obstacle points
        final_list = []
        for pt in points_list:
            final_pt = []
            final_pt.append((pt[0] - 20) / 4.0)  # robot starts by looking in the negative x direction
            final_pt.append((pt[1] - 26) / 4.0)  # left of robot is negative y direction
            final_list.append(final_pt)
        return final_list