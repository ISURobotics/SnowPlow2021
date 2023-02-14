import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
# from Movement_Threshold import Movement_Threshold
import ros_numpy

class Lidar:
    def __init__(self, sensors):
        self.sensors = sensors
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
        if not self.pose_set:
            print "Ready"
            self.pose_set = True
        self.pose = data.pose
        self.sensors.lidar_pose(self.pose)
        # for i in range(len(self.thresholds) - 1, -1, -1): # Gotta iterate backwards as stuff might get removed from the list
        #     t = self.thresholds[i]
        #     measured_val = 0
        #     above_thres = False
        #     if (t.axis == Movement_Threshold.X_AXIS):
        #         measured_val = self.pose.position.x
        #         #print t.value
        #         #print measured_val
        #         above_thres = (measured_val >= t.value)
        #     elif (t.axis == Movement_Threshold.Y_AXIS):
        #         measured_val = self.pose.position.y
        #         above_thres = (measured_val >= t.value)
        #     elif (t.axis == Movement_Threshold.Z_ROTATION):
        #         eulers = utils.quaternion_to_euler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
        #         measured_val = eulers[2] # z rotation or yaw
        #         # NEEDS TESTING. LOTS OF TESTING.
        #         print "measured: " + str(measured_val)
        #         if t.trigger_when_above:
        #             if (t.value > np.pi / 2):
        #                 above_thres = (measured_val >= t.value) or (measured_val < t.value - 3 * np.pi / 2) # target is close to 180 degrees (pi radians)
        #             else:
        #                 above_thres = (measured_val >= t.value) and (measured_val < t.value + np.pi / 4)
        #         else:
        #             if (t.value < np.pi / 2):
        #                 above_thres = (measured_val > t.value) and (measured_val <= t.value + 3 * np.pi / 2)
        #             else:
        #                 above_thres = (measured_val > t.value) or (measured_val <= t.value - np.pi / 4)
        #         #print above_thres
        #         #print t.trigger_when_above
        #         #print ""
        #     if above_thres == t.trigger_when_above:
        #         self.thresholds.pop(i)
        #         t.function() # run the lamba associated with the threshold

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

    # def add_listener(self, threshold):
    #     self.thresholds.append(threshold)

    # def remove_listeners(self, tag):
    #     """
    #         Removes all listeners with the given tag
    #     """
    #     for i in range(len(self.thresholds) - 1, -1, -1):
    #         if self.thresholds[i].tag == tag:
    #             self.thresholds.pop(i)