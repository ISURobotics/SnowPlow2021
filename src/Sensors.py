import Lidar



class Sensors:

    def __init__(self):
        self.lidar = Lidar(self)

    def init_lidar(self):
        self.lidar.wait_for_pose_set()
    
    def callback_lidar_pose(self, pose):
        pass

    def get_lidar_pose(self):
        return self.lidar.get_pose()