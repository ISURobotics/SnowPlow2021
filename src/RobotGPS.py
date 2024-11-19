from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


class RobotGPS:
    """
        The part of the robot node that receives data from the gps node
        Interact with this using the Sensors facade
    """

    def __init__(self, sensors, node):
        self.sensors = sensors
        self._sub_position = node.create_subscription(PoseStamped, "/gps_pose", 10)
        self.gps_pose: Pose = None

    def gps_callback(self, msg: PoseStamped):
        self.gps_pose = msg.pose

    def get_pose(self) -> Pose:
        return self.gps_pose
