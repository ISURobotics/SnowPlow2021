from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class IMU:
    """
        This class handles the IMU subscription on the Robot node.
        It subscribes to the /imu/euler_rotation topic (Arduino_Node.py publishes to it)
        The Sensors class automatically creates one of these when instantiated
    """
    def __init__(self, sensors, node: Node):
        self.sensors = sensors
        print ("IMU sub activating...")
        self._sub_euler = node.create_subscription(Float32MultiArray, "/imu/euler_rotation", self.callback_euler, 10)
        # self._sub_euler = rclpy.Subscriber("/imu_euler", Float64MultiArray, self.callback_euler)
        self.euler = (0, 0, 0)
        
        print("IMU sub active")

    def callback_euler(self, data: Float32MultiArray):
        """
            Callback for the /imu/euler_rotation topic;
            this sets the self.euler variable, which can be accessed
            using the get_euler function
        """
        x_rot = data.data[0] * np.pi / 180
        if x_rot > np.pi:
            x_rot -= 2 * np.pi
        x_rot = -x_rot # Last-week hack from January 2024 according to git blame
        # We don't use the other axes 
        self.euler = (x_rot, data.data[1] * np.pi / 180, data.data[2] * np.pi / 180)
        self.sensors.callback_sensor_data()

    def get_euler(self) -> tuple[float]:
        """
            Returns the [x, y, z] rotation in radians.
            With the way the IMU is normally oriented in the electronics box,
            x is our yaw rotation.

            Called by Sensors.get_euler
        """
        return self.euler
