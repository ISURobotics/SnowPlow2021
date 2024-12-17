import rclpy
from std_msgs.msg import Float32MultiArray
import numpy as np

class IMU:
    def __init__(self, sensors, node):
        self.sensors = sensors
        print("IMU sub activating...")
        self._sub_euler = node.create_subscription(Float32MultiArray, "/imu/euler_rotation", self.callback_euler, 10)
        # self._sub_euler = rclpy.Subscriber("/imu_euler", Float64MultiArray, self.callback_euler)
        self.euler = [0, 0, 0]
        
        print("IMU sub active")

    def callback_euler(self, data):
        x_rot = data.data[0] * np.pi / 180
        if x_rot > np.pi:
            x_rot -= 2 * np.pi
        x_rot = -x_rot # Last-week hack
        # We don't use the other axes 
        self.euler = [x_rot, data.data[1] * np.pi / 180, data.data[2] * np.pi / 180]
        self.sensors.callback_sensor_data()

    def get_euler(self):
        return self.euler
