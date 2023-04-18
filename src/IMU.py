import rospy
from std_msgs.msg import Float32MultiArray

class IMU:
    """
        Gets acceleration, gyroscope, and magnetometer data from the Inertial Measurement Unit.
        Also does any necessary inertial navigation calculations.
    """
    def __init__(self, sensors):
        self.sensors = sensors
        self.points = []
        self._sub_euler = rospy.Subscriber("/imu/euler", Float32MultiArray, self.callback_imu_euler)
        self._sub_accel = rospy.Subscriber("/imu/accel", Float32MultiArray, self.callback_imu_accel)

    def callback_imu_euler(self, data):
        self.points = data.data

    def callback_imu_accel(self, data):
        self.points = data.data
