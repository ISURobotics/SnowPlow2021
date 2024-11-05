import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class IMUNode(Node):
    def __init__(self):
        self.imu_orientation_euler = [0, 0, 0]
        self.orientation_pub = self.create_publisher(Float32MultiArray, "/imu/euler_rotation", 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200) # May need to update

    def serial_loop():
        while True:
             = self.ser.read()
