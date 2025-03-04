import time

import rclpy
from rclpy.node import Node
from Motor import *
from Sensors import *
import rclpy.topic_endpoint_info

class Robot(Node):
    """
        This class controls the motors and keeps track of the sensors module. 
        Passing a robot into a function allows the function to get sensor data and run the speed functions
    """
    def __init__(self):
        rclpy.init()
        super().__init__("Robot")
        self.left = Motor(self, 'left_motor')
        self.right = Motor(self, 'right_motor')
        self.sensors = Sensors(self)
        # self.lidar = Lidar()

    def stop(self):
        self.left.set_speed(0)
        self.right.set_speed(0)

    def set_speed(self, speed: float):
        self.left.set_speed(speed)
        self.right.set_speed(speed)
        print ("speed set")

    def set_speeds(self, leftSpeed: float, rightSpeed: float):
        self.left.set_speed(leftSpeed)
        self.right.set_speed(rightSpeed)
        print ("speeds set")

    def get_speeds(self) -> tuple[float]:
        """
            Returns (left speed, right speed)
        """
        return (self.left.speed, self.right.speed)

    def wait_for_pub(self):
        print ("Waiting for publishers..")
        topics = self.get_topic_names_and_types()
        while not (('/left_motor/speed', ['std_msgs/msg/Int8']) in topics) or not(('/right_motor/speed', ['std_msgs/msg/Int8']) in topics):
            topics = self.get_topic_names_and_types()
            print(topics)
            time.sleep(1)
        print ("waiting...")
        time.sleep(2)
        print ("Publishers active.")
        print ("Waiting for subscribers...")
        while self.sensors.gps.gps_pose is None:
            rclpy.spin_once(self)
            print ("waiting...")
        return