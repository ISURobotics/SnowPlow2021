import rclpy
from std_msgs.msg import Int8

class Motor:
    def __init__(self, node, path):
        self._speed_pub = node.create_publisher(Int8, "{}/speed".format(path), 10)
        self.speed = 0

    def set_speed(self, speed):
        assert -100 < speed < 100
        value = Int8()
        value.data = int(speed)
        self.speed = int(speed)
        self._speed_pub.publish(value)
        print("speed published " + str(value))
