from std_msgs.msg import Int8
from rclpy.node import Node

class Motor:
    """
        This handles publishing to the motors from the Robot node.
        Two of these objects are created; they publish to the 
        left/speed and right/speed topics. Use the set_speed command 
        to send data over the topic
    """
    def __init__(self, node: Node, path: str):
        self._speed_pub = node.create_publisher(Int8, "{}/speed".format(path), 10)
        self.speed = 0

    def set_speed(self, speed: float):
        """
            Sets this motor to run at a certain speed.
            The speed can be anywhere from -100 to 100; 
            negative values make the motor spin backwards.
        """
        assert -100 < speed < 100
        value = Int8()
        value.data = int(speed)
        self.speed = int(speed)
        self._speed_pub.publish(value)
        print ("speed published " + str(value))
