import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class DataListener(Node):

    def __init__(self):
        super().__init__("node_test")
        self.counter_ = 0
        self.left_speed=0
        self.right_speed=0
        # create_subscriber needs 3 parameters: Msg Type, topic name, the callback, and queue size buffer
        self.left_subscriber = self.create_subscription(Int8, "left_motor/speed", self.left_callback, 10)
        self.right_subscriber = self.create_subscription(Int8, "right_motor/speed", self.right_callback, 10)
        self.ser = serial.Serial('COM1', 115200)
    def left_callback(self, val):
        left_speed=val
        self.update_serial()
    def right_callback(self, val):
        right_speed=val
        self.update_serial()
    def update_serial(self):
        self.ser.write(str(self.left_speed)+"|"+str(self.right_speed))
def main(args=None):
    rclpy.init(args=args)
    node = DataListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()