import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Float64MultiArray
class SerialToROSCommunication(Node):

    def __init__(self):
        super().__init__("node_test")
        self.counter_ = 0
        self.left_speed=0
        self.right_speed=0
        self.left_subscriber = self.create_subscription(Int8, "/left_motor/speed", self.left_callback, 10)
        self.right_subscriber = self.create_subscription(Int8, "/right_motor/speed", self.right_callback, 10)
        self.imu_publisher=self.create_publisher(Float64MultiArray,"/imu_euler",10)
        self.timer = self.create_timer(1/10, self.timer_callback)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
    #called when ROS updates the topic for the left motor
    def left_callback(self, val):
        self.left_speed=val
        self.update_serial()
    #called when ROS updates the topic for the right motor
    def right_callback(self, val):
        self.right_speed=val
        self.update_serial()
    def timer_callback(self):
        in_=self.ser.readLine().decode('utf-8').strip().split()
        data=list()
        for i in range(len(data)):
            data.append(float(in_[i]))
        self.imu_publisher.publish(data)
    #takes the stored values for the right and left motor speeds and outputs that to the Serial
    def update_serial(self):
        self.ser.write((str(self.left_speed)+"|"+str(self.right_speed)).encode('utf-8'))
        print(str(self.left_speed)+"|"+str(self.right_speed))
def main(args=None):
    rclpy.init(args=args)
    node = SerialToROSCommunication()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()