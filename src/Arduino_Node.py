import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray
from serial import SerialException
class SerialToROSCommunication(Node):

    def __init__(self):
        super().__init__("imu_node")
        self.counter_ = 0
        self.left_speed=0
        self.right_speed=0
        self.left_subscriber = self.create_subscription(Int8, "/left_motor/speed", self.left_callback, 10)
        self.right_subscriber = self.create_subscription(Int8, "/right_motor/speed", self.right_callback, 10)
        self.imu_orientation_euler = [0, 0, 0]
        self.imu_magnetometer = [0, 0, 0]
        self.magnetometer_pub = self.create_publisher(Float32MultiArray, "/imu/magnetometer", 10)
        self.orientation_pub = self.create_publisher(Float32MultiArray, "/imu/euler_rotation", 10)
        self.timer = self.create_timer(1/10, self.timer_callback)
        self.working_data = ""
        self.data_started = False
        try:
            self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
        except SerialException:
            self.ser=serial.Serial('/dev/ttyACM1',baudrate=115200)
    #called when ROS updates the topic for the left motor
    def left_callback(self, val):
        self.left_speed=val.data
        self.update_serial()
    #called when ROS updates the topic for the right motor
    def right_callback(self, val):
        self.right_speed=val.data
        self.update_serial()
    def timer_callback(self):
        while True:
            newest_byte = self.ser.read(1)
            if (newest_byte is None or newest_byte == b"\r"):
                break
            if (newest_byte == b"\n"):
                if not self.data_started:
                    self.data_started = True
                    break
                print(self.working_data)
                datas = self.working_data.split(";")
                self.imu_orientation_euler = [float(c) for c in datas[0].split(",")]
                self.imu_magnetometer = [float(c) for c in datas[1].split(",")]
                print("publishing")
                orient_msg = Float32MultiArray()
                orient_msg.data = self.imu_orientation_euler
                self.orientation_pub.publish(orient_msg)

                mag_msg = Float32MultiArray()
                mag_msg.data = self.imu_magnetometer
                self.magnetometer_pub.publish(mag_msg)
                self.working_data = ""
            elif self.data_started:
                self.working_data += newest_byte.decode("utf-8")
    #takes the stored values for the right and left motor speeds and outputs that to the Serial
    def update_serial(self):
        self.ser.write((str(self.left_speed)+"|"+str(self.right_speed)+"'\n").encode('utf-8'))
        print(str(self.left_speed)+"|"+str(self.right_speed))
def main(args=None):
    rclpy.init(args=args)
    node = SerialToROSCommunication()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
