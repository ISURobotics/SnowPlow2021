import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class IMUNode(Node):
    """
        This class is a ROS node that handles communications with the IMU Arduino.
        After initializing it, run the serial_loop function to make it read data from the Arduino
        and publish it to the ROS topics /imu/magnetometer and /imu/euler_rotation.
    """
    def __init__(self):
        super().__init__("imu_node")
        self.imu_orientation_euler = [0, 0, 0]
        self.imu_magnetometer = [0, 0, 0]
        self.magnetometer_pub = self.create_publisher(Float32MultiArray, "/imu/magnetometer", 10)
        self.orientation_pub = self.create_publisher(Float32MultiArray, "/imu/euler_rotation", 10)
        self.ser = serial.Serial('/dev/ttyACM1', 115200) # May need to update
        self.working_data = ""
        self.data_started = False

    def serial_loop(self):
        while True:
            newest_byte = self.ser.read(1)
            if (newest_byte is None or newest_byte == b"\r"):
                continue
            if (newest_byte == b"\n"):
                if not self.data_started:
                    self.data_started = True
                    continue
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

if __name__ == "__main__":
    rclpy.init()
    node = IMUNode()
    node.serial_loop()