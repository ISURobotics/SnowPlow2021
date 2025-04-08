# Ryan Madigan
#

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from ublox_gps import UbloxGps
import serial
import time
import math



class GPS(Node):

    def __init__(self):
        super().__init__("gps_node")
        self.port = serial.Serial('/dev/ttyACM1', baudrate=9600, timeout=1)
        self.gps = UbloxGps(self.port)
        self.set_output_rate(10)
        self.origin = [0, 0]
        self.x = 0.0
        self.y = 0.0
        self.calibrated = False
        self.imu_orientation = 0.0
        self.cal_orientations = []
        self.starting_orientation = 0.0
        self.imu_sub = self.create_subscription(Float32MultiArray, '/imu/magnetometer', self.imu_callback, 10)
        self.gps_pose_pub = self.create_publisher(PoseStamped, '/gps_pose', 10)
        pub_rate = 20
        self.timer = self.create_timer(1/pub_rate, self.publish_pose)







    

    def set_origin(self):
        global_x, global_y = self.get_global_x_y()
        self.origin = [global_x, global_y]





    def get_lat_lon(self):
        coords = self.gps.geo_coords()
        if (coords is None):
            return 0.0, 0.0
        return coords.lat, coords.lon





    def set_output_rate(self, rate: int):

        assert rate >= 1 and rate <= 20
        rate_ms = int(1000 / rate)

        msg = bytearray([0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (rate_ms & 0xFF),
                        (rate_ms >> 8), 0x01, 0x00, 0x00, 0x00])
        ck_a = 0
        ck_b = 0
        for byte in msg[2:]:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        msg.extend([ck_a, ck_b])
        self.port.write(msg)






    def get_global_x_y(self):
        lat, lon = self.get_lat_lon()

        lat_factor = 111132.92 - 559.82*math.cos(2*math.radians(lat)) + 1.175*math.cos(4*math.radians(lat)) - 0.0023*math.cos(6*math.radians(lat))
        lon_factor = 111412*math.cos(math.radians(lat)) - 93.5*math.cos(3*math.radians(lat)) + 0.118*math.cos(5*math.radians(lat))

        global_y = lat_factor * lat
        global_x = lon_factor * lon

        return global_x, global_y








    def get_local_x_y(self):
        global_x, global_y = self.get_global_x_y()
        local_x = global_x - self.origin[0]
        local_y = global_y - self.origin[1]
        local_x, local_y = self.rotate_axis(local_x, local_y, self.starting_orientation)

        return global_x - self.origin[0], global_y - self.origin[1]



    def rotate_axis(self, x, y, theta):
        x_prime = x*math.cos(theta) - y*math.sin(theta)
        y_prime = x*math.sin(theta) + y*math.cos(theta)
        return x_prime, y_prime




    def close_port(self):
        self.port.close()




    def imu_callback(self, msg):
        self.imu_orientation = self.angle_conversion(msg.data[2])
        if (msg.data[0] == 0.0):
            self.calibrated = True



    def create_pose(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        x, y = self.get_local_x_y()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = self.imu_orientation
        msg.pose.orientation.w = 0.0
        
        return msg





    def publish_pose(self):
        msg = self.create_pose()
        self.gps_pose_pub.publish(msg)

    
    def angle_conversion(self, angle):
        return ((-angle + 360)%360)



    def calibrate_orientation(self):
        if (len(self.cal_orientations) > 20):
            self.starting_orientation = (sum(self.cal_orientations) / len(self.cal_orientations))
            self.cal_orientations = []
            return True
        else:
            if (not self.calibrated):
                print("Calibrating Orientation...")
                time.sleep(0.5)
            else:
                self.cal_orientations.append(self.imu_orientation)
            
            return False





    def main(self):
        """start_time = time.time()
        try:
            while True:
                try:
                    lat, lon = self.get_lat_lon()
                    print(lat, lon)
                    if (time.time() - start_time > 10):
                        break;
                except (ValueError, IOError) as err:
                    print(err)

            time.sleep(2)
            self.change_output_rate(5)

            while True:
                try:
                    lat, lon = self.get_lat_lon()
                    print(lat, lon)
                except (ValueError, IOError) as err:
                    print(err)

        finally:
            self.close_port()
        """
        for i in range(5):
            print(self.get_local_x_y())
        self.set_output_rate(10)
        input("Press enter when ready to set origin")
        self.set_origin()
        while True:
            print(self.get_local_x_y())


def main(args=None):
    rclpy.init(args=args)
    gps = GPS()
    while(not gps.calibrate_orientation()):
        rclpy.spin_once(gps)
    gps.set_output_rate(10)
    input("Press enter to set origin")
    gps.set_origin()
    rclpy.spin(gps)
    gps.destroy_node()
    rclpy.shutdown()



if __name__=='__main__':
    main()
