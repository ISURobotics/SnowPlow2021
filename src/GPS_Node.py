# Ryan Madigan
# This code functions as a ROS2 node that interacts with our GPS RTK system. The GPS RTK system we use and documentation about it can be found
# at this link (https://www.sparkfun.com/sparkfun-gps-rtk-sma-kit.html). The GPS RTK system works similar to a regular GPS system, receiving
# signals from satilites which allow it to determine its position with accuracy being on the scale of meters. However, to get more accurate 
# positioning, the GPS RTK system uses "correction data" to get accuracy down to the centimeter scale. This correction data needs to come from
# another GPS RTK module that is in the area which is called a base station. This station is sedentary and does not move ever. Because it 
# it doesn't move, we know its precise positioning on the earth. And so we can generate correction data by looking at the differences between 
# where the live GPS data says the base station is, and where we know it actually is. All of the math and calculations required to use the
# correction data to get more accurate position data is handled on board the GPS RTK module and doens't need to be worried about by us. Using 
# this generated correction data, any other GPS RTK system that is in the same general area (within ~10km radius) can accurately determine its 
# position. Luckily for us, both Minnesota and Iowa have freely available correction data that can be streamed over the internet. This is how 
# we get our correction data. We use a Raspberry pi that is connected to someone's phone's hotspot, and stream that correction data and pipe 
# it into the GPS RTK module on board the robot. After receiving the correction data, the GPS take a little while to work its magic (can up up to
# a few minutes) and then is ready to give precise positioning over a USB serial wire which is connected directly to the Jetson Nano
#
#
# HOW TO SETUP GPS TO GIVE ACCURATE GPS DATA
# 1. Make sure GPS antenna (black box about 2"x2"x1") with SMA connector is fully connected to GPS
# 2. Setup the correction data machine (Raspberry pi)
#       - Ensure Serial Basic Breakout module (Tiny red board with usb c and 6ish output pins) is plugged into Raspberry Pi via usb c cable
#       - Connect Serial Basic Breakout to GPS RTK 
#           - Searial Basic Pin  | GPS RTK Pin
#                           TX0  | RX2 (One near USB C)
#                           GND  | GND (One near RX2) 
# 3. Turn on GPS by plugging it into Jetson Nano and powering on Jetson Nano
# 4. Wait a few minutes (no exact time, at least 2 minutes though)
# 5. At this point take the GPS outside if it isn't already otherwise the GPS won't be able to see any satelites
# 6. Start the correction data by turning on Raspberry pi, connecting it to wifi or hotspot and starting one of the bash scripts on the home 
# screen by double clicking, and pressing execute in terminal
#   - There is one bash script for Ames, and one for St. Paul/Minneapolis so run the appropriate one for your location
# 7. At this point, the TX light on the Serial Basic Breakout should start blinking or just be lit up, if not, no data is being sent from the Pi
# 8. Also at this point (although it could take a few seconds) the RTK light on the GPS RTK module should go from ON to blinking.
#   - if it doesn't start blinking within a minute and the Serial Basic Breakout TX light is blinking, then just unplug and plug back in the GPS
# 9. After about 30 seconds (although can take up to a few minutes) the RTK light should stop blinking and turn off. At this point, it is sending
# accurate data over serial to the Jetson Nano.
#
#
# SparkFuns Official Hookup Guide (https://learn.sparkfun.com/tutorials/gps-rtk2-hookup-guide)



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
        self.port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
        self.gps = UbloxGps(self.port)
        self.set_output_rate(10)
        self.origin = [0, 0]
        self.x = 0.0
        self.y = 0.0
        self.calibrated = False
        self.imu_orientation = 0.0
        self.cal_orientations = []
        self.starting_orientation = 0.0
        self.lat_factor = 0.0
        self.lon_factor = 0.0
        self.imu_sub = self.create_subscription(Float32MultiArray, '/imu/magnetometer', self.imu_callback, 10)
        self.gps_pose_pub = self.create_publisher(PoseStamped, '/gps_pose', 10)
        pub_rate = 20
        self.timer = self.create_timer(1/pub_rate, self.publish_pose)







    

    def set_origin(self):
        global_x, global_y = self.get_global_x_y(True)
        self.origin = [global_x, global_y]





    def get_lat_lon(self, setting_origin = False):
        coords = self.gps.geo_coords()
        if (setting_origin):
            self.lat_factor = 111132.92 - 559.82*math.cos(2*math.radians(coords.lat)) + 1.175*math.cos(4*math.radians(coords.lat)) - 0.0023*math.cos(6*math.radians(coords.lat))
            self.lon_factor = 111412*math.cos(math.radians(coords.lat)) - 93.5*math.cos(3*math.radians(coords.lat)) + 0.118*math.cos(5*math.radians(coords.lat))
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






    def get_global_x_y(self, setting_origin = False):
        lat, lon = self.get_lat_lon(setting_origin)

        

        global_y = self.lat_factor * lat
        global_x = self.lon_factor * lon

        return global_x, global_y








    def get_local_x_y(self):
        global_x, global_y = self.get_global_x_y()
        local_x = global_x - self.origin[0]
        local_y = global_y - self.origin[1]
        local_x, local_y = self.rotate_axis(local_x, local_y, math.radians(self.starting_orientation))

        return local_x, local_y



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
    #while(not gps.calibrate_orientation()):
    #    rclpy.spin_once(gps)
    gps.set_output_rate(10)
    print("Done Calibrating IMU")
    gps.set_origin()
    rclpy.spin(gps)
    gps.destroy_node()
    rclpy.shutdown()



if __name__=='__main__':
    main()
