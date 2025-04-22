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




# One problem that is often had is making sure the serial port is correct. This can be changed in the constructor of the GPS class on the "self.port"
# line. The port should only really change by its last number, and should really only ever be 0, 1, or 2 (ex. '/dev/ttyACM1')




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
        # Reset arduinos and run Serial_Detect.py first to set the values in this file. This file puts the port names of various devices into a file
        with open("serial_ports.txt",'r') as f:
            _,_,port_name=f.readline().split(" ")
        self.port = serial.Serial(port_name, baudrate=9600, timeout=1)
        self.gps = UbloxGps(self.port)
        self.set_output_rate(10)
        self.origin = [0, 0]
        self.x = 0.0
        self.y = 0.0
        self.calibrated = False
        self.imu_orientation = 0.0
        self.cal_orientations = []
        self.starting_orientation = 0.0
        self.lat_factor = 0.0                                                                                   # when multiplied by the latitude, gives the position in meters relative to 0, 0 on the latitude longitude scale (somewhere in Africa)
        self.lon_factor = 0.0                                                                                   # # when multiplied by the longitude, gives the position in meters relative to 0, 0 on the latitude longitude scale (somewhere in Africa)
        self.imu_sub = self.create_subscription(Float32MultiArray, '/imu/magnetometer', self.imu_callback, 10)  # initializes subscription to imu's data
        self.gps_pose_pub = self.create_publisher(PoseStamped, '/gps_pose', 10)                                 # initialized publisher for GPS's data
        pub_rate = 20                                                                                           # how fast the GPS data is published
        self.timer = self.create_timer(1/pub_rate, self.publish_pose)                                           # initialized timer to publish at pub_rate 







    
    # when the robot is where you want the (0, 0) position to be, this method is called in order to help create a local frame of reference later on
    def set_origin(self):
        global_x, global_y = self.get_global_x_y(True)
        self.origin = [global_x, global_y]




    # @params  setting_origin - True if you are wanting this call to set your current location as the origin
    #
    # Gets and returns the raw latitude and longitude data from the GPS
    # Additionally sets the lattitude and longitude factor if necessary(explained in constuctor)
    def get_lat_lon(self, setting_origin = False):
        coords = self.gps.geo_coords()      # gets the raw latitude and longitude data
        if (setting_origin):
            # these latitude and longitude factors are equations based off of this wikipedia article (https://en.wikipedia.org/wiki/Geographic_coordinate_system)
            self.lat_factor = 111132.92 - 559.82*math.cos(2*math.radians(coords.lat)) + 1.175*math.cos(4*math.radians(coords.lat)) - 0.0023*math.cos(6*math.radians(coords.lat))
            self.lon_factor = 111412*math.cos(math.radians(coords.lat)) - 93.5*math.cos(3*math.radians(coords.lat)) + 0.118*math.cos(5*math.radians(coords.lat))
        if (coords is None):
            return 0.0, 0.0
        return coords.lat, coords.lon




    # @ param  rate - the rate at which you wish the GPS to output data in Hz
    #
    # Sets how fast the GPS module will send data in terms of Hz
    # MUST BE BETWEEN 1 AND 20 HZ
    def set_output_rate(self, rate: int):

        assert rate >= 1 and rate <= 20
        rate_ms = int(1000 / rate)

        # message associated with changing the rate on the GPS
        msg = bytearray([0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (rate_ms & 0xFF),
                        (rate_ms >> 8), 0x01, 0x00, 0x00, 0x00])
        ck_a = 0    # check byte a
        ck_b = 0    # check byte b
        # calculates the check bytes
        for byte in msg[2:]:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        msg.extend([ck_a, ck_b])    # adds check bytes to the message
        self.port.write(msg)        # sends the message to the GPS module





    # @param  setting_origin - True if you are wanting this call to set your current location as the origin
    #
    # Gets and returns the global x and y position relative to the (0, 0) point on the latitude longitude scale (somewhere near Africa)
    def get_global_x_y(self, setting_origin = False):
        lat, lon = self.get_lat_lon(setting_origin)

        

        global_y = self.lat_factor * lat    # converts latitude degrees to meters
        global_x = self.lon_factor * lon    # converts longitude degrees to meters

        return global_x, global_y







    # Gets and returns the local x y positioning which is relative to the origin you set and factors in your starting orientation as well by rotating the x y axes
    # to match how the snowplow was oriented when the origin was set
    def get_local_x_y(self):
        global_x, global_y = self.get_global_x_y()                                                      # gets global positioning in x, y coordinates
        local_x = global_x - self.origin[0]                                                             # translates x positioning to local positioning relative to origin
        local_y = global_y - self.origin[1]                                                             # translates y positioning to local positioning relative to origin
        local_x, local_y = self.rotate_axis(local_x, local_y, math.radians(self.starting_orientation))  # rotates the x and y positioning to match orienation the snowplow was in when the origin was set

        return local_x, local_y


    # @param  x - original x position, with a x y axes where y is due north and x is due east
    # @param  y - original y position, with a x y axes where y is due north and x is due east
    # @param  theta - orientation you wish to rotate the x y axes by in radians with a counterclockwise rotation being positive
    def rotate_axis(self, x, y, theta):
        x_prime = x*math.cos(theta) - y*math.sin(theta) 
        y_prime = x*math.sin(theta) + y*math.cos(theta) 
        return x_prime, y_prime



    # closes serial port for GPS
    def close_port(self):
        self.port.close()



    # Callback function for IMU subscriber which just gets the IMU data
    def imu_callback(self, msg):
        self.imu_orientation = self.angle_conversion(msg.data[2])
        if (msg.data[0] == 0.0):
            self.calibrated = True


    # The message type that the GPS node of ROS will send out is a PoseStamped message, so this method simply arranges all the data in that particular message format
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




    # Publish method for the GPS publisher which publishes the GPS's orientation in a PoseStamped format
    def publish_pose(self):
        msg = self.create_pose()
        self.gps_pose_pub.publish(msg)

    # Converts the angle given by the IMU to the frame of 0 degrees being the snowplow facing directly north and counterclockwise rotation being a positive rotation
    def angle_conversion(self, angle):
        return ((-angle + 360)%360)


    # The IMU needs to be calibrated properly before it can send somewhat accurate compass data. To calibrate it, it needs to be moved around  and rotated around
    # in all directions, usually done by a figure 8 motion.
    # Additionally, since the IMU's compasses data is a little noisy, this function averages 20 data points taken to try and acheive a more accurate reading
    def calibrate_orientation(self):
        # if there have been 20 data points taken, average them and set that to be the starting orientation of the snowplow
        if (len(self.cal_orientations) > 20):
            self.starting_orientation = (sum(self.cal_orientations) / len(self.cal_orientations))
            self.cal_orientations = []
            return True
        # otherwise, if the IMU's magnetometer isn't calibrated yet, wait
        # if it is calibrated, add the latest data point to the array that will be averaged to find starting orienation
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
    rclpy.init(args=args)                       # initialize ROS2
    gps = GPS()                                 # create GPS object
    #while(not gps.calibrate_orientation()):
    #    rclpy.spin_once(gps)
    gps.set_output_rate(10)                     # set output rate of GPS to 10 Hz
    print("Done Calibrating IMU")
    gps.set_origin()                            # set the origin an starting orientation to whereever the Snowplow is and the direction it is facing
    rclpy.spin(gps)                        
    gps.destroy_node()
    rclpy.shutdown()



if __name__=='__main__':
    main()
