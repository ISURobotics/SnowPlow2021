import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
class GPSListener(Node):
    def __init__(self):
        super().__init__("gps_node")
        self.counter_ = 0
        self.lat=0
        self.lon=0
        self.has_init_lat=False
        self.has_init_lon=False
        self.has_init_rot=False
        # create_subscriber needs 3 parameters: Msg Type, topic name, the callback, and queue size buffer
        self.lat_subscriber = self.create_subscription(Float32, "latitude", self.lat_callback, 1)
        self.lon_subscriber = self.create_subscription(Float32, "longitude", self.lon_callback, 1)
        self.rot_subscriber = self.create_subscription(Float32, "rotation", self.rot_callback, 1)
        self.pose_publisher = self.create_publisher(Float32MultiArray, 'cartesian pose', 10)
    def lat_callback(self, val):
        self.has_init_lat=True
        self.lat=val
        self.update_pub(self)
    def lon_callback(self, val):
        self.has_init_lon=True
        self.lon=val
        self.update_pub(self)
    def rot_callback(self, val):
        self.has_init_rot=True
        self.rot=val
        self.update_pub(self)
    def update_pub(self):
        if self.has_init_lat and self.has_init_lon and self.has_init_rot:
            dlon=self.lon-self.org_lon
            dlat=self.lat-self.org_lat
            drot=self.rot-self.drot
            dx=math.sin(dlon)*6378137#radius of the earth in meters. If you are a flat earther this should be infinite. If you are a tourus earther, this does not apply.
            dy=math.sin(dlat)*6378137
            phi=math.atan2(dx,dy)
            d=math.sqrt(dx*dx+dy*dy)
            x=d*math.sin(self.org_rot-phi)
            y=d*math.sin(self.org_rot-phi)
            self.pose_publisher.publish([x,y,drot])
        else:
            self.org_lat=self.lat
            self.org_lon=self.lon
            self.org_rot=self.rot

def main(args=None):
    rclpy.init(args=args)
    node = GPSListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()