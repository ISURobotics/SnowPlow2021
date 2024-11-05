import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class Test(Node):

    def __init__(self):
        super().__init__("test_node")
        self.pub = self.create_publisher(PoseStamped, '/test', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.callback)


    def callback(self):
        msg = PoseStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 3.0

        msg.pose.orientation.x = 4.0
        msg.pose.orientation.y = 5.0
        msg.pose.orientation.z = 6.0
        msg.pose.orientation.w = 7.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    t = Test()
    rclpy.spin(t)
    t.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
