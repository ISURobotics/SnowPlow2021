import rospy
from std_msgs.msg import Int8

class Motor:
    def __init__(self, path):
        self._speed_pub = rospy.Publisher("{}/speed".format(path), Int8, queue_size=1)
        self.speed = 0

    def set_speed(self, speed):
        assert -100 < speed < 100
        value = Int8()
        value.data = speed
        self.speed = speed
        self._speed_pub.publish(value)
        print("speed published " + str(value))