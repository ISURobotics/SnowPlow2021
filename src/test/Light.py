import rospy
from std_msgs.msg import String

# Note: Before running this file, be sure to start roscore and rosrun the rosserial_python
# >> rosrun rosserial_python serial_node.py
from typing import List


class Light:
    def __init__(self, path):
        self._light_pub = rospy.Publisher("light", String, queue_size=1)
        self.light = 0

    def set_light(self, light):
        assert -1 < light < 3
        value = String()
        value.data = str(light)
        self.light = light
        self._light_pub.publish(value)
