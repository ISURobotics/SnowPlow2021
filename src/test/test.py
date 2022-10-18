#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def testingLight():
    pub = rospy.Publisher('/light', Int32, queue_size=10)
    rospy.init_node('py_node', anonymous=False)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        val = input("Enter value: ")
        var = int(val)
        pub.publish(var)
        rospy.loginfo(var)
        rate.sleep()

if __name__ == '__main__':
    try:
        testingLight()
    except rospy.ROSInterruptException:
        pass
