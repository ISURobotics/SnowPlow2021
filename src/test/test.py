#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray


'''
Publishes the motor values 


motor values 
'''
def motorPublisher():
    #creates a publisher that publishes to the topic 'motorSpeed' with data type of Int16MultiArray
    pub = rospy.Publisher('/motorSpeed', Int16MultiArray, queue_size=10)
    #creates a node for this code called pynode
    rospy.init_node('py_node', anonymous=False)
    #sets the rate to publish at 
    rate=rospy.Rate(10)
    #array to be published
    arr = Int16MultiArray()
    arr.data = []

    while not rospy.is_shutdown():
        motor1 = input("Enter first value: ")
        motor2 = input("Enter second value: ")
        motor1 = int(val1)
        motor2 = int(val2)

        arr.data = [motor1, motor2]
        pub.publish(arr)
        rospy.loginfo(arr)
        rate.sleep()

if __name__ == '__main__':
    try:
        motorPublisher()
    except rospy.ROSInterruptException:
        pass
