#!/usr/bin/env python

import rclpy
from std_msgs.msg import Int8


'''
Publishes the motor values 


motor values 
'''
def motorPublisher():
    #creates a publisher that publishes to the topic 'motorSpeed' with data type of Int16MultiArray
    pub = rclpy.Publisher('/left_motor/speed', Int8, queue_size=10)
    #creates a node for this code called pynode
    rclpy.init_node('py_node', anonymous=False)
    #sets the rate to publish at 
    rate=rclpy.Rate(10)
    #array to be published

    while not rclpy.is_shutdown():
        val1 = input("Enter first value: ")
        #val2 = input("Enter second value: ")
        motor1 = int(val1)
        #motor2 = int(val2)

        pub.publish(val1)
        rclpy.loginfo(val1)
        rate.sleep()

if __name__ == '__main__':
    try:
        motorPublisher()
    except rclpy.ROSInterruptException:
        pass
