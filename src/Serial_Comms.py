import serial
import rclpy
import time
from rclpy.node import Node

from std_msgs.msg import Int8

#Subscribers to both /left_motor/speed and /right_motor/speed


#Callbacks to each motor speed to send a specific command over serial to the arduino
#Command format 0080, then arduino reads it as 0% left motor, 80% right motor.

arduino1Out = serial.__init__(port='COM4', baudrate=115200, timeout= 0.1)
arduino2In = serial.__init__(port='COM5', baudrate=115200, timeout= 0.1)

class SerialComms(Node):

    def __init__(self):
        
        super().__init__('serial_comms')
        LeftMotorSub = Node.create_subscription(Int8, '/left_motor/speed', self.readInLeft_callback, 10) #What does the '10' parameter do?
        RightMotorSub = Node.create_subscription(Int8, '/right_motor/speed', self.readInRight_callback, 10) #What does the '10' parameter do?
        LeftMotorSub
        RightMotorSub
        IMUPub = Node.create_publisher(Float64MultiArray, "/imu_euler", 10)
        IMUPub.timer = IMUPub.create_timer(0.05, IMUPub.timer_callback)
        

    def timer_callback():



    def readInLeft_callback(self, msg):
        leftMotorInputROS = msg.data
        if(leftMotorInputROS > 100):
            leftMotorInputROS = 100
        elif(leftMotorInputROS < -100):
            leftMotorInputROS = -100
        arduino1Out.write(leftMotorInputROS)



    def readInRight_callback(self, msg):
        rightMotorInputROS = msg.data
        if(rightMotorInputROS > 100):
            rightMotorInputROS = 100
        elif(rightMotorInputROS < -100):
            rightMotorInputROS = -100
        arduino1Out.write(rightMotorInputROS)


while True:
    IMUData = arduino2In.readline()
    time.sleep(0.05)
    IMUPub.publish()
    #Read in the data to a variable that can be used / formatted in the timer_callback





#Equivalent ROS stuff from Arduino to implement in here:

# //setting node name
# ros::NodeHandle arduino;

# //input data for motor speeds given by ros
# int leftMotorInputROS = 0;
# int rightMotorInputROS = 0;



# //reads in left motor speed and verifies the value is -100 < x < 100
# void readInLeft(const std_msgs::Int8 &msg) {
#   //assigns data from msg to motor left motor input value
#   leftMotorInputROS = msg.data;

#   //Checks to make sure motor inputs are between bounds of -100 and 100
#   if (leftMotorInputROS > 100)
#     leftMotorInputROS = 100;
#   else if(leftMotorInputROS < -100)
#     leftMotorInputROS = -100;
# }

# //reads in right motor speed and verifies the value is -100 < x < 100
# void readInRight(const std_msgs::Int8 &msg) {
#   //assigns data from msg to right motor input value
#   rightMotorInputROS = msg.data;

#   //Checks to make sure motor inputs are between bounds of -100 and 100
#   if (rightMotorInputROS > 100)
#     rightMotorInputROS = 100;
#   else if(rightMotorInputROS < -100)
#     rightMotorInputROS = -100;
# }

# //creates subscriber object listening to /left_motor/speed topic and uses readInLeft as callback function
# ros::Subscriber<std_msgs::Int8> subLeft("/left_motor/speed", &readInLeft);
# //creates subscriber object listening to /right_motor/speed topic and uses readInRight as callback function
# ros::Subscriber<std_msgs::Int8> subRight("/right_motor/speed", &readInRight);