import rospy
from std_msgs.msg import Float64MultiArray

def callback_euler(data):
    print "received data!"
    print data

def main():
    sub_euler = rospy.Subscriber("/imu_euler", Float64MultiArray, callback_euler)
    print("IMU subscriber active (This does NOT mean the IMU is publishing data unless we start detecting some)")
    rospy.spin()
    while not rospy.is_shutdown():
        pass