import rospy
from std_msgs.msg import Float64MultiArray
# This, like any other scripts that will go here, is not used by the robot itself, 
# but please don't delete it as we need it to check if the arduino and IMU are functional with the bare minimum of overhead
def callback_euler(data):
    print "received data!"
    print data

def main():
    rospy.init_node('Robot', anonymous=False)
    sub_euler = rospy.Subscriber("/imu_euler", Float64MultiArray, callback_euler)
    print "IMU subscriber active (This does NOT mean the IMU is publishing data unless we start detecting some)"
    print "This script will start printing the data if we receive any"
    print "Press ctrl-C to exit"
    rospy.spin()
    while not rospy.is_shutdown():
        pass