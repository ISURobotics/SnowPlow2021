from Robot import *
from Robot_Mover import RobotMover
from Path_Executor import *
from Func_Generator import FuncGenerator
import Lidar

def main():
    """
    This function will initialize the robot and lider,
    create the object map with lidar data,
    and begin motion of the robot
    """
    lidar = Lidar()
    r = Robot()
    print "Press enter to continue"
    x = raw_input()
    while 1:
        lidar.plot_points()
        x = raw_input
        if x == 'q':
            exit(0)
    # exit(0)
    rospy.spin()
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
