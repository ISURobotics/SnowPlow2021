from Robot import *
from Robot_Mover import RobotMover
from Path_Executor import *

def main():
    """
    This function will initialize the robot and lider,
    create the object map with lidar data,
    and begin motion of the robot
    """
    lidar = Lidar()
    r = Robot()
    rm = RobotMover(r)
    pe = Path_Executor(rm, lidar)
    print "Press enter to continue"
    x = raw_input()
    pe.apply_next_action()
    # exit(0)
    rospy.spin()
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()