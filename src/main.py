from Robot import *
from Robot_Mover import RobotMover
from Path_Executor import *
from Func_Generator import FuncGenerator

def main():
    """
    This function will initialize the robot and lidar,
    create the object map with lidar data,
    and begin motion of the robot
    """
    lidar = Lidar()
    r = Robot()
    rm = RobotMover(r)
    fg = FuncGenerator(rm)
    # points = [(3, 1), (2, 1), (2, 2), (2, 3), (3, 3), (3, 4), (3, 5), (3, 6), (3, 7), (4, 7), (4, 6), (3, 6), (2, 6)]
    points = [(3, 0), (2.5, 0), (2, 0), (1, 0)] #, (1.75, 2.5), (2.5, 2.5)] # replace with output of Ryan's Dijkstra stuff
    funcs = fg.get_funcs(points)
    pe = Path_Executor(rm, lidar, funcs)
    print "Wait for Ready, then press enter to continue"
    x = raw_input()
    pe.apply_next_action()
    # exit(0)
    rospy.spin()
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
