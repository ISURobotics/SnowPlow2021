from Robot import *
from Robot_Mover import RobotMover
from Path_Executor import *
from Func_Generator import FuncGenerator
import Path_Finder
import time
import Sensors

def main():
    """
    This function will initialize the robot and lidar,
    create the object map with lidar data,
    and begin motion of the robot
    """
    # for debugging - set to True for competition
    use_pathfinding = False

    sensors = Sensors()
    r = Robot()
    rm = RobotMover(r)
    fg = FuncGenerator(rm)

    sensors.init_lidar()
    sensors.init_imu()

    points = [(0, 0), (-0.5, 0), (-0.5, -1), (-1, -1), (-1, 0), (-2, 0)]


    print "Preparing path..."

   # points = []
    if use_pathfinding:
        object_points = sensors.get_obstacle_points()
        print "Obstacles at: "
        print object_points
        path_points = Path_Finder.path_generator(object_points)
        print path_points
        points = sensors.get_movement_points(path_points)
    else:
        pass
        #points = [[0, 0], [-10, 0]]
    print points
    funcs = fg.get_funcs(points)

    r.wait_for_pub()

    pe = Path_Executor(rm, funcs)
    # print "Wait for Ready, then press enter to continue"
    # x = raw_input()
    pe.apply_next_action()
    # exit(0)
    rospy.spin()
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
