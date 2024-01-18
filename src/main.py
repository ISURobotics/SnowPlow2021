from Robot import *
from Robot_Mover import *
from Path_Executor import *
from Func_Generator import *
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
    use_pathfinding = True

    r = Robot()
    sensors = r.sensors # We were creating a sensors object here and in robot until recently. Now we only create one in robot and use it here
    rm = Robot_Mover(r)
    fg = Func_Generator(rm)

    sensors.init_lidar()
    sensors.init_imu()

    # points = [(0, 0), (-0.5, 0), (-0.5, -1), (-1, -1), (-1, 0), (-2, 0)]
    # Uncomment for left box drill
    # points = [(0, 0), (-0.5, 0), (-0.5, -0.5), (0, -0.5)] * 5
    
    # Uncomment for right box drill
    # points = [(0, 0), (-0.5, 0), (-0.5, 0.5), (0, 0.5)] * 5


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
