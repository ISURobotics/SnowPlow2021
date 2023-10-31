from Robot import *
from Robot_Mover import RobotMover
from Path_Executor import *
from Func_Generator import FuncGenerator
import Path_Finder
import time

def main():
    """
    This function will initialize the robot and lidar,
    create the object map with lidar data,
    and begin motion of the robot
    """
    # for debugging
    use_pathfinding = 0

    lidar = Lidar()
    r = Robot()
    rm = RobotMover(r)
    fg = FuncGenerator(rm)


    points = [(0, 0), (-2, 0), (-2, -1), (-1, -1), (-1, 0), (-2, 0)]

    lidar.wait_for_pose_set()

    print "Preparing path..."

    points = []
    if use_pathfinding:
    	object_points = lidar.prepare_obstacle_points()
    	print "Obstacles at: "
    	print object_points
        path_points = Path_Finder.path_generator(object_points)
        print path_points
        points = lidar.prepare_movement_points(path_points)
    else:
        points = [[0, 0], [-0.5, 0], [-0.5, -0.5], [-1, -0.5]]

    funcs = fg.get_funcs(points)

    r.wait_for_pub()

    pe = Path_Executor(rm, funcs)
    print "Wait for Ready, then press enter to continue"
    x = raw_input()
    pe.apply_next_action()
    # exit(0)
    rospy.spin()
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
