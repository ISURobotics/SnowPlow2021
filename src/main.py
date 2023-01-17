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
    use_pathfinding = 1

    lidar = Lidar()
    r = Robot()
    rm = RobotMover(r)
    fg = FuncGenerator(rm)

    # points = [(3, 1), (2, 1), (2, 2), (2, 3), (3, 3), (3, 4), (3, 5), (3, 6), (3, 7), (4, 7), (4, 6), (3, 6), (2, 6)]

    points = [(0, 0), (-2, 0), (-2, -1), (-1, -1), (-1, 0), (-2, 0)] #, (1.75, 2.5), (2.5, 2.5)] # replace with output of Ryan's Dijkstra stuff

    if use_pathfinding:
    	object_points = lidar.prepare_obstacle_points()
    	print "Obstacles at: "
    	print object_points
    #if use_pathfinding:
        path_points = Path_Finder.path_generator(object_points)
	print path_points
        points = lidar.prepare_movement_points(path_points)

    funcs = fg.get_funcs(points)
    pe = Path_Executor(rm, lidar, funcs)
    print "Wait for Ready, then press enter to continue"
    x = raw_input()
    #lidar.wait_for_pose_set()
    pe.apply_next_action()
    # exit(0)
    rospy.spin()
    while not rospy.is_shutdown():
        pass


if __name__ == "__main__":
    main()
