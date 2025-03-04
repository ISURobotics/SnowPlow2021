from Robot import *
from Robot_Mover import *
from Path_Executor import *
from Func_Generator import *
import Path_Finder
import Sensors

def main():
    """
    This function will initialize the robot and lidar,
    create the object map with lidar data,
    and begin motion of the robot

    Before running the main function, we need to start sick_scan_xd, Arduino_Node, and GPS_Node.
    """

    # for debugging - set to True for competition, False for testing with a preprogrammed path
    use_pathfinding = True

    r: Robot = Robot()
    sensors: Sensors = r.sensors
    rm: Robot_Mover = Robot_Mover(r)
    fg: Func_Generator = Func_Generator(rm)

    sensors.init_lidar()
    sensors.init_imu()

    points = []

    # Uncomment for left box drill
   # points = [(0, 0), (-0.5, 0), (-0.5, -0.5), (0, -0.5)] * 5
    
    # Uncomment for right box drill
   # points = [(0, 0), (-0.5, 0), (-0.5, 0.5), (0, 0.5)] * 5

    #points = [(0, 0), (-1, 0)]

    print("Preparing path...")
    if use_pathfinding:
        object_points = sensors.get_obstacle_points()
        print("Obstacles at: ")
        print(object_points)
        path_points = Path_Finder.path_generator(object_points)
        print(path_points)
        points = sensors.get_movement_points(path_points)

    print(points)
    funcs = fg.get_funcs(points)

    r.wait_for_pub()

    pe = Path_Executor(rm, funcs)

    # This starts executing the path
    pe.apply_next_action()
    print("running next action")
    # exit(0)
    #rclpy.spin(r)
    while True:
        rclpy.spin(r)


if __name__ == "__main__":
    main()
