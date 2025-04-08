from Robot_Mover import Robot_Mover

class Func_Generator(object):
    """
        This class is used to convert a path of (row, column) points into a list of movement functions, 
        which the Path_Executor can run in seqence.
    """
    def __init__(self, mover: Robot_Mover):
        self.mover = mover

    def move_forward(self, meters: float):
        """
            Returns a function to tell the mover to go forward meters meters
        """

        print ("move forward " + str(meters))
        if meters < 0:
            print ("returning move_backward instead")
            return lambda: self.mover.move_backward(-meters)
        return lambda: self.mover.move_forward(meters)

    def move_backward(self, meters: float):
        """
            Returns a function to tell the mover to go backward meters meters
            (unused)
        """

        print ("move back " + str(meters))
        if meters < 0:
            print ("returning move_forward instead")
            return lambda: self.mover.move_forward(-meters)
        return lambda: self.mover.move_backward(meters)

    def rotate_left(self, degrees: float):
        """
            Returns a function to tell the mover to turn left degrees degrees
        """

        print ("rotate left " + str(degrees))
        #return lambda p_lidar: self.mover.rotate_left(p_lidar, degrees)
        return lambda: self.mover.rotate_left_imu(degrees)

    def rotate_right(self, degrees: float):
        """
            Returns a function to tell the mover to turn right degrees degrees
        """

        print ("rotate right " + str(degrees))
        #return lambda p_lidar: self.mover.rotate_right(p_lidar, degrees)
        return lambda: self.mover.rotate_right_imu(degrees)
    
    def pursue(self, pt):
        return lambda: self.mover.move_toward(pt)

    def get_funcs(self, points: list[tuple]) -> list:
        """
            Given a list of points from a pathfinding algorithm, 
            returns a list of lambdas pointing to functions in the mover object.
            Starts at points[0]

            Will only move in the cardinal directions. If two consecutive points are not in the same row nor the same column,
            the loop breaks. The starting direction is assumed to be up.

            If the points imply the robot should move forward, then backward, it will move forward by the difference.
            e.g. (1, 2) (1, 4) (1, 3) when facing right results in just moving forward 1
            Our robot shouldn't need to do that, but it's worth mentioning.

            points is a list of (row, column) pairs. The first item is the start point.
        """
        funcs = []
        print("Points: ", points)
        for p in points:
            funcs.append(self.pursue(p))
        return funcs

    # points = [(3, 1), (2, 1), (2, 2), (2, 3), (3, 3), (3, 4), (3, 5), (3, 6), (3, 7), (4, 7), (4, 6), (3, 6), (2, 6)]
    # points_to_functions(points)
