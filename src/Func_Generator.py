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
        """
            Returns a function to tell the mover to "pursue" a certain point
            Will replace the move_forward and move_backward functions
        """
        return lambda: self.mover.pursue(pt)

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
        current = points[0]
        funcs = []
        up = 0
        left = 1
        down = 2
        right = 3
        dir = up
        print("Points: ", points)
        for p in points:
            if p == current:
                continue
            if dir == up:
                if p[1] == current[1]:
                    pass # We still haven't reached the end of the straight, carry on
                elif p[0] == current[0]:
                    # We have reached the end of the straight, add pursue function and rotation functions
                    funcs.append(self.pursue(current))
                    if p[1] > current[1]:
                        funcs.append(self.rotate_right(90))
                        dir = right
                    else:
                        funcs.append(self.rotate_left(90))
                        dir = left
                else:
                    print ("Diagonals not supported")
                    break

            elif dir == down:
                if p[1] == current[1]:
                    pass
                elif p[0] == current[0]:
                    funcs.append(self.pursue(current))
                    if p[1] < current[1]:
                        funcs.append(self.rotate_right(90))
                        dir = left
                    else:
                        funcs.append(self.rotate_left(90))
                        dir = right
                else:
                    print ("Diagonals not supported")
                    break

            elif dir == left:
                if p[0] == current[0]:
                    pass
                elif p[1] == current[1]:
                    funcs.append(self.pursue(current))
                    if p[0] < current[0]:
                        funcs.append(self.rotate_right(90))
                        dir = up
                    else:
                        funcs.append(self.rotate_left(90))
                        dir = down
                else:
                    print ("Diagonals not supported")
                    break

            elif dir == right:
                if p[0] == current[0]:
                    pass
                elif p[1] == current[1]:
                    funcs.append(self.pursue(current))
                    if p[0] > current[0]:
                        funcs.append(self.rotate_right(90))
                        dir = down
                    else:
                        funcs.append(self.rotate_left(90))
                        dir = up
                else:
                    print ("Diagonals not supported")
                    break
            
            current = p
        funcs.append(self.pursue(current))
        return funcs

    # points = [(3, 1), (2, 1), (2, 2), (2, 3), (3, 3), (3, 4), (3, 5), (3, 6), (3, 7), (4, 7), (4, 6), (3, 6), (2, 6)]
    # points_to_functions(points)
