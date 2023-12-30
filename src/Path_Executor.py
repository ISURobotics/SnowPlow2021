import time
class Path_Executor:
    def __init__(self, mover, path):
        """
            Initializes with a sequence of movement functions
            :param: mover: The in-use instance of the Robot_Mover object.
                    Only used when hardcoding a path for testing
            :param: path: a list of lambdas pointing to functions in a 
                    Robot_Mover object
        """
        self.mover = mover
        # The robot mover has a listener system for when it finishes an action
        self.mover.add_finish_listener(lambda: self.mover_finished_action())
        self.path = path
        # self.path = [
        #     lambda p_lidar: mover.rotate_left(p_lidar, 45), 
        #     lambda p_lidar: mover.move_forward(p_lidar, .5),
        #     lambda p_lidar: mover.move_backward(p_lidar, .5),
        #     lambda p_lidar: mover.rotate_right(p_lidar, 45)
        # ]
        self.current_step = 0

    def apply_next_action(self):
        """
            Runs the next action in the path sequence. Returns false if there are no actions to run
        """
        if self.current_step >= len(self.path):
            return False
        self.path[self.current_step]()
        self.current_step += 1
        return True

    def mover_finished_action(self):
        """
            Runs when finishing an action in the path sequence
            (Extra pausing or debugging stuff can be done here)
        """
        if not self.apply_next_action():
            print "path ended"
            exit()
