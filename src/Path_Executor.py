class Path_Executor:
    def __init__(self, mover, sensors, path):
        self.mover = mover
        # The robot mover has a listener system for when it finishes an action
        self.mover.add_finish_listener(lambda: self.mover_finished_action())
        self.sensors = sensors
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
            Runs the next action in the path sequence
        """
        self.path[self.current_step](self.sensors)
        self.current_step += 1
        return self.current_step >= len(self.path)

    def mover_finished_action(self):
        """
            Runs when finishing an action in the path sequence
            (Extra pausing or debugging stuff can be done here)
        """
        # x = raw_input()
        # if x == 'q':
        #     exit()
        self.apply_next_action()
