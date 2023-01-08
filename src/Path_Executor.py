class Path_Executor:
    def __init__(self, mover, lidar, path):
        self.mover = mover
        # The robot mover has a listener system for when it finishes an action
        self.mover.add_finish_listener(lambda: self.mover_finished_action())
        self.lidar = lidar
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
        # pose = self.lidar.get_pose()
        self.path[self.current_step](self.lidar)
        self.current_step += 1
        return self.current_step >= len(self.path)

    def mover_finished_action(self):
        """
            Runs when finishing an action in the path sequence
            (Extra pausing or debugging stuff can be done here)
        """
        self.apply_next_action()
