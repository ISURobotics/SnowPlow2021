class Path_Executor:
    def __init__(self, mover, lidar):
        self.mover = mover
        self.mover.add_finish_listener(lambda: self.mover_finished_action())
        self.lidar = lidar
        self.path = [
            lambda p_lidar: mover.move_forward(p_lidar, 0.5), 
            lambda p_lidar: mover.move_backward(p_lidar, 0.5)
        ]
        self.current_step = 0

    def apply_next_action(self):
        pose = self.lidar.get_pose()
        self.path[self.current_step](self.lidar)
        self.current_step += 1
        return self.current_step >= len(self.path)

    def mover_finished_action(self):
        print "Path executor says mover done"
        self.apply_next_action()