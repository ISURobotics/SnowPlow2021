# NOT IN USE ANY LONGER
# I THINK
class Path_Planner:
    def __init__(self, mover, lidar):
        self.mover = mover
        self.lidar = lidar
        self.path = [
            lambda pose: mover.moveForward(pose, 5), 
            lambda pose: mover.moveBackward(pose, 5)
        ]
        self.current_step = 0

    def apply_next_action(self):
        pose = self.lidar.get_pose()
        self.path[self.current_step](pose)
        self.current_step += 1
        return self.current_step >= len(self.path)