import numpy as np


def dist(p, oldPos):
    """
        Returns the distance between two pose objects
        p and oldPos are both pose objects
    """
    return np.sqrt(
        np.power(p.position.x - oldPos.position.x, 2) + np.power(p.position.y - oldPos.position.y, 2) + np.power(
            p.position.z - oldPos.position.z, 2))


def angle_between(pose1, pose2):
    """
        Returns the angle between two poses' orientations in radians
        Return value will be in the range [0, pi]
        pose1 and pose2 are both pose objects
        TODO: Test
    """
    rad1 = 2 * np.arctan(pose1.orientation.z / pose1.orientation.w)
    rad2 = 2 * np.arctan(pose2.orientation.z / pose2.orientation.w)
    return np.minimum(rad1 - rad2, rad2 - rad1)  # TODO: Get these to work when on opposite sides of 180

