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

def quaternion_to_euler(x, y, z, w):
    """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        returns the tuple (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians