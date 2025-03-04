
class Movement_Threshold:

    def __init__(self, axis_func, trigger_when_above: bool, value: float, function, tag: str):
        """
            :param: axis_func: A lambda pointing to a function in Axes.py that returns true if the sensor data has passed this threshold or false otherwise
            :param: trigger_when_above: true if this threshold should trigger when a value greater than this is detected, false otherwise
            :param: value: the value to watch for. If the axis_func is translational (Axes.LIDAR_X or AXES.LIDAR_Y), 
                    it watches for this displacement relative to the origin in meters.
                    If the axis is rotational (Axes.IMU_ROT), the value is in radians and it watches for a range between the given value and the given value + pi/4 radians if rotating left,
                    or the given value - pi/4 radians if rotating right, accounting for the jump from -pi to pi. If the target value is outside
                    of the range [-pi, pi], subtract or add 2pi to bring it back into the range.
                    Rotating more than pi radians (180 degrees) in a single instruction is not recommended.
            :param: function: A lambda function to run when this threshold is reached
            :param: tag: An identifier for the type of threshold this is. Used to easily delete threshold listeners.
                    Values in use are:
                    move
                    rotate
                    correct
                    stop correct
                    slow
        """
        self.axis_func = axis_func
        self.trigger_when_above = trigger_when_above
        self.value = value
        self.function = function
        self.tag = tag
