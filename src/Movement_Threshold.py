class Movement_Threshold:
    X_AXIS = 0
    Y_AXIS = 1
    Z_ROTATION = 2

    def __init__(self, axis, trigger_when_above, value, function, tag):
        """
            axis: X_AXIS, Y_AXIS, or Z_ROTATION (0, 1, or 2)
            trigger_when_above: true if this threshold should trigger when a value greater than this is detected, false otherwise
            value: the value to watch for. If the axis is X_AXIS or Y_AXIS, it watches for this displacement relavtive to the origin.
            If the axis is Z_ROTATION, it watches for a range between the given value and the given value + pi/4 radians if rotating left,
            or the given value - pi/4 radians if rotating right, accounting for the jump from -pi to pi. If the target value is outside
            of the range [-pi, pi], subtract or add 2pi to bring it back into the range.
            Rotating more than pi radians (180 degrees) in a single instruction is not recommended.
        """
        self.axis = axis
        self.trigger_when_above = trigger_when_above
        self.value = value
        self.function = function
        self.tag = tag
