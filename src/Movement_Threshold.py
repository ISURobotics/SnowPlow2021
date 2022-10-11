class Movement_Threshold:
    X_AXIS = 0
    Y_AXIS = 1
    Z_ROTATION = 2

    def __init__(self, axis, trigger_when_above, value, function):
        """
            axis: X_AXIS, Y_AXIS, or Z_ROTATION (0, 1, or 2)
            trigger_when_above: true if this threshold should trigger when a value greater than this is detected, false otherwise
            value: the value to watch for
        """
        self.axis = axis
        self.triggerWhenAbove = trigger_when_above
        self.value = value
        self.function = function