class IMU:
    """
        Gets acceleration, gyroscope, and magnetometer data from the Inertial Measurement Unit.
        Also does any necessary inertial navigation calculations.
    """
    def __init__(self, sensors):
        self.sensors = sensors

    def callback_imu_data(self, data):
        # TODO
        pass