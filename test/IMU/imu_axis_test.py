import numpy as np

class Mock_Sensors:
    def get_euler(self):
        return [7 * np.pi / 8, 0, 0]
    
class Mock_Threshold:
    def __init__(self):
        self.value = -3 * np.pi / 4
        self.trigger_when_above = False

def imu_z_rotation(sensors, t):
    measured_val = 0
    above_thres = False
    eulers = sensors.get_euler()
    measured_val = eulers[0] # The IMU is oriented so that its x axis is up and down, so we need that
    # NEEDS TESTING. LOTS OF TESTING.
    print("measured: " + str(measured_val))
    if t.trigger_when_above:
        if (t.value > np.pi / 2):
            above_thres = (measured_val >= t.value) or (measured_val < t.value - 3 * np.pi / 2) # target is close to 180 degrees (pi radians)
        else:
            above_thres = (measured_val >= t.value) and (measured_val < t.value + np.pi / 2)
    else:
        if (t.value > -np.pi / 2):
            above_thres = (measured_val > t.value) or (measured_val <= t.value - np.pi / 2)
        else:
            above_thres = (measured_val > t.value) and (measured_val <= t.value + 3 * np.pi / 2)
    print(above_thres)
    print(t.trigger_when_above)
    print("")
    return (above_thres == t.trigger_when_above)

print(imu_z_rotation(Mock_Sensors(), Mock_Threshold()))