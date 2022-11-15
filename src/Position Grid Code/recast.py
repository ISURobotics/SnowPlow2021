"""
    File: recast.py
    Author: ISU Robotics - Snow Plow
    Creation Date: 11/14/2022
"""

import re


# Defines pose class
# Represents cardinality and directionality at given point of vehicle
class Pose:
    def __init__(self, seq, position, orientation):
        self.seq = seq
        self.position = {'X': position[0],
                         'Y': position[1],
                         'Z': position[2]}
        self.orientation = {'X': orientation[0],
                            'Y': orientation[1],
                            'Z': orientation[2],
                            'W': orientation[3]}

    def getSeq(self):
        return self.seq

    def getPosition(self):
        return self.position

    def getOrientation(self):
        return self.orientation


# Imports pose data into an array of pose objects
def importPoseData(source):
    file = open(source, "r")
    data = []

    seq = None
    position = []
    orientation = []
    reading = -1
    for line in file:
        if "seq:" in line:
            if reading == 0:
                data.append(Pose(seq, position, orientation))
                position = []
                orientation = []
                reading = 0
            seq = int(re.findall(r'\d+', line)[0])
        elif "position:" in line:
            reading = 1
        elif "orientation:" in line:
            reading = 2
        elif reading == 1:
            position.append(float(re.findall(r'[+-]?\d+(?:\.\d+)?', line)[0]))
            if len(position) == 3:
                reading = 0
        elif reading == 2:
            orientation.append(float(re.findall(r'[+-]?\d+(?:\.\d+)?', line)[0]))
            if len(orientation) == 4:
                reading = 0
    data.append(Pose(seq, position, orientation))

    return data
