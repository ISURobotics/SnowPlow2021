poses = {
    "Left": 0,
    "Right": 200,
    "Top": 0,
    "Bottom": 200
}

def getData():
    file = open("./sampleInput.txt", "r")
    poses = []
    pose = (None, None)
    for line in file:
        if "x:" in line:
            pose = (line.split()[1], pose[1])
        if "y:" in line:
            pose = (pose[0], line.split()[1])
            poses.append(pose)
            pose = (None, None)
    for pose in poses:
        print(pose)

def main():
    # in cm
    startPose = getStartingPose()
    robotPose = getRobotPose(startPose)

    print(checkOutOfBounds(robotPose))
    if poses.get(robotPose) != None:
        print("Object Hit")



def getRobotPose(pose):
    displacementX = 1
    displacementY = 1
    return (pose[0] + displacementX, pose[1] + displacementY)

def getStartingPose():
    x = 0
    y = 0
    return (x, y)

def checkOutOfBounds(pose):
    if pose[0] < poses.get("Left"):
        return True
    elif pose[0] > poses.get("Right"):
        return True
    elif pose[1] < poses.get("Top"):
        return True
    elif pose[1] > poses.get("Bottom"):
        return True
    else:
        return False

getData()