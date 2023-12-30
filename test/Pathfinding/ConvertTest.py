def prepare_obstacle_points(points):
    # We only care about obstacles in the range of: (may need to be changed)
    # Y: .2 < y < 4.2
    # X: -4 < x < 6
    # To convert:
    # Multiply values by 4
    # Subtract y value from 28
    # Add 26 to x value

    # Get points to prepare
    point_list = points
    # FIlter points to only use those in the range of the field
    new_list = []
    for pt in point_list:
        if 4.9 > pt[0] > .9 and 7.25 > pt[1] > -7.75:
            new_list.append(pt)
    # Convert points to correspond with the pathfinding grid
    final_list = []
    for pt in new_list:
        final_pt = []
        final_pt.append(round(28 - (pt[0] * 4)))
        final_pt.append(round(26 + (pt[1] * 4)))
        final_list.append(final_pt)
    # Return list of points
    return final_list


def prepare_movement_points(points_list):
    # Reverse from prepare obstacle points
    final_list = []
    for pt in points_list:
        final_pt = []
        final_pt.append((pt[0] - 28) / 4)  # robot starts by looking in the negative x direction
        final_pt.append((pt[1] - 26) / 4)  # left of robot is negative y direction
        final_list.append(final_pt)
    return final_list


test_points = [[1.3, -6, 3], [2, 3, 4]]
obstacles = prepare_obstacle_points(test_points)
movements = prepare_movement_points(obstacles)
print("Obstacles: ", obstacles)
print("Movements: ", movements)
