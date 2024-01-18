# Ryan Madigan
#
# This is to be used for the Autonomous Snowplow segment of the ISU Robotics Club in Dunwoody's Ion Autonomous Snowplow Competition
#
# The code's purpose is to provide a path to clear the snow off of the path defined by the 2023 Competition Rulebook
# which can be found at https://www.autosnowplow.com/welcome.html. The teams plan to clear the field is by first going up
# into the lower part of the snow path, left while staying on the bottom half till the end of the snow, up into the top half,
# toward the right till the end of the path, down into the bottom half again, left until back at the middle again, and finally
# down, back into the "garage". Essentially taking a clockwise path around the entire snow path.
#
# The snowfield is represented by a 2D array which is is 28 x 56 with each square representing 0.25m x 0.25m in real life.
#
#
# For this competition, there will be two cones randomly placed within the field, one in the snow path and one outside of the snow path
# Since the teams robot is roughly 1.2m x 1.4m will only be represented by one spot on the grid, all cones will be represented by an x by y
# section of the snowfield, meaning they will "take up" 0.25*x meters by 0.25*y meters on the grid. This extra space will be a buffer to
# ensure the snowplow doesn't crash into the cones. The x and y variables can be set down below.
#
# To find the cones on the field, the team will be using a SICK LiDAR system which will be able to tell if there are any cones
# within the snowfield. The LiDAR will also (in cooperation with the HECTOR SLAM algorithm) be able to tell the coordinate
# points of where it is located, based off of its start point (0, 0).
#
# To actually path find, Dijkstra's Algorithm is used. This algorithm finds the shortest path between two points by implementing
# breadth first search on a weighted 2D array (where the value at each position in the array represents the "weight" or distance of
# that point. The higher the weight, the more it "costs" to travel through that point)
#
# Since the start point and end point are right next to each other, the path will be generated by connecting smaller paths
# found by the pathfinder (ie - find path going up, then add onto that path by path finding to the left, etc.)
# This method does cause some issues however, if there is a cone at the endpoint of any of the predefined movements. Because of this,
# the algorithm will instead search for a point around the predefined endpoint. It will search in a specific direction (defined later)
# depending on which direction the path is supposed to be taking (up, down, left, right). Only searching in one specific direction allows
# the pathfinder to work, however, it does cause some "irregular" paths
#




import pandas as pd
import numpy as np
from queue import PriorityQueue
from matplotlib import pyplot


# Class Variables
cb_width = 3   # INTEGER, width for the cone buffer to stretch left and right individually, measured in 0.25 m increments (ie 3 = 0.75m left and 0.75m right)
cb_height = 3  # INTEGER, height for the cone buffer to stretch up and down individually, measured in 0.25 m increments (ie 3 = 0.75m up and 0.75m down)


# param  grid - weighted 2D array representing the snowfield
# param  pos_of_cones - array of tuples which represent all of the positions on the field where cones are at
# returns  array of tuples that represent points of a 2 block (0.5m) buffer around the position of the cone
#
# The teams snowplow will be represented by only 1 0.25m x 0.25m block throughout pathfinding despite the snowplows real
# dimensions being larger. To account for this, and make sure the snowplow will not hit the cones, a 2 block (0.5m) buffer
# will be placed in all directions around the cones. This method, takes in the current position of the cones and returns
# all the points of the aforementioned buffer
def find_cone_buffer(grid, pos_of_cones):
    # array of tuples which represent the points on the grid/snowfield where a buffer is to be placed
    cone_buffer = []
    # loops for number of cones on the field
    for num in range(len(pos_of_cones)):
        # I'm suspicious about the most recent change. Test if possible
        # loops to place a row buffer around the cone
        for r in range(2 * cb_height + 1):
            # loops to place a column buffer around the cones
            for c in range(2 * cb_width + 1):
                # checks if the position is out of bounds on the snowfield/grid
                if (pos_of_cones[num][0] + r - cb_height) < 0 or (pos_of_cones[num][0] + r - cb_height) > (grid.shape[0] - 1) or (pos_of_cones[num][1] + c - cb_width) < 0 or (pos_of_cones[num][1] + c - cb_width) > (grid.shape[1] - 1):
                    continue
                # otherwise, adds that position to the cone buffer position list
                else:
                    # checks to make sure position not already accounted for
                    if [pos_of_cones[num][0] + r - cb_height, pos_of_cones[num][1] + c - cb_width] not in cone_buffer:
                        cone_buffer.append([pos_of_cones[num][0] + r - cb_height, pos_of_cones[num][1] + c - cb_width])
    return cone_buffer



# param  grid - weighted 2D array representing the snowfield
# param  start - tuple representing the start position of the snowplow on the grid
# param  end - tuple representing the end position of the snowplow on the grid
# returns  new endpoint where no cone is located
#
# Finds the position on the grid closest to the given endpoint, however it only searches in one specific direction for each direction
# This point could be the given point. It only searches for a new point if there is a cone at the predetermined endpoint
#
# PATH DIRECTION (FROM START TO FINISH)     SEARCH DIRECTION
# Left                                      Up
# Right                                     Down
# Up                                        Left
# Down                                      Right
def find_pos_nearby_end(grid, start, end):
    # if the endpoint is not where a cone is, return the given endpoint
    if grid[end] != 0 and grid[end] != float('inf'):
        return end

    # identifies the direction (up, down, left, right) the start to endpoint is in
    movements = identify_direction(grid, start, end)
    # if going right, find first non-cone spot down
    if movements[0] == (0, 1, 1):
        while grid[end] == 0:
            end = (end[0] - 1, end[1])
    # if going left, find first non-cone spot up
    elif movements[0] == (0, -1, 1):
        while grid[end] == 0:
            end = (end[0] + 1, end[1])
    # if going up, find first non-cone spot to the left
    elif movements[0] == (-1, 0, 1):
        while grid[end] == 0:
            end = (end[0], end[1] - 1)
    # if going down, find the first non-cone spot to the right
    else:
        while grid[end] == 0:
            end = (end[0], end[1] + 1)

    # return new endpoint
    return end



# param  grid - weighted 2D array representing the snowfield
# param  start - tuple representing the start position of the snowplow on the grid
# param  end - tuple representing the end position of the snowplow on the grid
# returns  array of tuples representing the various possible movements types (up, down, left, right) and their cost/weight
#
# Identifies the direction (up, down, left, right) the start to endpoint goes. Based off of which direction the path is in,
# movements (array of tuples representing the 4 directions of travel and their weight/cost to move in that direction). To ensure
# the robot stays as much as possible on a straight line between the start and endpoints and only deviates to avoid cones.
# To ensure this happens, the direction between the start and end is weighted half as much and all points on the grid besides
# the points directly between the start and endpoints become weighted +10. Additionally, for each direction, due to the nature of
# this competition, there is a preferred side-to-side/perpendicular movement as defined below. To accomplish this, a large weight
# of 100 is place in the opposite of the preferred direction in order to deter movement in that direction past where the start point is
#
# DIRECTION     PREFErRED SIDE-TO-SIDE MOVEMENT
# Up            Right
# Down          Left
# Left          Down
# Right         Up
def identify_direction(grid, start, end):
    # >0 if going right, <0 if going left
    rl = end[1] - start[1]
    # >0 if going down, <0 if going up
    ud = end[0] - start[0]

    # if going right, initiate the movements array to prefer going right
    if rl > 0:
        movements = [
            (0, 1, 1),   # go right, prefers to go this way which is why weight of moving is 1(last part of tuple)
            (-1, 0, 2),  # go up
            (1, 0, 2),   # go down
            (0, -1, 2),  # go left
        ]

        # For every point on the field not directly between start and end, and the point isn't a cone(weight of 0)
        # add 10 to its weight to encourage movement along direct path as much as possible.
        # Also add an additional 90 to the weight of all points below the direct path between start and end
        for c in range(grid.shape[1]):
            if grid[start[0] + 1, c] != 0:
                grid[start[0] + 1, c] = 90
            for r in range(grid.shape[0]):
                if r != start[0] and grid[r, c] != 0:
                    grid[r, c] = grid[r, c] + 10

    # if going left, initiate the movements array to prefer going left
    elif rl < 0:
        movements = [
            (0, -1, 1),  # go left, prefers to go this way which is why weight of moving is 1(last part of tuple)
            (1, 0, 2),   # go down
            (-1, 0, 2),  # go up
            (0, 1, 2),   # go right

        ]

        # For every point on the field not directly between start and end, and the point isn't a cone(weight of 0)
        # add 10 to its weight to encourage movement along direct path as much as possible.
        # Also add an additional 90 to the weight of all points above the direct path between start and end
        for c in range(grid.shape[1]):
            if grid[start[0] - 1, c] != 0:
                grid[start[0] - 1, c] = 90
            for r in range(grid.shape[0]):
                if r != start[0] and grid[r, c] != 0:
                    grid[r, c] = grid[r, c] + 10


    # if going up, initiate the movements array to prefer going up
    elif ud < 0:
        movements = [
            (-1, 0, 1),  # go up
            (0, 1, 2),  # go right
            (0, -1, 2),  # go left
            (1, 0, 2),  # go down
        ]

        # For every point on the field not directly between start and end, and the point isn't a cone(weight of 0)
        # add 10 to its weight to encourage movement along direct path as much as possible.
        # Also add an additional 90 to the weight of all points to the left the direct path between start and end
        for r in range(grid.shape[0]):
            if grid[r, start[1] - 1] != 0:
                grid[r, start[1] - 1] = 90
            for c in range(grid.shape[1]):
                if c != start[1] and grid[r, c] != 0:
                    grid[r, c] = grid[r, c] + 10

            # for c in range(grid.shape[1] - start[1] - 1):
            #    if grid[r, c + start[1] + 1] != 0:
            #        grid[r, c + start[1] + 1] = grid[r, c + start[1] + 1] + 5

    # if going down, initiate the movements array to prefer going down
    else:
        movements = [
            (1, 0, 1),  # go down
            (0, -1, 2),  # go left
            (0, 1, 2),  # go right
            (-1, 0, 2),  # go up
        ]

        # For every point on the field not directly between start and end, and the point isn't a cone(weight of 0)
        # add 10 to its weight to encourage movement along direct path as much as possible.
        # Also add an additional 90 to the weight of all points to the right the direct path between start and end
        for r in range(grid.shape[0]):
            if grid[r, start[1] + 1] != 0:
                grid[r, start[1] + 1] = 90
            for c in range(grid.shape[1]):
                if c != start[1] and grid[r, c] != 0:
                    grid[r, c] = grid[r, c] + 10


    return movements


# param  grid - weighted 2D array representing the snowfield
# param  start - tuple representing the start position of the snowplow on the grid
# param  end - tuple representing the end position of the snowplow on the grid
# return  2D array of tuples representing the points on the snowfield the robot will travel along
#
# Uses Dijkstra's algorithm to find the shortest path between start and end in a weighted 2D array. This algorithm finds
# the shortest path between two points by implementing
# breadth first search on a weighted 2D array (where the value at each position in the array represents the "weight" or
# distance of that point. The higher the weight, the more it "costs" to travel through that point)
def path_finder(grid, start, end):

    # creates a copy of the grid as to not change the inputted grid
    grid_copy = np.array(grid, copy=True)

    # define the possible movements and their weights
    movements = identify_direction(grid_copy, start, end)
    # if the given endpoint has a cone on it, find the closest available point
    end = find_pos_nearby_end(grid_copy, start, end)

    # keep track of the visited cells
    visited = set()

    # keep track of the previous cell for each cell
    prev = {}

    # keep track of the distance for each cell
    dist = {}

    # initialize the distance for each cell to infinity
    for i in range(grid_copy.shape[0]):
        for j in range(grid_copy.shape[1]):
            dist[i, j] = float('inf')

    # define a priority queue to select the next cell to visit
    # the priority is the distance of the cell from the starting point
    pq = PriorityQueue()

    # set the starting point
    pq.put((0, start))
    dist[start] = 0

    print(grid_copy)
    # repeat until the priority queue is empty
    while not pq.empty():
        # get the cell with the smallest distance
        _, current_point = pq.get()

        # if the cell has not been visited
        if current_point not in visited:
            # mark the cell as visited
            visited.add(current_point)

            # for each possible movement from the current cell
            for r, c, w in movements:
                # get the destination cell
                new_point = (current_point[0] + r, current_point[1] + c)

                # if the destination cell is inside the grid and has not been visited
                if 0 <= new_point[0] < grid_copy.shape[0] and 0 <= new_point[1] < grid_copy.shape[
                    1] and new_point not in visited:
                    if grid_copy[new_point] == 0 or grid_copy[new_point] == float('inf'):
                        continue

                    # if the distance from the starting point to the destination cell is smaller
                    # than the current distance, update the distance and the previous cell
                    if dist[current_point] + (w * grid_copy[current_point]) < dist[new_point]:
                        dist[new_point] = dist[current_point] + (w * grid_copy[current_point])
                        prev[new_point] = current_point

                        # add the destination cell to the priority queue
                        pq.put((dist[new_point], new_point))

    # initialize the path with the ending point
    path = [end]

    # get the previous cell for each cell in the path
    # until the starting point is reached
    while path[-1] != start:
        path.append(prev[path[-1]])

    # reverse the path
    path = path[::-1]

    return path



# return  2D array representing the snowfield as defined for this competition. There are different weights for different
#        parts of the field
# WEIGHT    PART OF FIELD
# 0         cone or no go section
# 1         primary/top section of snow
# 2         secondary/bottom section of snow
# 4         gravel/non-snow
#
# Creates 2D array representing the snowfield as defined for this competition with various weights for the various sections
# of the snowfield as defined above
def make_grid():
    grid = np.ones((28, 56))
    for r in range(16, 28):
        for c in range(18):
            grid[r, c] = 0
            grid[r, c + 38] = 0

    for r in range(6):
        for c in range(56):
            grid[r, c] = 4
            grid[r + 10, c] = 4

    for r in range(10, 14):
        for c in range(24, 32):
            grid[r, c] = 2

    for r in range(16):
        for c in range(8):
            grid[r, c] = 4
            grid[r, c + 48] = 4

    for r in range(16, 28):
        for c in range(18, 38):
            grid[r, c] = 4

    return grid


# param  grid - weighted 2D array representing the snowfield
# param  pos_of_cones_with_buffer - 2D array of tuples representing all points on the snowfield where cone buffers are
# return  inputted grid except with 0s placed where cones are
#
# Places cones in the snowfield
def place_cones_on_grid(grid, pos_of_cones_with_buffer):

    for i in range(len(pos_of_cones_with_buffer)):
        grid[pos_of_cones_with_buffer[i][0], pos_of_cones_with_buffer[i][1]] = 0

    return grid

# param  path - array of tuples representing the points which the snowplow will travel along
# return  altered path with removed consecutive repeats
#
# Removes all consecutive repeats
def delete_duplicates(path):
    limit = len(path) - 1
    i = 0
    while i < limit:
        if path[i][0] == path[i + 1][0] and path[i][1] == path[i + 1][1]:
            del path[i + 1]
            limit -= 1
        i += 1

    return path


# param  grid - weighted 2D array representing the snowfield
# param  path - array of tuples representing the points which the snowplow will travel along
#
# Visual representation of the snowplow traveling along the given path
def display_path(grid, path):
    pyplot.imshow(grid, cmap='magma')
    pyplot.pause(2)
    for point in range(len(path)):
        grid[path[point]] = grid[path[point]] + 1
        pyplot.imshow(grid, cmap='magma')
        # pyplot.pause(0.05)

    pyplot.show()


# param  pos_of_cones - position of all cones on the field which will be found out from the LiDAR data
# return  2D array of tuples representing the full path which the snowplow will travel along
#
# Main function which only requires the position of cones on the field to find the full path which the snowplow will travel along
def path_generator(pos_of_cones):

    # creates grid
    grid = make_grid()
    # adds buffer around all cones
    pos_of_cones_with_buffer = find_cone_buffer(grid, pos_of_cones)
    # places cones with buffer on grid
    grid = place_cones_on_grid(grid, pos_of_cones_with_buffer)


    # all predetermined start and endpoints
    start1 = (20, 26)
    end1 = (8, 26)
    end2 = (8, 7)
    end3 = (9, 7)
    end4 = (9, 6)
    end5 = (7, 6)
    end6 = (7, 48)
    end7 = (6, 48)
    end8 = (6, 49)
    end9 = (8, 49)
    end10 = (8, 29)
    end11 = (22, 29)

    # for all predetermined start and endpoints, finds the best path between the two and then adds it on to the end of the
    # previously found paths
    path = path_finder(grid, start1, end1)
    start2 = find_pos_nearby_end(np.array(grid, copy=True), start1, end1)
    path.extend(path_finder(grid, start2, end2))
    start3 = find_pos_nearby_end(np.array(grid, copy=True), start2, end2)
    path.extend(path_finder(grid, start3, end3))
    start4 = find_pos_nearby_end(np.array(grid, copy=True), start3, end3)
    path.extend(path_finder(grid, start4, end4))
    start5 = find_pos_nearby_end(np.array(grid, copy=True), start4, end4)
    path.extend(path_finder(grid, start5, end5))
    start6 = find_pos_nearby_end(np.array(grid, copy=True), start5, end5)
    path.extend(path_finder(grid, start6, end6))
    start7 = find_pos_nearby_end(np.array(grid, copy=True), start6, end6)
    path.extend(path_finder(grid, start7, end7))
    start8 = find_pos_nearby_end(np.array(grid, copy=True), start7, end7)
    path.extend(path_finder(grid, start8, end8))
    start9 = find_pos_nearby_end(np.array(grid, copy=True), start8, end8)
    path.extend(path_finder(grid, start9, end9))
    start10 = find_pos_nearby_end(np.array(grid, copy=True), start9, end9)
    path.extend(path_finder(grid, start10, end10))
    start11 = find_pos_nearby_end(np.array(grid, copy=True), start10, end10)
    path.extend(path_finder(grid, start11, end11))

    # deletes consecutive duplicates
    path = delete_duplicates(path)

    # displays path
    # display_path(grid, path)
    return path




# def generate_path(obstacles):
if __name__ == '__main__':

    # allows for more print columns in terminal
    pd.set_option('display.width', 50)
    np.set_printoptions(linewidth=400)
    np.set_printoptions(threshold=np.inf)

    # test position of cone
    pos_of_cones = [(8, 15)]

    path_generator(pos_of_cones)
