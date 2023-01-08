# Ryan Madigan

import pandas as pd
import numpy as np
from queue import PriorityQueue
from matplotlib import pyplot


def identify_direction_snow(grid, start, end):
    rl = end[1] - start[1]
    ud = end[0] - start[0]

    if rl > 0:
        movements = [
            (0, 1, 1),  # go right
            (-1, 0, 2),  # go up
            (1, 0, 2),  # go down
            (0, -1, 2),  # go left

            # (1, 1, np.sqrt(2)), # go down-right
            # (1, -1, np.sqrt(2)), # go down-left
            # (-1, 1, np.sqrt(2)), # go up-right
            # (-1, -1, np.sqrt(2)) # go up-left
        ]

        for c in range(grid.shape[1]):
            grid[start[0] + 1, c] = float('inf')
            if grid[start[0] - 1, c] != 0:
                grid[start[0] - 1, c] = grid[start[0] - 1, c] + 1


    elif rl < 0:
        movements = [
            (0, -1, 1),  # go left
            (1, 0, 2),  # go down
            (-1, 0, 2),  # go up
            (0, 1, 2),  # go right

        ]

        for c in range(grid.shape[1]):
            grid[start[0] - 1, c] = float('inf')
            if grid[start[0] + 1, c] != 0:
                for r in range(start[0] - 1):
                    grid[r - 1, c] = grid[r - 1, c] + 5



    elif ud < 0:
        movements = [
            (-1, 0, 1),  # go up
            (0, 1, 2),  # go right
            (0, -1, 2),  # go left
            (1, 0, 2),  # go down
        ]


        for r in range(grid.shape[0]):
            grid[r, start[1] - 1] = float('inf')
            if grid[r, start[1] + 1] != 0:
                for c in range(grid.shape[1] - start[1] - 1):
                    grid[r, c + start[1] + 1] = grid[r, c + start[1] + 1] + 5

    else:
        movements = [
            (1, 0, 1),  # go down
            (0, -1, 2),  # go left
            (0, 1, 2),  # go right
            (-1, 0, 2),  # go up

        ]

    return movements


def identify_direction_gravel(grid, start, end):
    for c in range(7, 49):
        grid[5, c] = float('inf')
        grid[10, c] = float('inf')
    for r in range(5, 11):
        grid[r, 7] = float('inf')
        grid[r, 49] = float('inf')
    for c in range(23, 33):
        grid[15, c] = float('inf')
    for r in range(11, 15):
        grid[r, 22] = float('inf')
        grid[r, 32] = float('inf')

    movements = [
        (0, 1, 1),  # go right
        (-1, 0, 1),  # go up
        (1, 0, 1),  # go down
        (0, -1, 1),  # go left
    ]

    return movements


def path_finder(grid, start, end, on_snow):
    # define the possible movements from a cell
    # the value of each tuple represents the movement's direction and the weight of the edge
    grid_copy = np.array(grid, copy=True)

    #if on_snow:
    movements = identify_direction_snow(grid_copy, start, end)
    #else:
    #   movements = identify_direction_gravel(grid_copy, start, end)

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

    # reverse the path and print it
    path = path[::-1]
    for
    return path


def display_path(grid, path):
    pyplot.imshow(grid, cmap='magma')
    pyplot.pause(2)
    for point in range(len(path)):
        grid[path[point]] = grid[path[point]] + 1
        pyplot.imshow(grid, cmap='magma')
        pyplot.pause(0.05)

    pyplot.show()



# def generate_path(obstacles):
if __name__ == '__main__':
    """
    Generates a path according to preset hardcoded path, avoiding points in obstacles[]
    """
    pd.set_option('display.width', 50)
    np.set_printoptions(linewidth=400)
    np.set_printoptions(threshold=np.inf)

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

    # sets up hardcoded obstacle zone values, for testing
    grid[17, 26] = 0
    grid[17, 27] = 0

    # sets up obstacle zones from parameter
    # for obstacle in enumerate(obstacles):
    # axes are reversed here so flip x,y locations
    # grid[obstacle[1], obstacle[0]] = 0

    print(grid)

    # hardcoded first step start and end points
    start = (22, 26)
    end = (8, 26)

    # generate the complete path in an array
    # look at later, consecutive points should have different end and start points?
    path = path_finder(grid, start, end, False)
    path.extend(path_finder(grid, (7, 7), (7, 49), True))
    #path.extend(path_finder(grid, (8, 49), (8, 6), True))
    #path.extend(path_finder(grid, (9, 6), (11, 21), False))

    print(path)

    print(grid)

    display_path(grid, path)
