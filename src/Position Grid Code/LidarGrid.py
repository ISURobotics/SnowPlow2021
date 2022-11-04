#0 = non legal ground
#1 = ground
#2 = snow
#3 = garage
#4 = cone
#5 = robot

#Grid of quarter meters^2 (each group of 4 is a meter^2)
grid_view = [[1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 3, 3, 3, 3, 3, 3],
            [1, 1, 1, 2, 2, 1, 1, 1, 3, 3, 3, 3, 3, 3], 
            [1, 1, 1, 2, 2, 1, 1, 1, 3, 3, 3, 3, 3, 3], 
            [1, 1, 1, 2, 2, 1, 1, 1, 3, 3, 5, 5, 3, 3], 
            [1, 1, 1, 2, 2, 1, 1, 1, 3, 3, 5, 5, 3, 3], # <-- robot facing this way (positive x)
            [1, 1, 1, 2, 2, 1, 1, 1, 3, 3, 3, 3, 3, 3], #
            [1, 1, 1, 2, 2, 1, 1, 1, 3, 3, 3, 3, 3, 3], # |
            [1, 1, 1, 2, 2, 1, 1, 1, 3, 3, 3, 3, 3, 3], # |
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0], # |
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0], #\/
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0], # posiive y
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0]]
        
grid_rows = len(grid_view)
grid_columns = len(grid_view[0])


class tile:
    def __init__(self, legal, snow, garage, cone, robot):
        self.legal = legal
        self.snow = snow
        self.garage = garage
        self.cone = cone
        self.robot = robot
    def __str__(self):
        if self.robot:
            return "r"
        elif self.cone:
            return "c"
        elif self.garage:
            return "g"
        elif self.snow:
            return "s"
        elif self.legal:
            return "l"
        elif self.legal == False:
            return "o"


grid = [[0 for x in range(grid_columns)] for x in range(grid_rows)]

def init_grid():
    for r in range(grid_rows):
        for c in range(grid_columns):
            if grid_view[r][c] == 0:
                new_tile = tile(False, False, False, False, False)
                grid[r][c] = new_tile
            if grid_view[r][c] == 1:
                new_tile = tile(True, False, False, False, False)
                grid[r][c] = new_tile
            if grid_view[r][c] == 2:
                new_tile = tile(True, True, False, False, False)
                grid[r][c] = new_tile
            if grid_view[r][c] == 3:
                new_tile = tile(True, False, True, False, False)
                grid[r][c] = new_tile
            if grid_view[r][c] == 4:
                new_tile = tile(True, False, False, True, False)
                grid[r][c] = new_tile
            if grid_view[r][c] == 5:
                new_tile = tile(True, False, True, False, True)
                grid[r][c] = new_tile  


def printGrid():
    for r in range(grid_rows):
            print()
            for c in range(grid_columns):
                print(grid[r][c], " ", end = '')

def update_grid(y, x):
    
    for r in range(grid_rows):
        for c in range(grid_columns):
            if grid[r][c].robot == True:
                grid[r][c].robot = False
                
                print(r, c)

    grid[y][x].robot = True
    grid[y+1][x].robot = True
    grid[y][x+1].robot = True
    grid[y+1][x+1].robot = True


def index_calc(lidar_x, lidar_y):
    
    lidar_x = round((lidar_x + 0.25) * 4)/4
    lidar_y = round((lidar_y - 0.25) * 4)/4

    print(lidar_x, lidar_y)

    #lidar_x = (-0.5 * gridxj) + 5.25
    #lidar_y = (0.5 * gridyi) - 6.75
    gridxc = int((-2 * lidar_x) + 10.5)
    gridyr = int((-2 * lidar_y) + 13.5)

    print(gridxc, gridyr)
    update_grid(gridyr, gridxc)

    return


init_grid()

printGrid()

index_calc(0.22, -0.22)

input("Press a key")

printGrid()
