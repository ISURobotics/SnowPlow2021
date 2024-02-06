from Robot import Robot
import time
plow_done = False

r = Robot()
# start plowing
r.left.set_speed(100)
r.right.set_speed(100)

while not plow_done:
    time.sleep(.1)

r.stop()
