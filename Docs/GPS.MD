# Autonomous Snowplow - GPS
## About GPS

The GPS used for this project is:
- [Product Specs](https://www.sparkfun.com/sparkfun-gps-rtk-sma-kit.html)

There are provided code examples:
- [GPS Hookup Guide](https://learn.sparkfun.com/tutorials/gps-rtk2-hookup-guide)

Our implementation:
- [Datasheet](./ZED-F9P-02B_DataSheet_UBX-21023276.pdf)

Our implementation:
- [Schematic](./SparkFun-GPS-RTK-SMA-ZED-F9P-Schematic.pdf)

Our implementation:
- [RobotGPS.py](../src/RobotGPS.py)
- [GPS_Node.py](../src/GPS_Node.py)


The last communication from the previous team in issue #30 indicates: We have made some improvements to the GPS since last competition, but there's still more stuff we could experiment with. Maybe we could make the GPS extrapolate our position in between data packets? That way, if we drop a few when moving, the robot can still make a good estimate of when it's reached the end of a movement action.
