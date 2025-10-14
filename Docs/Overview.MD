
## Safety - E-Stop (physical and remote) mechanisms stop all power and motion on command, Remote E-Stop is always able to cut power to the robot, Snowplow moves at a safe speed - Vehicle stops within 3 meters of either E-Stop being triggered
Guidance - Snowplow autonomously follows our planned course Remote E-Stop range ≥ 50 m,  Snowplow speed ≤ 2 m/s, Within 5 inches of our path
Operation - Robot can operate in the required conditions - The robot can operate in temperatures between 0 and 100 degrees Fahrenheit and in wet/snowy conditions.

## Overall Risk Assessment Summary
Batteries - Temperature and lack of charge are problematic because the entire system relies on the batteries - Insulating battery box, Charge rotation and frequent checking
Electronic Components - Failing or malfunctioning components can cause varying degrees of system failure such as electrical shorts, incorrect sensor output, etc. - Surge protectors, Fuses, Power relays, E-Stops
Navigation System/Software - Errors in the navigation system can cause the snowplow to move in unanticipated directions - E-Stops, Algorithm testing


## Plowing Strategy:
“T” Snowfield

LiDAR scan for cones
Start with "ideal route"
Use pathfinding algorithms to adjust the path and avoid cones


## Size & Weight Dimensions:
4 ft  wide
~3.5 ft long, 4.5 ft with blade
 2.3 ft tall at highest point
Blade is 4.5 feet wide
~700lbs

## Materials:
1in thick Steel base plate
Aluminum extrusion framing
OSB wood and foam insulation battery box
Plastic electronics enclosure 

## Drive System:
2x AmpFlow™ gear motors
2.1 hp peak, 24 VDC
Skid steer
Chain driven
~1 m/s max speed

## Power:
4x 12V Deep Cycle Marine/RV Batteries
Lead Acid chemistry
2x wired in series which are then wired in parallel
24V output

## Electronics Housing
Plastic enclosure
Thermally Insulative
+24VDC supplied by car batteries
Components mounted with screws/3M™ Dual Lock™
Electrical connections made with Anderson Power poles, lug terminals, and screw terminals.
Cable glands used for wiring to inside of box
3D printed brackets for mounting some components

## GPS RTK (New this year!)
Precise translational positioning
Adafruit BNO055 9-Axis IMU
Gyroscope and magnetometer track rotation

## SICK 2D LiDAR 
Detects cones at the start of the run
Dijkstra's Algorithm
Start with a set of nodes we want to visit
Use Dijkstra's algorithm to find paths from one node to the next, avoiding cones
Squares with snow in them have lower weights than other squares

## Jetson Nano Orin
Runs JetpackOS and ROS 2
Remotely controllable via SSH
Arduino MEGA
Interacts with Jetson via serial communication
Sends signals to motor controllers and plow lift solenoid valves
Also sends IMU data to the Jetson

## Physical & Remote E-Stop
Power cut if either triggered
Remote E-Stop is a dead man's switch

## Stopping Distance -
2" on cement
4 rough tread tires
Slow moving (~1m/s)

## Testing:
GPS RTK - Acquire positioning coordinates from the GPS RTK system while the snowplow moves and check their accuracy - GPS RTK Data - Position Coordinates
ROS Serial Motor Control - Control Motor Speed Through Arduino over ROS Serial Outputs from Jetson Nano - Motor Speed Values - Motor Movement
LiDAR Cone Detection - Acquire positioning of all the cones on the field that the LiDAR detects, and disregard all other LiDAR data - LiDAR Data - Cone Positioning

## Failure Modes & Recovery Actions
Motor Chains - The motor chains disengage during use - Activate remote or physical E-Stop, Restore chain manually and safely
Batteries and Electronics - The batteries or electronics freeze up or exhibit reduced functionality because of cold temperatures - Engage heating system
Processors, Sensors, & Software - The robot takes an incorrect route during a run due to processing or sensing error - Activate remote or physical E-Stop, Terminate the path execution via SSH, Return robot to start point, Make any adjustments, Restart guidance software

## Most software can be tested by pushing the Lidar around on a cart and logging data
Requires fewer people than a full driving test
Allowed us to test before the full robot is operational
Perform driving tests in the small parking lot next to the building
Mostly flat and we have permission to use it
Use freight elevator to get from the workshop to the test site 

## Integrating new GPS system
Transitioning to ROS2
Transitioning to new computer (Jetson Nano Orin)
Robust but unwieldy existing robot frame
Integrating new plow lift mechanism
