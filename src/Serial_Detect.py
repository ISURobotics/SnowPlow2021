#3/3/2025
# Run this on jetson startup
#Tests a range of ports(0-10 on windows, 0-4 on linux)
# Reads 12 characters(twice the length of the longest input characteristic message, "IAMIMU")
# Sees if the characteristic message of a detected arduino is present in the serial monitor
# If so, it is an arduino and we record that, if not it is the gps
# Output these to a file for later use
# -

import serial
import time
#Use this on jetson
# vals=range(4)
# prefix='/dev/ttyACM'
#Use this for windows testing
vals=range(10)
prefix='COM'
rc='no'
imu='no'
gps='no'
for i in vals:
    try:
        arduino = serial.Serial(port=prefix+str(i), baudrate=115200,timeout=3)
    except:
        continue
    read=str(arduino.read(12))
    print(read)
    if 'IAMRC' in read:
        print(str(i)+" is RC!")
        rc=prefix+str(i)
        arduino.write(b'A')
    elif 'IAMIMU' in read:
        print(str(i)+" is IMU!")
        imu=prefix+str(i)
        arduino.write(b'A')
    else:
        print(str(i)+" is GPS!")
        gps=prefix+str(i)
with open("serial_ports.txt",'w') as f:
    f.write(rc+" "+imu+" "+gps)
f.write("RC, IMU, GPS")