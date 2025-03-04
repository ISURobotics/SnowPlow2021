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
for i in vals:
    try:
        arduino = serial.Serial(port=prefix+str(i), baudrate=115200,timeout=3)
    except:
        continue
    read=str(arduino.read())
    if '0' in read:
        print(str(i)+" is RC!")
        rc=prefix+str(i)
        arduino.write(b'A')
    if '1' in read:
        print(str(i)+" is IMU!")
        imu=prefix+str(i)
        arduino.write(b'A')
with open("serial_ports.txt",'w') as f:
    f.write(rc+" "+imu)