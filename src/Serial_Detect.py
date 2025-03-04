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