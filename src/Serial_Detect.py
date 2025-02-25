import serial
import time
for i in range(10):
    try:
        arduino = serial.Serial(port='COM'+str(i), baudrate=115200,timeout=3)
    except:
        print("done with "+str(i))
        continue
    read=str(arduino.read())
    print(read)
    if '0' in read:
        print(str(i)+" is RC!")
        arduino.write('A')
    if '1' in read:
        print(str(i)+" is IMU!")
        arduino.write('A'.encode('utf-8'))
    print("done with "+str(i))
