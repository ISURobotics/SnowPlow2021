import serial
from serial import SerialException
import time
try:
    ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
except SerialException:
    ser=serial.Serial('/dev/ttyACM1',baudrate=115200)
while True:
    ser.write((str(-40) + "|" + str(-40)+'\n').encode('utf-8'))
    time.sleep(5)