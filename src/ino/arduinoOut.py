import serial

with serial.Serial('/dev/ttyACM0', 9600, timeout=10) as ser:
    while True:
        motor = input('Enter desired motor power: ')
        ser.write(bytes(motor,'utf-8'))
