import serial
"""
Just a simple script to interact with the Motor Arduino for testing.
To use, connect the Arduino with the MotorAndRCArdino code on it to the USB port
and run the script. The expected format is leftSpeed|RightSpeed
example:
50|50
"""
with serial.Serial('/dev/ttyACM0', 9600, timeout=10) as ser:
    while True:
        motor = input('Enter desired motor power: ')
        ser.write(bytes(motor,'utf-8'))
