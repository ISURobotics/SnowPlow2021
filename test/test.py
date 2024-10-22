from ublox_gps import UbloxGps
import serial

port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
gps = UbloxGps(port)

def run():

    try:
        print("Listening for UBX messages.")
        while True:
            try:
                coords = gps.geo_coords()
                print(coords.lon, coords.lat)
            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()
    

if __name__ == '__main__':
    run()
