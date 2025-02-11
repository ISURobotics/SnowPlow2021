from ublox_gps import UbloxGps
import serial
import math

port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
gps = UbloxGps(port)


def get_global_x_y():
    coords = gps.geo_coords()
    lat = coords.lat
    lon = coords.lon

    lat_factor = 111132.92 - 559.82*math.cos(2*math.radians(lat)) + 1.175*math.cos(4*math.radians(lat)) - 0.0023*math.cos(6*math.radians(lat))
    lon_factor = 111412*math.cos(math.radians(lat)) - 93.5*math.cos(3*math.radians(lat)) + 0.118*math.cos(5*math.radians(lat))

    global_y = lat_factor * lat
    global_x = lon_factor * lon

    return global_x, global_y



def run():

    try:
        print("Listening for UBX messages.")
        origin_x, origin_y = get_global_x_y()
        while True:
            try:
                x, y = get_global_x_y()

                x -= origin_x       
                y -= origin_y
                print(x, y)
            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()
    

if __name__ == '__main__':
    run()
