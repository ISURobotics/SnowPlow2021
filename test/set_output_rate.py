import serial


def set_output_rate(port,rate: int):

    assert rate >= 1 and rate <= 20
    rate_ms = int(1000 / rate)

    msg = bytearray([0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (rate_ms & 0xFF),
                        (rate_ms >> 8), 0x01, 0x00, 0x00, 0x00])
    ck_a = 0
    ck_b = 0
    for byte in msg[2:]:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF

    msg.extend([ck_a, ck_b])
    port.write(msg)


if __name__ == '__main__':
    port = serial.Serial('/dev/ttyACM1', baudrate=9600, timeout=1)
    rate = int(input('Input a rate from 1 to 20 in Hz: '))
    set_output_rate(port, rate)
    port.close()


