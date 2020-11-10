from lib.imu_lib import MinIMU_v5_pi

import time
import serial
import pynmea2 as nmea
import adafruit_mpl3115a2
import board
import busio
import requests as r
import json
from digitalio import DigitalInOut, Direction, Pull
from adafruit_tinylora.adafruit_tinylora import TTN, TinyLoRa
from cayennelpp import LppFrame

def main():
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    cs = DigitalInOut(board.CE1)
    irq = DigitalInOut(board.D22)
    rst = DigitalInOut(board.D25)
    i2c = busio.I2C(board.SCL, board.SDA)

    devaddr = bytearray([0x26, 0x02, 0x1F, 0x2F])
    nwkey = bytearray([0x4B, 0x0D, 0x9E, 0xB1, 0x06, 0x3F, 0x33, 0xE3, 0x2B, 0x4A, 0xFE, 0x50, 0x5E, 0x12, 0xBC, 0x89])
    app = bytearray([ 0xEE, 0x9C, 0xBD, 0xCC, 0x79, 0x95, 0xC3, 0x52, 0x2F, 0x91, 0xD4, 0x82, 0xA9, 0x02, 0x00, 0x0C])

    ttn_config = TTN(devaddr, nwkey, app, country='US')
    lora = TinyLoRa(spi, cs, irq, rst, ttn_config,channel=0)
    lora.set_datarate("SF10BW125")

    # serial object for gps
    ser = serial.Serial(
        port='/dev/ttyS0', 
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

    # imu obj 
    imu = MinIMU_v5_pi()

    # barometer objs
    i2c = busio.I2C(board.SCL, board.SDA)
    mpl = adafruit_mpl3115a2.MPL3115A2(i2c)
    mpl.sea_level_pressure = 102250 # Calgary's local pressure ASL

    while True:
        # gather IMU data
        frame = LppFrame()

        # update IMU Angle 30 times
        for _ in range(30): imu.updateAngle(); time.sleep(0.004)

        accel = imu.readAccelerometer()
        gyr = imu.readGyro()
        magn = imu.readMagnetometer()

        frame.add_accelerometer(0, round(imu.prevAngle[0][0], 1), round(imu.prevAngle[0][1], 1), round(imu.prevAngle[0][2], 1))
        frame.add_gyrometer(0, round(gyr[0], 1), round(gyr[1], 1), round(gyr[2], 1))

	# Add generics on channels 1, 2, and 3 for magnetometer data
        frame.add_generic(1, round(magn[0], 1))
        frame.add_generic(2, round(magn[1], 1))
        frame.add_generic(3, round(magn[2], 1))

	# Add generics on channels 4, 5, 6 for angle data
        frame.add_generic(4, round(imu.prevAngle[0][0], 1))
        frame.add_generic(5, round(imu.prevAngle[0][1], 1))
        frame.add_generic(6, round(imu.prevAngle[0][2], 1))

        # gather gps data
        lat = 0.0
        lng = 0.0
        alt = 0.0

        line = "" 
        try: 
            line = str(ser.readline(), 'utf-8').strip()
        except UnicodeDecodeError: 
            pass
        else: 
            if "GPGGA" in line:
                data = nmea.parse(line)
                lat = data.latitude
                lng = data.longitude
                alt = data.altitude

        frame.add_gps(0, lat, lng, alt)

        # gather barometer/temp data
	# Channel 1: temp
	# Channel 2: pressure
	# Channel 3: alt

        frame.add_barometer(1, round(mpl.temperature, 2))
        frame.add_barometer(2, round(mpl.pressure, 2))
        frame.add_barometer(3, round(mpl.altitude, 2))
	
        byt = frame.bytes()
        lora.send_data(byt, len(byt), lora.frame_counter)
        print(frame)
        #time.sleep(5)

if __name__ == "__main__":
    main()
