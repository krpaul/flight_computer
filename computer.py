from lib.imu_lib import MinIMU_v5_pi

import time
import serial
import pynmea2 as nmea
import adafruit_bme280
import board
import busio

def main():
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
    bme = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    bme.sea_level_pressure = 103.0 # Calgary's local pressure ASL
      
    while True:
        # gather IMU data

        # update IMU Angle 30 times
        for _ in range(30): imu.updateAngle(); time.sleep(0.004)

        accel = imu.readAccelerometer()
        gyr = imu.readGyro()
        magn = imu.readMagnetometer()

        imu_data = {
            'angle': {
                'x': imu.prevAngle[0][0],
                'y': imu.prevAngle[0][1],
                'z': imu.prevAngle[0][2],
            },
            'acceleration': {
                'x': accel[0],
                'y': accel[1],
                'z': accel[2],
            },
            'gyro': {
                'x': gyr[0],
                'y': gyr[1],
                'z': gyr[2],
            },
            'mag': {
                'x': magn[0],
                'y': magn[1],
                'z': magn[2],
            }
        }
        
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

        gps_data = {
            'lat': lat,
            'lng': lng,
            'alt': alt
        }

        # gather barometer/temp data
        barometer_data = {
            'temp': bme.temperature,
            'humidity': bme.humidity,
            'pressure': bme.pressure,
            'baro_alt': bme.altitude
        }
        
        # entire data packet of all gathered info
        data = {**imu_data, **gps_data, **barometer_data}

        print(data)

        time.sleep(5)

if __name__ == "__main__":
    main()
