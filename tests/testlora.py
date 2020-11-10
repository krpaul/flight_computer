import busio
import board
from digitalio import DigitalInOut, Direction, Pull
from adafruit_tinylora.adafruit_tinylora import TTN, TinyLoRa
from cayennelpp import LppFrame

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = DigitalInOut(board.CE1)
irq = DigitalInOut(board.D22)
rst = DigitalInOut(board.D25)

devaddr = bytearray([0x26, 0x02, 0x1F, 0x2F])
nwkey = bytearray([0x4B, 0x0D, 0x9E, 0xB1, 0x06, 0x3F, 0x33, 0xE3, 0x2B, 0x4A, 0xFE, 0x50, 0x5E, 0x12, 0xBC, 0x89])
app = bytearray([ 0xEE, 0x9C, 0xBD, 0xCC, 0x79, 0x95, 0xC3, 0x52, 0x2F, 0x91, 0xD4, 0x82, 0xA9, 0x02, 0x00, 0x0C])

ttn_config = TTN(devaddr, nwkey, app, country='US')
lora = TinyLoRa(spi, cs, irq, rst, ttn_config,channel=0)
lora.set_datarate("SF10BW125")

frame = LppFrame()

frame.add_gps(0,50.5434, 4.4069, 100.98)
frame.add_temperature(0,25.0)
frame.add_gyrometer(0,50,10,2)
frame.add_barometer(0,100.0)
frame.add_accelerometer(0,0,0,-9.81)

lora.send_data(frame.bytes(), len(frame.bytes()), lora.frame_counter)
