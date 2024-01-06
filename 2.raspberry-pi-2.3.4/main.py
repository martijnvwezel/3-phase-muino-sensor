
from veml6030 import VEML6030
import smbus
from time import sleep

bus = smbus.SMBus(1)  # For Raspberry Pi 
sensor = VEML6030(bus)

while True:
    print(f"Light value: {sensor.read_light()}")
    sleep(100)