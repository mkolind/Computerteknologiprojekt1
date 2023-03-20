# Code for RGB-sensor,
# Se Week 3 for opsætning, Der skal tilføjes brugere og alt muligt lort.
# Man skal bare lave en instance af RGBsensor og kalde 

from smbus2 import SMBus
import time

class RGBsensor():
    def __init__(self):
        # Get I2C bus
        self.bus = smbus2.SMBus(1)

        time.sleep(0.5)

        # ISL29125 address, 0x44(68) (Standarden)
        # Select configuation-1 register: 0x01(01)
        # 0b00101(5) Operation: RGB, Range: 375 lux, Res: 16 Bits
        self.bus.write_byte_data(0x44, 0x01, 0x05)

        print("Reading colour values and displaying them in a new window\n")
        print("data til mathias: ", self.bus.read_byte_data(0x44, 0x01))

    # Reads reading from RGB sensor
    # return as tuple (r, g, b)
    def getColour(self) -> tuple:
        # Selects the right registers
        data = self.bus.read_i2c_block_data(0x44, 0x09, 6)

        green = data[1] # Green high byte
        red = data[3]   # Red high byte
        blue = data[5]  # Blue high byte
        return (data[3], data[1], data[5])