import smbus2
import time

# Get I2C bus
bus = smbus2.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

def binToInt(l: list) -> int:
    sum = 0
    for i, a in enumerate(l):
        sum += a * (1 << (i))
    return sum

def getAndUpdateColour():
    while True:
	# Read the data from the sensor
        # Insert code here
        
        # Green read:
        greenLow = bus.read_i2c_block_data(0x44, 0x09, 8)
        
        greenHigh = bus.read_i2c_block_data(0x44, 0x0A, 8)
        
        # Red read:
        redLow = bus.read_i2c_block_data(0x44, 0x0B, 8)
         
        redHigh = bus.read_i2c_block_data(0x44, 0x0C, 8)
        
        # Blue read:
        blueLow = bus.read_i2c_block_data(0x44, 0x0D, 8)
         
        blueHigh = bus.read_i2c_block_data(0x44, 0x0E, 8)
        
        # Convert the data to green, red and blue int values
        # Insert code here
        green = binToInt(greenLow) + (binToInt(greenHigh) << 8)
        red = binToInt(redLow) + (binToInt(redHigh) << 8)
        blue = binToInt(blueLow) + (binToInt(blueHigh) << 8)
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        print("RGB(%d %d %d)\n" % (red, green, blue))
        
        time.sleep(2) 
        
getAndUpdateColour()
