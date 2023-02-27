import smbus2
import time

# Get I2C bus
bus = smbus2.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")
print("data til mathias: ", bus.read_byte_data(0x44,0x01))
print("threshhold low: ", bus.read_byte_data(0x44,0x04))
print("threshold high: ", bus.read_byte_data(0x44,0x06))
print("threshold high high", bus.read_byte_data(0x44, 0x07))


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
        greenLow = bus.read_byte_data(0x44, 0x09)
        
        gH = bus.read_byte_data(0x44, 0x0A)
        
        # Red read:
        redLow = bus.read_byte_data(0x44, 0x0B)
         
        redHigh = bus.read_byte_data(0x44, 0x0C)
        
        # Blue read:
        blueLow = bus.read_byte_data(0x44, 0x0D)
         
        blueHigh = bus.read_byte_data(0x44, 0x0E)
        
        # Convert the data to green, red and blue int values
        # Kalibrere når der er mørkt. ik brug low da det er for random.
        green =(gH)-45
        red = (redHigh)
        blue =(blueHigh)+20
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        print("RGB(%d %d %d)\n" % (red, green, blue))
        if(blue > green and blue > red):
            print("blue!")
        elif(green > red and green > blue):
            print("green!")
        elif(red > green and red > blue):
            print("red!")
        time.sleep(2)
        
getAndUpdateColour()
