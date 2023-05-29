# coding: utf-8
import time
import RPi.GPIO as GPIO
from gpiozero import LED



# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO = 17

# Define LED GPIO pin number
LED_pinNumber = 26
led = LED(LED_pinNumber)

print("Ultrasonic Measurement")

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO,GPIO.OUT)  # Initial state as output


# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO, False)

def measure():
  # This function measures a distance
  # Pulse the trigger/echo line to initiate a measurement
    GPIO.output(GPIO_TRIGECHO, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGECHO, False)
  #ensure start time is set in case of very quick return
    start = time.time()

  # set line to input to check for start of echo response
    GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
    while GPIO.input(GPIO_TRIGECHO)==0:
        start = time.time()

  # Wait for end of echo response
    while GPIO.input(GPIO_TRIGECHO)==1:
        stop = time.time()
  
    GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO, False)

    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    time.sleep(0.1)
    return distance

def blink(userTime):
    led.on()
    time.sleep(userTime)
    led.off()
    time.sleep(userTime)

try:
    while True:
        distance = measure()
        if(distance < 18):
            print("Jeg blinker hurtigt!")
            for i in range(10):
                blink(0.1)
             
        elif(distance < 30):
            print("Jeg blinker bare")
            blink(1)
        else:
            print("Jeg blinker ikke")
            time.sleep(1)
        print ("  Distance : %.1f cm" % distance)

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()
