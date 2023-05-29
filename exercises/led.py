from gpiozero import LED
from time import sleep

pin_number = 21 #gpio pin number

led = LED(pin_number)

while True:
    led.on()
    sleep(1)
    led.off()
    sleep(1)
