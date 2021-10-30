import gpiozero as gpio
from time import sleep
print("past imports")
led = gpio.LED(13)

led.on()

sleep(10)