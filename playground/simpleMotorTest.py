import gpiozero as gpio
from time import sleep
print("past imports")
leftPwm = gpio.PWMLED(13)
leftDir = gpio.LED(6)

leftDir.on()
leftPwm.value = 0.5

sleep(1)
leftPwm.value = 0
