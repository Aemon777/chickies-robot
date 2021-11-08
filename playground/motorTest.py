import gpiozero as gpio
from time import sleep
print("past imports")

leftPwm = gpio.PWMLED(13)
leftDir = gpio.LED(6)
rightPwm = gpio.PWMLED(12)
rightDir = gpio.LED(5)

while True:
    leftDir.on()
    rightDir.on()
    for duty in range(0,91,1):
        leftPwm.value = duty/100 #provide duty cycle in the range 0-100
        rightPwm.value = duty/100
        sleep(0.05)
    sleep(3)
    for duty in range(91,-1,-1):
        leftPwm.value = duty/100
        rightPwm.value = duty/100
        sleep(0.05)
    