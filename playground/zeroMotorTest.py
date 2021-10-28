import gpiozero as gpio
from time import sleep

leftPwm = gpio.PWMLED(13)
leftDir = gpio.LED(6)
rightPwm = gpio.PWMLED(12)
rightDir = gpio.LED(5)

while True:
    leftDir.on()
    rightDir.on()
    for duty in range(51):
        leftPwm.value = duty
        sleep(0.05)
    sleep(0.5)
    for duty in range(51,-1,-1):
        leftPwm.value = duty
        sleep(0.05)
    sleep(0.5)
    
    for duty in range(0,76,1):
        rightPwm.value = duty
        sleep(0.05)
    sleep(0.5)
    for duty in range(75,-1,-1):
        rightPwm.value = duty
        sleep(0.05)
    sleep(0.5)

    leftDir.off()
    rightDir.off()
    for duty in range(51):
        leftPwm.value = duty
        sleep(0.05)
    sleep(0.5)
    for duty in range(51,-1,-1):
        leftPwm.value = duty
        sleep(0.05)
    sleep(0.5)
    
    for duty in range(0,76,1):
        rightPwm.value = duty
        sleep(0.05)
    sleep(0.5)
    for duty in range(75,-1,-1):
        rightPwm.value = duty
        sleep(0.05)
    sleep(0.5)