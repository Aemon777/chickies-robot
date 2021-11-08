import gpiozero as gpio
from time import sleep

leftPwm = gpio.PWMLED(13)
leftDir = gpio.LED(6)
rightPwm = gpio.PWMLED(12)
rightDir = gpio.LED(5)

leftDir.on()
rightDir.on()

for duty in range(0, 91, 1):
    leftPwm.value = duty/100
    sleep(0.01)

sleep(0.5)
for duty in range(91,-1,-1):
    leftPwm.value = duty/100
    sleep(0.01)

sleep(0.5)
    
for duty in range(0,91,1):
    rightPwm.value = duty/100
    sleep(0.01)

sleep(0.5)
for duty in range(91,-1,-1):
    rightPwm.value = duty/100
    sleep(0.01)
sleep(0.5)

leftDir.off()
rightDir.off()
for duty in range(0, 91, 1):
    leftPwm.value = duty/100
    sleep(0.01)

sleep(0.5)
for duty in range(91,-1,-1):
    leftPwm.value = duty/100
    sleep(0.01)

sleep(0.5)

for duty in range(0,91,1):
    rightPwm.value = duty/100
    sleep(0.01)

sleep(0.5)
for duty in range(91,-1,-1):
    rightPwm.value = duty/100
    sleep(0.01)

sleep(0.5)
sleep(0.5)
