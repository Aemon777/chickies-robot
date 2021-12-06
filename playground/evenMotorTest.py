import gpiozero as gpio
from time import sleep

leftPwm = gpio.PWMLED(13)
leftDir = gpio.LED(6)
rightPwm = gpio.PWMLED(12)
rightDir = gpio.LED(5)

leftDir.on()
rightDir.on()
for duty in range(0,101,10):
    leftPwm.value = duty/100 #/1.15 #provide duty cycle in the range 0-100
    rightPwm.value = duty/100
    sleep(0.01)
sleep(2)
for duty in range(100,-1,-10):
    leftPwm.value = duty/100 #/1.15
    rightPwm.value = duty/100
    sleep(0.01)
