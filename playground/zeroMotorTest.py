import gpiozero as gpio
from time import sleep
print("past imports")
leftPwm = gpio.PWMLED(13)
leftDir = gpio.LED(6)
rightPwm = gpio.PWMLED(12)
rightDir = gpio.LED(5)

print("declared vars")
leftDir.on()
rightDir.on()
print("turned on wheels")
for duty in range(0, 91, 5):
    print(duty)
    leftPwm.value = duty/100
    sleep(0.2)
print("loop 1")
sleep(0.5)
for duty in range(91,-1,-5):
    leftPwm.value = duty/100
    sleep(0.2)
print("loop 2")
sleep(0.5)
    
for duty in range(0,91,5):
    rightPwm.value = duty/100
    sleep(0.2)
print("loop 3")
sleep(0.5)
for duty in range(91,-1,-5):
    rightPwm.value = duty/100
    sleep(0.2)
print("loop 4")
sleep(0.5)
print("switching directions")
leftDir.off()
rightDir.off()
for duty in range(0, 91, 5):
    leftPwm.value = duty/100
    sleep(0.2)
print("loop 5")
sleep(0.5)
for duty in range(91,-1,-5):
    leftPwm.value = duty/100
    sleep(0.2)
print("loop 6")
sleep(0.5)

for duty in range(0,91,5):
    rightPwm.value = duty/100
    sleep(0.2)
print("loop 7")
sleep(0.5)
for duty in range(91,-1,-5):
    rightPwm.value = duty/100
    sleep(0.2)
print("loop 8")
sleep(0.5)
sleep(0.5)
