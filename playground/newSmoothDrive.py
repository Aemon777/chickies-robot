import serial
import gpiozero as gpio
from time import sleep
import yaml

with open("comports.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        serialPort=serial.Serial(data['RADIO_PORT'],data['RADIO_BAUDRATE'])
    except yaml.YAMLError as exc:
        print(exc)

leftPwm = gpio.PWMLED(13)
leftDir = gpio.LED(6)
rightPwm = gpio.PWMLED(12)
rightDir = gpio.LED(5)

leftVal = 0.00
rightVal = 0.00
n = True

while n:
    try:
        read = serialPort.readline().decode('utf-8')
        data = read.split(',')
        throttle = int(data[2])
        steering = int(data[0])
        secondaryThrottle = int(data[1])
        secondarySteering = int(data[3])
        switchA = (int(data[4]) > 50)
        switchB = round(int(data[5]) / 50)
        if switchA:
            throttle = -throttle
        
        speedModifier = throttle
        leftModifier = int(steering*1.5) - 75
        rightModifier = -leftModifier
        left = speedModifier + leftModifier
        right = speedModifier + rightModifier
        if left > 100:
            left = 100
        if left < -100:
            left = -100
        if right > 100:
            right = 100
        if left < -100:
            left = -100
        
        leftVal = round(0.75*leftVal + 0.25*left)
        rightVal = round(0.75*rightVal + 0.25*right)
        
        if leftVal <= 2 and leftVal >= -2:
            leftVal = 0
        if rightVal <= 2 and rightVal >= -2:
            rightVal = 0

        if leftVal < 0 and leftDir.is_active:
            leftDir.off()
        elif leftVal > 0 and not leftDir.is_active:
            leftDir.on()
        if rightVal < 0 and rightDir.is_active:
            rightDir.off()
        elif rightVal > 0 and not rightDir.is_active:
            rightDir.on()

        if leftPwm.value != abs(leftVal/100.00):
            leftPwm.value = abs(leftVal/100.00)
        if rightPwm.value != abs(rightVal/100.00):
            rightPwm.value = abs(rightVal/100.00)
        
        print(leftVal,rightVal)
            
    except KeyboardInterrupt:
        n = False
        
serialPort.close()
