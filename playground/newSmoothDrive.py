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
        
        speedModifier = throttle
        leftModifier = int(steering*1.5) - 75
        rightModifier = -leftModifier
        left = speedModifier + leftModifier
        if(left > 100):
            left = 100
        right = speedModifier + rightModifier
        if(right > 100):
            right = 100
        
        #print(left, right)
        
        if(left < 0):
            leftDir.off()
        else:
            leftDir.on()
            
        if(right < 0):
            rightDir.off()
        else:
            rightDir.on()
        
        leftVal = round(0.7*leftVal + 0.3*left)
        rightVal = round(0.7*rightVal + 0.3*right)
        
        if(leftVal <= 2):
            leftVal = 0
        if(rightVal <= 2):
            rightVal = 0
        leftPwm.value = leftVal/100.00
        rightPwm.value = rightVal/100.00
        
        print(leftVal,rightVal)
            
    except KeyboardInterrupt:
        n = False
        
serialPort.close()
