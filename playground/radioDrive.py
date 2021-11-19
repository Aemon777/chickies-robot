import serial
import gpiozero as gpio
from time import sleep

serialPort=serial.Serial('/dev/ttyACM0',115200)

leftPwm = gpio.PWMLED(13)
leftDir = gpio.LED(6)
rightPwm = gpio.PWMLED(12)
rightDir = gpio.LED(5)

n = True

while n:
    try:
        read = serialPort.readline().decode('utf-8')
        data = read.split(',')
        left = int(data[0])
        right = int(data[1])
        if(left < 0):
            leftDir.off()
            left = -left
        else:
            leftDir.on()
            
        if(right < 0):
            rightDir.off()
            right = -right
        else:
            rightDir.on()
        leftPwm.value = left/100.00
        rightPwm.value = right/100.00
        
        print(left,right)
            
    except KeyboardInterrupt:
        n = False
        
serialPort.close()
