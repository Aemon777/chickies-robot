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
        print(throttle,steering,secondaryThrottle,secondarySteering,switchA,switchB)
            
    except KeyboardInterrupt:
        n = False
        
serialPort.close()