import serial
import gpiozero as gpio
from time import sleep
import yaml

with open("/home/ubuntu/chickies-robot/resources/comports.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        serialPort=serial.Serial(data['TEENSY_PORT'],data['TEENSY_BAUDRATE'])
    except yaml.YAMLError as exc:
        print(exc)

n = True

while n:
    try:
        read = serialPort.readline().decode('utf-8')
        data = read.split(',')
        
        print(data)
            
    except KeyboardInterrupt:
        n = False
        
serialPort.close()