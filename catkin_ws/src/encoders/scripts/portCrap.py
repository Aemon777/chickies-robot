import serial
import gpiozero as gpio
from time import sleep
import yaml
import os

with open("/home/ubuntu/chickies-robot/playground/comports.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        serialPort=serial.Serial(data['TEENSY_PORT'],data['TEENSY_BAUDRATE'])
        os.system('ls -l /dev/Teensy')
    except yaml.YAMLError as exc:
        print(exc)

n = True
exceptionCounter = 0
while n:
    try:
        read = serialPort.readline().decode('utf-8')
        data = read.split(',')
        
        print(data, exceptionCounter)
    
    except KeyboardInterrupt:
        n = False
        
    except:
        exceptionCounter = exceptionCounter + 1
        print('exception hit')
        sleep(5)
        serialPort.close()
        sleep(5)
        with open("/home/ubuntu/chickies-robot/playground/comports.yaml", "r") as stream:
            try:
                data = yaml.safe_load(stream)
                serialPort=serial.Serial(data['TEENSY_PORT'],data['TEENSY_BAUDRATE'])
                os.system('ls -l /dev/Teensy')
            except yaml.YAMLError as exc:
                print(exc)
        sleep(5)

serialPort.close()
