import serial
import gpiozero as gpio
from time import sleep
import yaml
import os



n = True
while n:
    try:
        with open("/home/ubuntu/chickies-robot/playground/comports.yaml", "r") as stream:
            try:
                data = yaml.safe_load(stream)
                serialPort=serial.Serial(data['TEENSY_PORT'],data['TEENSY_BAUDRATE'])
                os.system('ls -l /dev/Teensy')
            except yaml.YAMLError as exc:
                print(exc)
        sleep(10)
    
    except KeyboardInterrupt:
        n = False

serialPort.close()
