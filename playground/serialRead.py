import serial

serialPort=serial.Serial('/dev/ttyACM0',115200)

n = True

while n:
    try:
        read = serialPort.readline().decode('utf-8')
        #data = read.split(',')
        print(read)
    except KeyboardInterrupt:
        n = False
        
serialPort.close()
