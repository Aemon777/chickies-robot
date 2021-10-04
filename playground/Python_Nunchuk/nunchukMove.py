#*****************************
#don't set pwm value above 75.
#*****************************
import RPi.GPIO as GPIO
import time
import board
import adafruit_nunchuk
import motorInterface

nc =  adafruit_nunchuk.Nunchuk(board.I2C())

pwmLeftPin = 13   #left
dirLeftPin = 6
pwmRightPin = 12
dirRightPin = 5

GPIO.setwarnings(False)			#disable warnings
#GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(pwmLeftPin,GPIO.OUT)
GPIO.setup(dirLeftPin,GPIO.OUT)
GPIO.setup(pwmRightPin,GPIO.OUT)
GPIO.setup(dirRightPin,GPIO.OUT)
pwmLeft = GPIO.PWM(pwmLeftPin,1000)		#create PWM instance with frequency
pwmRight = GPIO.PWM(pwmRightPin,1000)		#create PWM instance with frequency
pwmLeft.start(0)				#start PWM of required Duty Cycle 
pwmRight.start(0)
directions = ((0, True, 0, True))
while True:
    x, y = nc.joystick
    print("joystick = {},{}".format(x, y))
    turning = abs(x-130) > 10
    turndir = 'right'
    turnrat = 0
    if turning:
        turndir = 'right' if x-130 > 0 else 'left'
        turnrat = int(abs(x-130)*3/4)
        print('Turn direction: ')
        print(turndir)
        print('  Turn rate: ')
        print(turnrat)
    moving = abs(y-129) > 10
    movdir = True
    movspd = 0
    if moving:
        movdir = True if y-129 > 0 else False
        movspd = int(abs(y-129)*75/98)
        print(' Move speed: ')
        print(movspd)
        print(' Move direction: ')
        print(movdir)
    if nc.buttons.Z:
        movspd = 0
        turnrat = 0
    #drive will return a 4-Tuple containing (pwmLeft, dirLeft, pwmRight, dirRight)
    directions = motorInterface.drive(movspd, movdir, turnrat, turndir, directions)
    print(directions)
    GPIO.output(dirLeftPin,directions[1])
    GPIO.output(dirRightPin,directions[3])
    pwmLeft.ChangeDutyCycle(directions[0])
    pwmRight.ChangeDutyCycle(directions[2]) #provide duty cycle in the range 0-100
    time.sleep(0.05)
