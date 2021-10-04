#*****************************
#don't set pwm value above 75.
#*****************************
import RPi.GPIO as GPIO
import time
import board
import adafruit_nunchuk
import motorInterface

nc =  adafruit_nunchuk.Nunchuk(board.I2C())

pwmA = 13   #left
dirA = 6
pwmB = 12
dirB = 5

GPIO.setwarnings(False)			#disable warnings
#GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(pwmA,GPIO.OUT)
GPIO.setup(dirA,GPIO.OUT)
GPIO.setup(pwmB,GPIO.OUT)
GPIO.setup(dirB,GPIO.OUT)
pwma = GPIO.PWM(pwmA,1000)		#create PWM instance with frequency
pwmb = GPIO.PWM(pwmB,1000)		#create PWM instance with frequency
pwma.start(0)				#start PWM of required Duty Cycle 
pwmb.start(0)

# Front-back key: False is forward
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
    movdir = False
    movspd = 0
    if moving:
        movdir = False if y-129 > 0 else True
        movspd = int(abs(y-129)*75/98)
        print(' Move speed: ')
        print(movspd)
        print(' Move direction: ')
        print(movdir)
    #if nc.buttons.Z:
     #   movspd = 0
      #  turnrat = 0
    directions = motorInterface.drive(movspd, movdir, turnrat, turndir)
    print(directions)
    GPIO.output(dirA,directions[1])
    GPIO.output(dirB,directions[3])
    pwma.ChangeDutyCycle(directions[0])
    pwmb.ChangeDutyCycle(directions[2]) #provide duty cycle in the range 0-100
    time.sleep(0.1)
