#*****************************
#don't set pwm value above 75.
#*****************************
import RPi.GPIO as GPIO
from time import sleep
import time
import board
import adafruit_nunchuk

nc = adafruit_nunchuk.Nunchuk(board.I2C())

pwmA = 33
dirA = 31
pwmB = 32
dirB = 29

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

while True:
    x, y = nc.joystick
    print("joystick = {},{}".format(x, y))
    turning = abs(x-130) > 5
    turndir = ''
    turnrat = 0
    if turning:
        turndir = 'right' if x-130 > 0 else 'left'
        turnrat = abs(x-130)
    moving = abs(y-129) > 5
    movdir = True
    movspd = 0
    if moving:
        movdir = True if y-129 > 0 else False
        movspd = abs(y-129)*75/98
    if nc.buttons.Z:
        movspd = 75
    if nc.buttons.C:
        print("button C")
    if nc.buttons.Z:
        print("button Z")
    GPIO.output(dirA,True)
    GPIO.output(dirB,True)
    for duty in range(0,76,1):
        #pwma.ChangeDutyCycle() #provide duty cycle in the range 0-100   //Max 75 for this implementation
        pwmb.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        sleep(0.05)
    sleep(0.1)