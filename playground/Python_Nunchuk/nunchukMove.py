#*****************************
#don't set pwm value above 75.
#*****************************
import RPi.GPIO as GPIO
import time
import board
import adafruit_nunchuk
import motorInterface

nc = adafruit_nunchuk.Nunchuk(board.I2C())

pwmLeftPin = 13   #left
dirLeftPin = 6
pwmRightPin = 12
dirRightPin = 5

GPIO.setwarnings(False)			#disable warnings
GPIO.setup(pwmLeftPin,GPIO.OUT)
GPIO.setup(dirLeftPin,GPIO.OUT)
GPIO.setup(pwmRightPin,GPIO.OUT)
GPIO.setup(dirRightPin,GPIO.OUT)
pwmLeft = GPIO.PWM(pwmLeftPin,1000)		#create PWM instance with frequency
pwmRight = GPIO.PWM(pwmRightPin,1000)		#create PWM instance with frequency
pwmLeft.start(0)				#start PWM of required Duty Cycle 
pwmRight.start(0)
directions = ((0, False, 0, False))
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
    moving = abs(y-129) > 10
    movdir = False
    movspd = 0
    if moving:
        movdir = False if y-129 > 0 else True
        movspd = int(abs(y-129)*75/98)
    #if nc.buttons.Z:
     #   movspd = 0
      #  turnrat = 0
    #drive will return a 4-List containing (pwmLeft, dirLeft, pwmRight, dirRight)
    directions = motorInterface.drive(movspd, movdir, turnrat, turndir, directions)
    print(directions)
    GPIO.output(dirLeftPin,directions[1])
    GPIO.output(dirRightPin,directions[3])
    pwmLeft.ChangeDutyCycle(directions[0])
    pwmRight.ChangeDutyCycle(directions[2]) #provide duty cycle in the range 0-100
    time.sleep(1)
