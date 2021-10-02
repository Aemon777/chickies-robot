import RPi.GPIO as GPIO
from time import sleep

pwmA = 33
dirA = 31
pwmB = 32
dirB = 29

GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(pwmA,GPIO.OUT)
GPIO.setup(dirA,GPIO.OUT)
GPIO.setup(pwmB,GPIO.OUT)
GPIO.setup(dirB,GPIO.OUT)
pwma = GPIO.PWM(pwmA,1000)		#create PWM instance with frequency
pwmb = GPIO.PWM(pwmB,1000)		#create PWM instance with frequency
pwma.start(0)				#start PWM of required Duty Cycle 
pwmb.start(0)

while True:
    GPIO.output(dirA,True)
    GPIO.output(dirB,True)
    for duty in range(0,76,1):
        pwma.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        sleep(0.05)
    sleep(0.5)
    for duty in range(75,-1,-1):
        pwma.ChangeDutyCycle(duty)
        sleep(0.05)
    sleep(0.5)
    
    for duty in range(0,76,1):
        pwmb.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        sleep(0.05)
    sleep(0.5)
    for duty in range(75,-1,-1):
        pwmb.ChangeDutyCycle(duty)
        sleep(0.05)
    sleep(0.5)

    GPIO.output(dirA,False)
    GPIO.output(dirB,False)
    for duty in range(0,76,1):
        pwma.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        sleep(0.05)
    sleep(0.5)
    for duty in range(75,-1,-1):
        pwma.ChangeDutyCycle(duty)
        sleep(0.05)
    sleep(0.5)
    
    for duty in range(0,76,1):
        pwmb.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
        sleep(0.05)
    sleep(0.5)
    for duty in range(75,-1,-1):
        pwmb.ChangeDutyCycle(duty)
        sleep(0.05)
    sleep(0.5)