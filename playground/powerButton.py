import os
from gpiozero import Button
button = Button(19)

n = True
while n:
	try:
		if(button.is_active == True):
			os.system('sudo shutdown now -h')
			n = False
	except(KeyboardInterrupt):
		n = False

print(button.is_active)

