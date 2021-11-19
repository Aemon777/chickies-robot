from gpiozero import Button
button = Button(19)

n = True
while n:
	try:
		if(button.is_active == True):
			print('button pushed')
	except(KeyboardInterrupt):
		n = False

print(button.is_active)
