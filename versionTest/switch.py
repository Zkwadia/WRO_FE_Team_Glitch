import pigpio
import os
import time
os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")
time.sleep(1)

pwm = pigpio.pi()
button = False
prev_state = 0
pwm.set_mode(6, pigpio.INPUT)
pwm.set_pull_up_down(6, pigpio.PUD_DOWN)

toggle = False
while pwm.read(6) == 1:
	print("make sure the toggle is off")
toggle = False
while True and not toggle:
	if(pwm.read(6) == 1):
		button = True
		print(f"high")
	else:
		button = False
		print(f"low")
	#print(f" button : {button}")

