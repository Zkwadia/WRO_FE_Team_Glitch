import os

os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')
from Servo import Servo


ser = Servo(8)

if __name__ == "__main__":
	try:
		angle = int(input("Angle = "))
		print("Hello")
		ser.setAngle(angle)
	except Exception as e:
		print(f"Exception: {e}")

