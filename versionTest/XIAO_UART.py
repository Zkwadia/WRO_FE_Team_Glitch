
import time
import os

os.system("sudo pkill pigpiod")
time.sleep(1)
os.system("sudo pigpiod -x -1")     # -x -1 = allow every GPIO
time.sleep(2)                       # give the daemon time to come up

import pigpio
import serial
import math


reset_pin = 6


# Open the serial port
pwm = pigpio.pi()
if not pwm.connected:
    print("Could not connect to pigpio daemon")
    exit(1)
pwm.set_mode(reset_pin, pigpio.OUTPUT)
ser = serial.Serial('/dev/XIAO_USB', 115200)
#ser.flush()
#data = input()
'''time.sleep(1)
print(f"resetting.....")
pwm.write(reset_pin, 0)
print(f"status:{pwm.read(reset_pin)}")
time.sleep(0.5)
pwm.write(reset_pin, 1)
print(f"status:{pwm.read(reset_pin)}")

time.sleep(1)
ser.reset_input_buffer()'''  # flush boot garbage

print(f"reset complete!")
#rad = str(math.radians(float(data)))
#ser.write(b"1")
print("Command sent: b'1'")
#ser.flush()
head = 0.0
counts = 0



def reset_esp32():
    print("Sending software reset to ESP32...")
    ser.write(b"R")
    time.sleep(1)              # wait for ESP32 to fully reboot and BNO085 to init
    ser.reset_input_buffer()   # flush boot garbage
    print("ESP32 reset complete")


reset_esp32()

while True:
    # Rea	d data from ESP32
	
	#ser.write(rad.encode(
	esp_data = ser.readline().decode('utf-8', errors='ignore').strip()
	esp_data = esp_data.split()
	if len(esp_data) >= 2:
		try:
			head = float(esp_data[0])
			counts = int(esp_data[1])
		except ValueError:
			print(f"⚠️ Malformed numbers: {esp_data}")
	else:
		print(f"⚠️ Incomplete frame: {esp_data!r}")
	#esp_data = esp_data + 1   
	# if esp_data.startswith("X: "):
	#x, y = esp_data.split(" ")
	#x = float(x.split("")[1])
	#y = float(y)

	#print(f"Received X: {x}, Y: {y}")
	print(f" head: {head:.2f} , counts: {counts} status: {pwm.read(reset_pin)}")
	#print(f" ESP: {esp_data}, type:{type(esp_data)}")

