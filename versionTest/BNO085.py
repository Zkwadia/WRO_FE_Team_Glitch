import time
import board
import busio
from math import atan2, sqrt, pi
import adafruit_tcs34725
import math
import colorsys
class IMUandColorSensor:
	def __init__(self, scl, sda, frequency=400000):
		self.i2c1 = busio.I2C(scl, sda)
		self.prev_imu = 0

		#self.sensor = adafruit_tcs34725.TCS34725(self.i2c1)
		#self.sensor.gain = 60
		#self.color_rgb = [0, 0, 0]
		#self.r_norm = 0
		#self.g_norm = 0
		#self.b_norm = 0
	

	def get_color(self):
		#time.sleep(0.01)

		self.color_rgb = self.sensor.color_rgb_bytes
		total = (self.color_rgb[0] + self.color_rgb[1]+ self.color_rgb[2])
		
		color_hsv = colorsys.rgb_to_hsv(self.color_rgb[0], self.color_rgb[1], self.color_rgb[2])
		#color_hsv = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
		hsv_r = color_hsv[0]*1800//1
		hsv_g = color_hsv[1]*100//1
		hsv_b = color_hsv[2]		
		
		self.r_norm = self.color_rgb[0]*100 // total
		self.g_norm = self.color_rgb[1]*100 // total
		self.b_norm = self.color_rgb[2]*100 // total
		
		if self.r_norm > self.g_norm and self.b_norm < 12:
			return "Orange"
		elif  self.r_norm < self.b_norm and self.g_norm < self.b_norm:
			return "Blue"
		else:
			return "White"
			
		if hsv_r < 40 and hsv_g > 88 and hsv_b > 60:
			return "Orange"
		elif hsv_b <= 69 and hsv_g < 36:
			return "Blue"


			
			

	def close(self):
		GPIO.cleanup()
