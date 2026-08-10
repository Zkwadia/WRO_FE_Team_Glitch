import os
os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')

from TFmini import TFmini

sum_d = 0
prev_distance = 0
i = 0
avg_d = 0
if __name__ == "__main__":
	try:
		tfmini = TFmini(23, 24, 25, 27)

		while True:
			tfmini.getTFminiData()
			
			#avg_d = (tfmini.distance_right* 0.01) + (avg_d * 0.99)
			print(
				f"sum: {tfmini.distance_right + tfmini.distance_left} diff:{int((tfmini.distance_right - tfmini.distance_left)/2)} head : {tfmini.distance_head:.3f}, left: {tfmini.distance_left:.3f}, right: {tfmini.distance_right:.3f}  ")
	except Exception as e:
		print(e)

