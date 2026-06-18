import time
import os

os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")
time.sleep(5)

from datetime import datetime
import numpy as np
import cv2
import subprocess
import sys
import logging
import multiprocessing
import pigpio
import math
from Encoder import EncoderCounter
from Servo import Servo
import serial
import RPi.GPIO as GPIO
from TFmini import TFmini
import traceback
from collections import deque
from vision_pipeline_3 import VisionPipeline, FrameSmoother, USE_LAB, SMOOTH_N, FRAME_WIDTH, FRAME_HEIGHT, FRAME_MIDPOINT_X

os.makedirs("/home/pi/WRO_2026/logs", exist_ok=True)
log_file = open(f"/home/pi/wro_logs/logs/open_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log", "w")


class Tee:
	def __init__(self, *streams):
		self.streams = streams

	def write(self, data):
		for s in self.streams:
			s.write(data)
			s.flush()

	def flush(self):
		for s in self.streams:
			s.flush()


sys.stdout = Tee(sys.stdout, log_file)
sys.stderr = Tee(sys.stderr, log_file)
# ─────────────────────────────────────────────────────────────────────────────
# PINS
# ─────────────────────────────────────────────────────────────────────────────

RX_Head = 23
RX_Left = 24
RX_Right = 25
RX_Back = 27
button_pin = 5
exit_pin = 7
servo_pin = 8
blue_led = 26
red_led = 10
green_led = 6
reset_pin = 19

# ─────────────────────────────────────────────────────────────────────────────
# INITIALIZATION
# ─────────────────────────────────────────────────────────────────────────────

process = None
ser = serial.Serial("/dev/UART_USB", 115200)
print("created uart")

pwm = pigpio.pi()
if not pwm.connected:
	print("Could not connect to pigpio daemon")
	exit(1)

for pin in [reset_pin, blue_led, red_led, green_led]:
	pwm.set_mode(pin, pigpio.OUTPUT)
	pwm.write(pin, 0)
pwm.set_mode(button_pin, pigpio.INPUT)
pwm.set_pull_up_down(button_pin, pigpio.PUD_UP)

print("Resetting....")
pwm.write(reset_pin, 0)
pwm.write(green_led, 1)
time.sleep(1)
pwm.write(reset_pin, 1)
pwm.write(green_led, 0)
time.sleep(1)
print("Reset Complete")

servo = Servo(servo_pin)
tfmini_lock = multiprocessing.Lock()
tfmini = TFmini(RX_Head, RX_Left, RX_Right, RX_Back)
log = logging.getLogger("WRO")
rplidar = [None] * 360
previous_distance = 0
dist_0 = dist_90 = dist_270 = angle = 0
lidar_front = lidar_left = lidar_right = 0

# ─────────────────────────────────────────────────────────────────────────────
# MULTIPROCESSING VARIABLES
# ─────────────────────────────────────────────────────────────────────────────

counts = multiprocessing.Value("i", 0)
color_b = multiprocessing.Value("b", False)
red_b = multiprocessing.Value("b", False)
green_b = multiprocessing.Value("b", False)
pink_b = multiprocessing.Value("b", False)
centr_y = multiprocessing.Value("f", 0.0)
centr_x = multiprocessing.Value("f", 0.0)
centr_y_red = multiprocessing.Value("f", 0.0)
centr_x_red = multiprocessing.Value("f", 0.0)
centr_x_pink = multiprocessing.Value("f", 0.0)
centr_y_pink = multiprocessing.Value("f", 0.0)
head = multiprocessing.Value("f", 0.0)
previous_angle = multiprocessing.Value("d", 0.0)
orange_l = multiprocessing.Value("b", False)
blue_l = multiprocessing.Value("b", False)
wall_left = multiprocessing.Value("d", 0.0)
wall_right = multiprocessing.Value("d", 0.0)
outer_wall_left = multiprocessing.Value("d", 0.0)
outer_wall_right = multiprocessing.Value("d", 0.0)
close_wall = multiprocessing.Value("d", 0.0)
last_wall = multiprocessing.Value("d", 0.0)

time_video = multiprocessing.Value("d", 0.0)

left_a = multiprocessing.Value("f", 0.0)
right_a = multiprocessing.Value("f", 0.0)
red_area = multiprocessing.Value("b", False)
green_area = multiprocessing.Value("b", False)

# ─────────────────────────────────────────────────────────────────────────────
# PID VARIABLES
# ─────────────────────────────────────────────────────────────────────────────

currentAngle = 0
error_gyro = 0
prevErrorGyro = 0
totalErrorGyro = 0
correcion = 0
totalError = 0
prevError = 0

kp = 0.6
ki = 0
kd = 0.1
kp_e = 3
ki_e = 0
kd_e = 40

corr = 0
corr_pos = 0


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def get_closest_setpoint(heading: float) -> int:
	setpoints = [0, 90, 180, 270]

	heading = heading % 360

	def angular_diff(a, b):
		diff = abs(a - b) % 360
		return min(diff, 360 - diff)

	return min(setpoints, key=lambda sp: angular_diff(heading, sp))


def map_range(value, in_min, in_max, out_min, out_max):
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def update_heading(counter: int, heading_angle: float, blue: bool, orange: bool) -> float:
	if blue:
		return -((90 * counter) % 360)
	elif orange:
		return (90 * counter) % 360
	return heading_angle


def map_range(value, in_min, in_max, out_min, out_max):
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def correctWall(setPoint_distance, dist_left, dist_right, imu_h, orange, blue, sp_h):
	error_d = prevError_d = totalError_d = correction_d = 0

	if orange:
		error_d =setPoint_distance - dist_left
		print(f"orange error: {error_d}")
	elif blue:
		error_d = dist_right - setPoint_distance
		print(f"blue error: {error_d}")

	pTerm = 1.5 * error_d
	dTerm = 0 * (error_d - prevError_d)
	totalError_d += error_d
	iTerm = 0 * totalError_d
	correction = pTerm + iTerm + dTerm
	correction = max(-40, min(40, correction))



	#print(f"dist is {dist}")
	prevError_d = error_d

	correctAngle(sp_h + correction, imu_h, 1)



def correctAngle(setPoint_gyro, heading, multiplier):
    global corr
    error_gyro = prevErrorGyro = totalErrorGyro = correction = totalError = prevError = 0

    error_gyro = heading - setPoint_gyro
    if error_gyro > 180:
        error_gyro -= 360
    corr = error_gyro

    pTerm = kp * error_gyro * multiplier
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    if multiplier == 3:
        correction = max(-60, min(60, correction))
    else:
        correction = max(-30, min(30, correction))

    prevErrorGyro = error_gyro
    servo.setAngle(90 - correction)


def correctAngleAndWalls(setPoint_gyro, heading, multiplier, iwl, iwr, owl, owr):
    global corr
    error_gyro = prevErrorGyro = totalErrorGyro = correction = totalError = prevError = 0

    error_gyro = heading - setPoint_gyro
    if error_gyro > 180:
        error_gyro -= 360
    corr = error_gyro

    pTerm = kp * error_gyro * multiplier
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    if multiplier == 3:
        correction = max(-60, min(60, correction))
    else:
        correction = max(-30, min(30, correction))

    prevErrorGyro = error_gyro
    servo.setAngle(90 - correction)


def correctReverseAngle(setPoint_gyro, heading, multiplier):
	global corr
	error_gyro = prevErrorGyro = totalErrorGyro = correction = totalError = prevError = 0

	error_gyro = heading - setPoint_gyro
	if error_gyro > 180:
		error_gyro -= 360
	corr = error_gyro

	pTerm = kp * error_gyro * multiplier
	dTerm = kd * (error_gyro - prevErrorGyro)
	totalErrorGyro += error_gyro
	iTerm = ki * totalErrorGyro
	correction = pTerm + iTerm + dTerm

	if multiplier == 3:
		correction = max(-55, min(55, correction))
	else:
		correction = max(-30, min(30, correction))

	prevErrorGyro = error_gyro
	servo.setAngle(90 + correction)


def normalize_angle(angle, blue, orange, lane):
	if blue:
		return angle + 360 if angle < 180 and lane == 0 else angle
	elif orange:
		return angle - 360 if angle > 180 and lane == 0 else angle


def runMotor(speed, direction):
	pwm.set_PWM_dutycycle(12, int(speed * 2.55))
	pwm.write(20, direction)


# ─────────────────────────────────────────────────────────────────────────────
# PROCESS 1 — Live_Feed  (vision — HSV pipeline)
# ─────────────────────────────────────────────────────────────────────────────


def CameraProcess(
	red_b,
	green_b,
	pink_b,
	centr_y,
	centr_x,
	centr_y_red,
	centr_x_red,
	centr_x_pink,
	centr_y_pink,
	orange_l,
	blue_l,
	wall_left,
	wall_right,
	left_a,
	right_a,
	time_video,
	close_wall,
	red_area,
	green_area,
	outer_wall_left,
	outer_wall_right,
	last_wall
):
	print("Camera Process (HSV) started")

	SHOW_WINDOW = True
	os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
	os.environ.setdefault("DISPLAY", ":0")

	pipeline = VisionPipeline(USE_LAB)
	smoother = FrameSmoother(SMOOTH_N)
	fps_hist = deque(maxlen=30)
	t_prev = time.perf_counter()
	video_t = 0
	cap = pipeline.open_camera()

	recording = False
	date_str = datetime.now().strftime("%d-%m-%y_%H-%M-%S")
	recorder = None

	if cap is None:
		return

	if SHOW_WINDOW:
		WIN = "WRO 2026 — OpenCV Detection  |  Q quit"
		cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
		cv2.resizeWindow(WIN, FRAME_WIDTH, FRAME_HEIGHT)

	frame_count = 0

	try:
		while True:
			ret, frame = cap.read()
			if not ret or frame is None:
				continue
			frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

			detections = pipeline.detect(frame)

			# ── Filter by zone ────────────────────────────────────────────────
			magenta_list = [d for d in detections["magenta"] if d["zone"] == "main"]
			red_blocks = [d for d in detections["red"] if d["zone"] == "main"]
			green_blocks = [d for d in detections["green"] if d["zone"] == "main"]

			orange_detections = [d for d in detections["orange"] if d["zone"] == "line"]
			blue_detections = [d for d in detections["blue"] if d["zone"] == "line"]

			black_walls_left = [d for d in detections["black"] if d["zone"] == "wall_inner_left"]
			black_walls_right = [d for d in detections["black"] if d["zone"] == "wall_inner_right"]

			black_outer_walls_left = [d for d in detections["black"] if d["zone"] == "wall_left"]
			black_outer_walls_right = [d for d in detections["black"] if d["zone"] == "wall_right"]

			black_walls_close = [d for d in detections["black"] if d["zone"] == "close_black"]

			stop_condition_wall = [d for d in detections["black"] if d["zone"] == "last_counter_wall"]
			# Keep only the lowest (largest cy = closest) main block per colour
			if len(red_blocks) > 1:
				red_blocks = [max(red_blocks, key=lambda b: b["centroid"][1])]
			if len(green_blocks) > 1:
				green_blocks = [max(green_blocks, key=lambda b: b["centroid"][1])]

			red_present = bool(red_blocks)
			green_present = bool(green_blocks)
			pink_present = bool(magenta_list)

			# ── Debug print ───────────────────────────────────────────────────
			for color in ["red", "green", "magenta", "black", "orange", "blue"]:
				for d in detections[color]:
					cx, cy = d["centroid"]

			# ── Reset all shared flags every frame ────────────────────────────
			red_b.value = False
			green_b.value = False
			pink_b.value = False
			orange_l.value = False
			blue_l.value = False

			orange_l.value = bool(orange_detections)
			blue_l.value = bool(blue_detections)

			wall_left_flag = bool(black_walls_left)
			wall_right_flag = bool(black_walls_right)
			wall_close_flag = bool(black_walls_close)
			outer_left_flag = bool(black_outer_walls_left)
			outer_right_flag = bool(black_outer_walls_right)


			last_stopping_flag = bool(stop_condition_wall)
			# print(f"black wall left: {black_walls_left} black wall right: {black_walls_right}")

			# ── Magenta / pink ────────────────────────────────────────────────
			if pink_present:
				best = max(magenta_list, key=lambda b: b["area"])
				centr_x_pink.value = float(best["centroid"][0])
				centr_y_pink.value = float(best["centroid"][1])
				pink_b.value = True
			else:
				centr_x_pink.value = 0.0
				centr_y_pink.value = 0.0

			# ── Red + green simultaneously: biggest area wins ─────────────────
			if red_present and green_present:
				best_red = max(red_blocks, key=lambda b: b["area"])
				best_green = max(green_blocks, key=lambda b: b["area"])

				if best_green["centroid"][1] > best_red["centroid"][1]:
					centr_x.value = float(best_green["centroid"][0])
					centr_y.value = float(best_green["centroid"][1])
					centr_x_red.value = 0.0
					centr_y_red.value = 0.0
					green_area.value = True
					# if centr_x.value < 540:
					green_b.value = True
				else:
					centr_x_red.value = float(best_red["centroid"][0])
					centr_y_red.value = float(best_red["centroid"][1])
					centr_x.value = 0.0
					centr_y.value = 0.0
					red_area.value = True
					# if centr_x_red.value > 100:
					red_b.value = True

			elif red_present:
				best_red = max(red_blocks, key=lambda b: b["area"])
				centr_x_red.value = float(best_red["centroid"][0])
				centr_y_red.value = float(best_red["centroid"][1])
				centr_x.value = 0.0
				centr_y.value = 0.0
				# if centr_x_red.value > 100:
				red_b.value = True

			elif green_present:
				best_green = max(green_blocks, key=lambda b: b["area"])
				centr_x.value = float(best_green["centroid"][0])
				centr_y.value = float(best_green["centroid"][1])
				centr_x_red.value = 0.0
				centr_y_red.value = 0.0
				green_b.value = True

			else:
				centr_x.value = 0.0
				centr_y.value = 0.0
				centr_x_red.value = 0.0
				centr_y_red.value = 0.0

			# ── Walls ─────────────────────────────────────────────────────────
			# print(f"wall_left: {wall_left_flag} wall_right: {wall_right_flag}")
			wall_left_area = 0
			wall_right_area = 0
			if wall_left_flag:
				best_black_left = max(black_walls_left, key=lambda b: b["area"])
				wall_left.value = best_black_left["centroid"][0]
				left_a.value = best_black_left["area"]
			if wall_right_flag:
				best_black_right = max(black_walls_right, key=lambda b: b["area"])
				wall_right.value = best_black_right["centroid"][0]
				right_a.value = best_black_right["area"]
			if wall_close_flag:
				best_black_close = max(black_walls_close, key=lambda b: b["area"])
				close_wall.value = best_black_close["area"]

			if outer_left_flag and not outer_right_flag:
				best_outer_left = max(black_outer_walls_left, key=lambda b: b["area"])
				outer_wall_left.value = best_outer_left["centroid"][0]
				outer_wall_right.value = 0.0
			elif outer_right_flag and not outer_left_flag:
				best_outer_right = max(black_outer_walls_right, key=lambda b: b["area"])
				outer_wall_right.value = best_outer_right["centroid"][0]
				outer_wall_left.value = 0.0
			if outer_left_flag and outer_right_flag:
				best_outer_left = max(black_outer_walls_left, key=lambda b: b["area"])
				best_outer_right = max(black_outer_walls_right, key=lambda b: b["area"])
				outer_wall_right.value = best_outer_right["centroid"][0]
				outer_wall_left.value = best_outer_left["centroid"][0]

				
			if last_stopping_flag:
				best_last_wall = max(stop_condition_wall, key= lambda b:b["area"])
				last_wall.value = best_last_wall["centroid"][0]
			else:
				last_wall.value = 0.0
			#print(f"last_wall : {last_wall.value}")
			# print(f"outer l {outer_wall_left.value} outer r:{outer_wall_right.value}")
			# print(f"close: {close_wall.value}")

			if not wall_left_flag and not wall_right_flag:
				# print("elseeeeeeee")
				right_a.value = 0
				left_a.value = 0
				wall_left.value = 0.0
				wall_right.value = 0.0
				
			if not wall_close_flag:
				close_wall.value = 0.0

			if not outer_left_flag and not outer_right_flag:
				outer_wall_left.value = 0.0
				outer_wall_right.value = 0.0

			# ── Annotation & display ──────────────────────────────────────────
			t_now = time.perf_counter()
			fps = 1.0 / max(t_now - t_prev, 1e-6)
			t_prev = t_now
			frame_count += 1
			# print(f"left wall: {wall_left.value} right wall: {wall_right.value}")
			# print(f"r: {red_b.value} centr:{centr_x_red.value}")
			# print(f"left area : {left_a.value} right area: {right_a.value}")
			#print(f"close: {close_wall.value}")
			annotated = pipeline.annotate(frame, detections, USE_LAB, fps, time_video.value)
			# recorder.write(annotated)
			if video_t > 0:
				time_video.value = time.time() - video_t
			if SHOW_WINDOW:
				annotated = pipeline.annotate(frame, detections, USE_LAB, fps, time_video.value)
				cv2.imshow(WIN, annotated)
			key = cv2.waitKey(1) & 0xFF
			if key == ord("q"):
				break
			elif key == ord("s"):
				cv2.imwrite("/home/pi/WRO_2026/videos/image.png", annotated)
				print("Snapshot saved!")
			elif key == ord("r") and not recording:
				actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
				actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
				video_t = time.time()
				print(f"Actual resolution: {actual_w}x{actual_h}")
				recorder = cv2.VideoWriter(
					f"/home/pi/wro_logs/videos/ope_challenge_{date_str}.avi",
					cv2.VideoWriter_fourcc(*"MJPG"),
					20.0,
					(actual_w, 360),  # use actual size, not hardcoded
				)
				recording = True
				print("Recording started!")
			elif key == ord("t") and recording:
				recorder.release()
				recorder = None
				recording = False
				print("Recording stopped and saved")

			if recording and recorder is not None:
				recorder.write(annotated)
	except KeyboardInterrupt:
		pass
	finally:
		cap.release()
		cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────────────────
# PROCESS 2 — servoDrive  (main control loop)
# ─────────────────────────────────────────────────────────────────────────────


def DriveProcess(
	red_b,
	green_b,
	pink_b,
	counts,
	centr_y,
	centr_x,
	centr_y_red,
	centr_x_red,
	centr_x_pink,
	centr_y_pink,
	head,
	orange_l,
	blue_l,
	wall_left,
	wall_right,
	left_a,
	right_a,
	time_video,
	close_wall,
	red_area,
	green_area,
	outer_wall_left,
	outer_wall_right,
	last_wall,
):
	pwm = pigpio.pi()
	global imu, corr, corr_pos
	pwm_pin = 12
	direction_pin = 20
	pwm.set_mode(pwm_pin, pigpio.OUTPUT)
	pwm.set_mode(direction_pin, pigpio.OUTPUT)
	pwm.hardware_PWM(pwm_pin, 55, 0)
	pwm.set_PWM_dutycycle(pwm_pin, 0)

	enc = EncoderCounter()

	button = trigger = reset_f = False
	blue_flag = orange_flag = False
	timer_v = 0
	calc_time = False
	lap_finish = continue_parking = parking_heading = parking_flag = False
	turn_flag = reset_flags = counter_reset = False
	finished = False
	red_time = green_time = False
	parking_right = True
	parking_left = False
	exit_flag = False
	power = 95
	prev_power = 0
	last_counter = 12
	counter = turn_t = current_time = gp_time = rp_time = buff = c_time = 0
	heading_angle = 0
	lap_finish_time = prev_distance = turn_trigger_distance = target_count = offset = 0
	button_STATE = exit_STATE = 0
	time_p = prev_time = 0
	debounce_delay = 0.1
	last_time = 0
	exit_last_time = 0

	parking_flag = False
	inParkingatStart = False
	init = False
	STATE = 1
	parking_count = 500
	parking_count_current = 0
	full_park = False
	servo.setAngle(40)
	timer_t = time.time()
	STATE_INIT = 1
	enc_count = 0
	parking_STATE = 1
	OBSTACLE_STATE = 1
	RESET_STATE = 1
	start_enc_thresh = 0
	corr_thresh = 0
	parking_distance = 0
	forward_time = time.time()
	final_park = time.time()
	front_thresh = 95
	parking_timeout = 3
	finish_thresh = 1100
	before_finish = 0
	prev_angle = 0

	handle_turn = False
	counter_flag = True
	servo_angle = 0
	g_time = time.time()
	r_time = time.time()
	trig_time = time.time()

	err = 0
	prev_err = 0
	orange_detected = False
	blue_detected = False
	orange_vanished = False
	blue_vanished = False
	map_time = 0
	video_t = 0
	closest_heading = 0
	centroid = 0
	try:
		while True:

			imu_head = head.value
			tfmini.getTFminiData()
			tf_h = tfmini.distance_head
			tf_l = tfmini.distance_left
			tf_r = tfmini.distance_right
			x, y = enc.get_position(head.value, counts.value)

			if not init:
				if pink_b.value or red_b.value or green_b.value or wall_left.value or wall_right.value or outer_wall_left.value or outer_wall_right.value:
					correctAngle(heading_angle, head.value, 1.5)
					init = True
			# print(f"heading: {head.value} closest sp:{get_closest_setpoint(head.value)}")

			#########################################################################


			#######################################################################

			# print(f"parkingatStart: {inParkingatStart} orange:{orange_flag} blue:{blue_flag}")
			# map_time = map_range(tfmini.distance_left, 0, 100, 0, 2)

			# print(f"orange:{orange_l.value} blue:{blue_l.value} left:{tfmini.distance_left} map_time: {map_time}")

			# print(f"wall left:{wall_left.value} wall right: {wall_right.value}")
			if time.time() - last_time > debounce_delay:
				previous_STATE = button_STATE
				button_STATE = pwm.read(button_pin)
				if previous_STATE == 1 and button_STATE == 0:
					button = not button
					last_time = time.time()
					print(f"🔘 Button toggled! Drive {'started' if button else 'stopped'}")
					power = 95

			if time.time() - exit_last_time > debounce_delay:
				previous_EXIT_STATE = exit_STATE
				exit_STATE = pwm.read(exit_pin)
				if previous_EXIT_STATE == 1 and exit_STATE == 0:
					exit_flag = not exit_flag
					exit_last_time = time.time()
					print(f"🔘 Exit button toggled!")
					power = 95

			if not button:
				# print(f"tfmini.distance_left:{tfmini.distance_left} tfmini.distance_right:{tfmini.distance_right} tfmini.distance_head:{tfmini.distance_head}")
				# print(f"wall left: {wall_left.value} wall_right: {wall_right.value} orange_flag:{orange_flag} imu:{head.value}")
				pass

			if button:
				print("-------------------------------------------------")
				if red_b.value or green_b.value:
					power = 80
				elif not red_b.value and not green_b.value:
					power = 95
				x, y = enc.get_position(imu_head, counts.value)
    
				#################################### DIRECTION DECISION ########################################
				if not orange_flag and not blue_flag:
					if ((tfmini.distance_head < 70 and tfmini.distance_left > 100) or blue_l.value) and not blue_flag and not trigger:
						print(f"first triggger orange")
						blue_flag = True
						trigger = True
						orange_flag = False
						prev_time = time.time()
						trigger = True
						counter_flag = False
					elif ((tfmini.distance_head < 70 and tfmini.distance_right > 100) or orange_l.value) and not orange_flag and not trigger:
						print(f"first trigger blue")
						orange_flag = True
						blue_flag = False
						trigger = True
						prev_time = time.time()
						trigger = True
						counter_flag = False


			##################################### HANDLE TURNS ########################################


				##################################### HANDLE OBSTACLE AVOIDANCE ###########################

				centroid = 0
				centroid_pink = 0
				err = 0
				multiplier = 1
				if outer_wall_left.value > 0 and outer_wall_right.value > 0:
					print(f"correcting center...")
					left_offset = 320 - outer_wall_left.value
					right_offset = 320 + (640 - outer_wall_right.value)
					centroid = (left_offset + right_offset)/2
					err = 320 - centroid
				elif outer_wall_left.value > 0 and  not outer_wall_right.value > 0:
					print(f"correcting left at center...")
					left_offset = 320 + outer_wall_left.value
					#right_offset = 320 + (640 - outer_wall_right.value)
					#err = left_offset - 320
					#err = 5
					err = outer_wall_left.value/2
					#err = centr
				elif outer_wall_right.value > 0 and  not outer_wall_left.value > 0:
					print(f"correcting right at center...")
					right_offset = 640 - outer_wall_right.value
					#right_offset = 320 + (640 - outer_wall_right.value)
					centroid = 320 + right_offset
					#err = -5
					err = -(640 - outer_wall_right.value)/2
					#err = 320 - centroid
				print(f"err: {err}")
				# print(f"red_b.value: {red_b.value} green_b.value:{green_b.value}")
				# print(f"both diff: {abs(r_time - g_time)}  r_time diff: {time.time() - r_time:.2f} g_time: {time.time() - g_time:.2f} ")

				##################################################### PID CENTROID ############################################

				kp = 0.5#0.075  # 0.08
				kd = 0.5  # 0.65

				servo_angle = err * kp + ((err - prev_err) * kd)
				servo_angle = max(-45, min(45, servo_angle))
				prev_err = err

				if (outer_wall_left.value > 0 or outer_wall_right.value > 0) and not close_wall.value > 0 and counter != 0:
					print(f"wall pid...")
					#correctWall(25, tfmini.distance_left, tfmini.distance_right, head.value, orange_flag, blue_flag, heading_angle)
					servo.setAngle(90 + servo_angle)
				else:
					if not close_wall.value > 0 :
						if counter != 0:
							if (wall_left.value > 0 and not wall_right.value > 0):
								print(f"correcting  left")
								correctAngle(heading_angle + 10, head.value, 1)
							elif (wall_right.value > 0 and not wall_left.value > 0):
								print(f"correcting  right")
								correctAngle(heading_angle - 10, head.value, 1)
							else:
								print(f" correcting heading...")
								correctAngle(heading_angle, head.value, 1)
						else:
							print(f"nothing is there correcting heading...")
							correctAngle(heading_angle, head.value, 1.5)
						
					else:							
						print(f"correcting heading normal...")
						correctAngle(heading_angle, head.value, 1.5)

				# print(f"servo_angle: {servo_angle}")
				#print(f"r_time diff:{time.time() - r_time} r_time: {r_time} g_time diff: {time.time() - g_time} g_time: {g_time}")
				# print(f"multiplier:{multiplier} centroid: {centroid} err:{err} servo_angle:{servo_angle}  black left: {left_a.value} black_right: {right_a.value}")'''
				print(f"left wall cx:{wall_left.value} right wal cx:{wall_right.value} outer left: {outer_wall_left.value} outer_right: {outer_wall_right.value} close_wall:{close_wall.value}  last_wall: {last_wall.value}")
				print(f"orange_flag: {orange_flag} blue_flag: {blue_flag}")
				#print(f"red centroid: {centr_x_red.value} green centroid: {centr_x.value}")
				print(f"centroid : {centroid} imu :{head.value}")
				print(f"left sensor:{tfmini.distance_left} right sensor: {tfmini.distance_right} head: {tfmini.distance_head}")

				##########################################################################################################################
				##################################### HANDLE TURNS ########################################

				if not counter_flag and trigger:
					print(f"AT TRIGGERING EVENT")
					while not close_wall.value > 0:
						print(f"correcting heading at triggre :{time_video.value:.2f} {heading_angle}")
						runMotor(95, 1)
						correctAngle(heading_angle, head.value, 1.5)
					if time.time() - prev_time > 0:
						counter += 1
						counter_flag = True
						heading_angle = update_heading(counter, heading_angle, blue_flag, orange_flag)
   
				if orange_l.value and orange_flag and not trigger :
					print(f"Tigger Detected...")
					prev_time = time.time()
					map_time = map_range(tfmini.distance_left, 0, 100, 0, 0.7)
					trigger = True
					counter_flag = False
				# elif ((orange_l.value or (blue_l.value and tfmini.distance_right < 50)) and blue_flag) and not trigger:
				elif blue_l.value and blue_flag and not trigger:
					print(f"Tigger Detected...")
					map_time = map_range(tfmini.distance_right, 0, 100, 0, 0.7)
					prev_time = time.time()
					trigger = True
					counter_flag = False
				elif time.time() - prev_time > 2 and trigger:
					trigger = False

 

					###########################################################################################################
				if counter == last_counter:
					if not trigger:
						if time.time() - prev_time > 3.5 and get_closest_setpoint(head.value) == heading_angle and last_wall.value > 0:
							print(f"Open Challenge Done...")
							power = 0
							prev_pwer = 0
							runMotor(power, 1)
							sys.exit(0)


				total_power = (power * 0.1) + (prev_power * 0.9)
				prev_power = total_power
				runMotor(total_power, 1)

				#print(f"centroid val: {centroid} servo_angle = {servo_angle} err = {err}")
				#print(f"pink: {pink_b.value} green:{green_b.value} red:{red_b.value} imu:{head.value:.2f}")
				print(f"tigger :{trigger} counter_flag: {counter_flag} map_time:{map_time:.2f} ")
				print(f"counter:{counter} heading_angle:{heading_angle} closest_sp:{closest_heading} diff: {time.time() - prev_time:.2f} video_time:{time_video.value:.2f}")

				# print(f"OBSTACLE_STATE:{OBSTACLE_STATE} RESET_STATE:{RESET_STATE}")
				print("---------------------------------------------------")

			else:
				power = 0
				pwm.hardware_PWM(12, 55, 0)
				counter = 0

			if exit_flag and button:
				print("Shutting down the program")
				sys.exit(0)

	except Exception as e:
		print(f"Exception: {e}")
		tb = traceback.extract_tb(e.__traceback__)
		filename, lineno, func, text = tb[-1]
		print(f"⚠️ Exception in {filename}, line {lineno}, in {func}")
		if isinstance(e, KeyboardInterrupt):
			power = 0
			pwm.hardware_PWM(12, 55, 0)
			heading_angle = 0
			counter = 0
			correctAngle(heading_angle, head.value, 1.5)
			red_b.value = False
			green_b.value = False
	finally:
		pwm.set_PWM_dutycycle(12, 0)
		pwm.write(20, 0)
		print("Motors stopped safely.")
		pwm.stop()


# ─────────────────────────────────────────────────────────────────────────────
# PROCESS 3 — Encoder / IMU reader
# ─────────────────────────────────────────────────────────────────────────────


def IMUandEncoder(counts, head):
	pwm = pigpio.pi()
	print("IMU and Encoder Process Started")
	time.sleep(2)
	try:
		while True:
			line = ser.readline().decode("utf-8", errors="ignore").strip()
			esp_data = line.split()
			if len(esp_data) >= 2:
				try:
					head.value = float(esp_data[0])
					counts.value = int(esp_data[1])
				except ValueError:
					print(f"⚠️ Malformed ESP data: {esp_data}")
			else:
				print(f"⚠️ Incomplete ESP data: {esp_data}")
	except Exception as e:
		print(f"Exception Encoder:{e}")
	finally:
		ser.close()


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
	try:
		print("Starting process")

		P = multiprocessing.Process(
			target=CameraProcess,
			args=(
				red_b,
				green_b,
				pink_b,
				centr_y,
				centr_x,
				centr_y_red,
				centr_x_red,
				centr_x_pink,
				centr_y_pink,
				orange_l,
				blue_l,
				wall_left,
				wall_right,
				left_a,
				right_a,
				time_video,
				close_wall,
				red_area,
				green_area,
				outer_wall_left,
				outer_wall_right,
				last_wall
			),
		)
		S = multiprocessing.Process(
			target=DriveProcess,
			args=(
				red_b,
				green_b,
				pink_b,
				counts,
				centr_y,
				centr_x,
				centr_y_red,
				centr_x_red,
				centr_x_pink,
				centr_y_pink,
				head,
				orange_l,
				blue_l,
				wall_left,
				wall_right,
				left_a,
				right_a,
				time_video,
				close_wall,
				red_area,
				green_area,
				outer_wall_left,
				outer_wall_right,
				last_wall
			),
		)
		E = multiprocessing.Process(target=IMUandEncoder, args=(counts, head))

		P.start()
		E.start()
		S.start()

	except KeyboardInterrupt:
		ser.close()
		E.terminate()
		S.terminate()
		P.terminate()
		E.join()
		S.join()
		P.join()
		pwm.hardware_PWM(12, 55, 0)
		pwm.bb_serial_read_close(RX_Head)
		pwm.bb_serial_read_close(RX_Left)
		pwm.bb_serial_read_close(RX_Right)
		pwm.stop()
		tfmini.close()
