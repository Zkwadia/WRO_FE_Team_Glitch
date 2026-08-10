import time
import os
from build_video import _build_video

os.system("sudo pkill pigpiod")
time.sleep(2)
os.system("sudo pigpiod -x -1")  # -x -1 = allow every GPIO
time.sleep(3)  # give the daemon time to come up
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
import traceback
from collections import deque
from vision_pipeline_new import VisionPipeline, FrameSmoother, USE_LAB, SMOOTH_N, FRAME_WIDTH, FRAME_HEIGHT, FRAME_MIDPOINT_X
from vl53l0x import VL53L0XArray
import json
from TFmini import TFmini

GAINS_FILE = "/home/pi/WRO_2026/versionTest/gains.json"
GAINS_CHECK_INTERVAL = 0.5  # seconds between file checks — cheap, don't check every tick
_gains_last_check = 0.0
_gains_mtime = 0.0


"""{
  "kp": 0.6,
  "kd": 0.01,
  "ki": 0,
  "kp_s": 0.06,
  "kd_s": 0.8,
  "kp_v": 0.8,
  "ki_v": 0.0,
  "kd_v": 0.05,
  "SPEED_SCALE": 40.0
}"""

kp = 0.6
kd = 0.01
ki = 0
kp_s = 0.06
kd_s = 0.8


def load_gains_if_changed():
	global kp, kd, ki, kp_s, kd_s, kp_v, ki_v, kd_v, SPEED_SCALE, _gains_last_check, _gains_mtime

	now = time.time()
	if now - _gains_last_check < GAINS_CHECK_INTERVAL:
		return
	_gains_last_check = now

	try:
		mtime = os.path.getmtime(GAINS_FILE)
		if mtime == _gains_mtime:
			return  # file hasn't changed since last load
		with open(GAINS_FILE) as f:
			gains = json.load(f)
		kp = gains.get("kp", kp)
		kd = gains.get("kd", kd)
		ki = gains.get("ki", ki)
		kp_s = gains.get("kp_s", kp_s)
		kd_s = gains.get("kd_s", kd_s)
		kp_v = gains.get("kp_v", kp_v)
		ki_v = gains.get("ki_v", ki_v)
		kd_v = gains.get("kd_v", kd_v)
		SPEED_SCALE = gains.get("SPEED_SCALE", SPEED_SCALE)
		_gains_mtime = mtime
		print(f"[gains] reloaded: kp={kp} kd={kd} ki={ki} kp_s={kp_s} kd_s={kd_s} kp_v={kp_v} ki_v={ki_v} kd_v={kd_v} SPEED_SCALE={SPEED_SCALE}")
	except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
		print(f"[gains] reload skipped ({e}) — keeping current values")


os.makedirs("/home/pi/WRO_2026/logs", exist_ok=True)
log_file = open(f"/home/pi/wro_logs/logs/obstacle_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log", "w")


class Tee:
	def __init__(self, *streams):
		self.streams = streams
		self._counts = 0

	def write(self, data):
		for s in self.streams:
			s.write(data)
			# s.flush()
			self._counts += 1
		if self._counts % 20 == 0:
			self.flush()

	def flush(self):
		for s in self.streams:
			s.flush()


sys.stdout = Tee(sys.stdout, log_file)
sys.stderr = Tee(sys.stderr, log_file)
# ─────────────────────────────────────────────────────────────────────────────
# PINS
# ─────────────────────────────────────────────────────────────────────────────

RX_Head = 23
RX_Left = 17
RX_Right = 25
RX_Back = 27
button_pin = 5
switch_pin = 6
exit_pin = 7
servo_pin = 8
blue_led = 26
red_led = 13
green_led = 6
reset_pin = 19

# MDD3A Motor Driver pins
PIN_A = 16  # MDD3A M1A
PIN_B = 20  # MDD3A M1B

# ─────────────────────────────────────────────────────────────────────────────
# INITIALIZATION
# ─────────────────────────────────────────────────────────────────────────────

process = None
# ser = serial.Serial("/dev/XIAO_USB", 115200)
print("created uart")


# VL53L0X distance sensor array (replaces TFmini)
# tof = VL53L0XArray()
# tof.init()

# log = logging.getLogger("WRO")
rplidar = [None] * 360
previous_distance = 0
dist_0 = dist_90 = dist_270 = angle = 0
lidar_front = lidar_left = lidar_right = 0

# ─────────────────────────────────────────────────────────────────────────────
# MULTIPROCESSING VARIABLES
# ─────────────────────────────────────────────────────────────────────────────

counts = multiprocessing.Value("i", 0)
fps_m = multiprocessing.Value("f", 0.0)

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
park_wall = multiprocessing.Value("d", 0.0)
last_wall = multiprocessing.Value("d", 0.0)
last_wall_2 = multiprocessing.Value("d", 0.0)

time_video = multiprocessing.Value("d", 0.0)

left_a = multiprocessing.Value("f", 0.0)
right_a = multiprocessing.Value("f", 0.0)
red_area = multiprocessing.Value("b", False)
green_area = multiprocessing.Value("b", False)


centr_y_close = multiprocessing.Value("f", 0.0)
centr_x_close = multiprocessing.Value("f", 0.0)
centr_y_red_close = multiprocessing.Value("f", 0.0)
centr_x_red_close = multiprocessing.Value("f", 0.0)

# near your other multiprocessing.Value declarations in __main__
switch_state = multiprocessing.Value("b", False)

# ─────────────────────────────────────────────────────────────────────────────
# SHARED MEMORY FOR TOF SENSOR READINGS (so DriveProcess can read them)
# ─────────────────────────────────────────────────────────────────────────────

tof_front = multiprocessing.Value("d", 0.0)
tof_left = multiprocessing.Value("d", 0.0)
tof_right = multiprocessing.Value("d", 0.0)
tof_rear = multiprocessing.Value("d", 0.0)


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
prevErrorServo = 0
totalErrorServo = 0
prev_loop_time = time.time()

kp_e = 3
ki_e = 0
kd_e = 40

corr = 0
corr_pos = 0

# ── SPEED PID (closed-loop, compensates for battery voltage sag) ──────────
# Corrects motor duty cycle so actual wheel speed (from encoder counts)
# tracks a target speed derived from the requested 'power' percentage,
# regardless of battery voltage sag. Direction-aware: encoder counts
# DECREASE during reverse on this robot, so the delta is sign-flipped
# for direction == 0. Auto-resets whenever direction flips, since forward
# and reverse have different dynamics and stale state would cause a kick.
kp_v = 0.8
ki_v = 0.0
kd_v = 0.05
SPEED_SCALE = 40.0  # counts/sec at duty=100 on a fresh battery — CALIBRATE from logs

prev_counts_speed = 0
prev_error_speed = 0
total_error_speed = 0
speed_duty = 0.0
prev_loop_time_speed = time.time()
prev_direction_speed = None  # tracks last direction so we auto-reset on flip


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────


def is_switch_off_and_stop(pwm):
	"""Returns True if the kill switch has been flipped off.
	As a side effect, immediately cuts motor PWM so the robot
	doesn't coast/keep driving while the outer loop catches up."""
	if pwm.read(switch_pin) == 0:
		pwm.set_PWM_dutycycle(PIN_A, 0)
		pwm.set_PWM_dutycycle(PIN_B, 0)
		return True
	return False


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


def correctWall(setPoint_distance, dist, sp_h, imu_h, orange, blue, pink_l, counter, left, right):
	error_d = prevError_d = totalError_d = correction_d = 0

	if right:
		error_d = dist - setPoint_distance
		print(f"right error: {error_d}")
	elif left:
		error_d = setPoint_distance - dist
		print(f"left error: {error_d}")

	pTerm = 2.5 * error_d
	dTerm = 0 * (error_d - prevError_d)
	totalError_d += error_d
	iTerm = 0 * totalError_d
	correction = pTerm + iTerm + dTerm
	correction = max(-40, min(40, correction))

	"""if dist < 30 and setPoint_distance == 35:
		print("correction 0 35")
		correction = 0
	if setPoint_distance == 60 and dist < 20:
		print("correction 0 60")
		correction = 0
	if setPoint_distance == 45 and dist < 20:
		print("correction 0 is 45")
		correction = 0"""

	print(f"dist is {dist}")
	prevError_d = error_d
	if counter % 4 == pink_l:
		correctAngle(sp_h + correction, imu_h, 1)
	else:
		correctAngle(sp_h + correction, imu_h, 1.8)


def correctAngle(setPoint_gyro, heading, multiplier):
	global corr, prevErrorGyro, totalErrorGyro, prev_loop_time, kp, kd
	error_gyro = correction = 0
	now = time.time()
	dt = now - prev_loop_time
	dt = max(dt, 1e-3)  # avoid divide-by-zero
	dt = min(dt, 0.2)  # clamp so one long stall doesn't spike the derivative
	prev_loop_time = now
	error_gyro = heading - setPoint_gyro
	if error_gyro > 180:
		error_gyro -= 360
	corr = error_gyro

	pTerm = kp * error_gyro * multiplier
	dTerm = kd * ((error_gyro - prevErrorGyro) / dt)
	totalErrorGyro += error_gyro
	iTerm = ki * totalErrorGyro
	correction = pTerm + iTerm + dTerm

	if multiplier == 3:
		correction = max(-45, min(45, correction))
	else:
		correction = max(-25, min(25, correction))

	prevErrorGyro = error_gyro
	servo.setAngle(95 - correction)


def correctReverseAngle(setPoint_gyro, heading, multiplier):
	global corr, prev_loop_time, kp, kd
	error_gyro = prevErrorGyro = totalErrorGyro = correction = totalError = prevError = 0
	now = time.time()
	dt = now - prev_loop_time
	dt = max(dt, 1e-3)  # avoid divide-by-zero
	dt = min(dt, 0.2)  # clamp so one long stall doesn't spike the derivative
	prev_loop_time = now
	error_gyro = heading - setPoint_gyro
	if error_gyro > 180:
		error_gyro -= 360
	corr = error_gyro

	pTerm = kp * error_gyro * multiplier
	dTerm = kd * ((error_gyro - prevErrorGyro) / dt)
	totalErrorGyro += error_gyro
	iTerm = ki * totalErrorGyro
	correction = pTerm + iTerm + dTerm

	if multiplier == 3:
		correction = max(-45, min(45, correction))
	else:
		correction = max(-25, min(25, correction))

	prevErrorGyro = error_gyro
	servo.setAngle(95 + correction)


def normalize_angle(angle, blue, orange, lane):
	if blue:
		return angle + 360 if angle < 180 and lane == 0 else angle
	elif orange:
		return angle - 360 if angle > 180 and lane == 0 else angle


def resetSpeedPID(counts_value):
	"""Clears speed-PID integrator/derivative/output state. Call this
	whenever the target speed jumps discontinuously (e.g. a parking STATE
	transition) so stale error from the previous setpoint doesn't cause
	a kick. Direction flips reset automatically inside correctSpeed."""
	global prev_error_speed, total_error_speed, speed_duty, prev_counts_speed, prev_loop_time_speed
	prev_error_speed = 0.0
	total_error_speed = 0.0
	speed_duty = 0.0
	prev_counts_speed = counts_value
	prev_loop_time_speed = time.time()


def correctSpeed(target_power_pct, counts_value, direction):
	"""
	Closed-loop speed control, direction-aware. Standard positional PID:
	output duty = baseline power + PID correction, recomputed fresh each
	call — NOT accumulated tick-over-tick (that was the bug).
	direction: 1 = forward, 0 = reverse. Encoder counts DECREASE during
	reverse on this robot, so raw delta is sign-flipped for direction==0.
	Auto-resets on direction change.
	"""
	global prev_counts_speed, prev_error_speed, total_error_speed
	global prev_loop_time_speed, prev_direction_speed
	global kp_v, ki_v, kd_v, SPEED_SCALE

	if direction != prev_direction_speed:
		resetSpeedPID(counts_value)
		prev_direction_speed = direction
		return target_power_pct   # first call after flip — start at baseline, not 0

	now = time.time()
	dt = now - prev_loop_time_speed
	dt = max(dt, 1e-3)
	dt = min(dt, 0.2)
	prev_loop_time_speed = now

	raw_delta = counts_value - prev_counts_speed
	prev_counts_speed = counts_value

	actual_cps = (raw_delta / dt) if direction == 1 else (-raw_delta / dt)

	target_cps = target_power_pct * SPEED_SCALE / 100.0
	error = target_cps - actual_cps

	total_error_speed += error * dt
	total_error_speed = max(-500, min(500, total_error_speed))  # anti-windup

	derivative = (error - prev_error_speed) / dt
	prev_error_speed = error

	correction = kp_v * error + ki_v * total_error_speed + kd_v * derivative
	duty = target_power_pct + correction   # baseline + correction, computed fresh
	duty = max(0.0, min(100.0, duty))

	return duty


def driveAtSpeed(pwm, target_power_pct, direction, counts_value):
	"""One-line drop-in replacement for runMotor(pwm, power, direction)
	that closes the loop on wheel speed via correctSpeed()."""
	duty = correctSpeed(target_power_pct, counts_value, direction)
	runMotor(pwm, duty, direction)
	return duty


def runMotor(pwm, speed, direction):
	"""
	MDD3A Truth Table (pigpio):
	  Forward  (direction=1) → PIN_A=PWM, PIN_B=0
	  Backward (direction=0) → PIN_A=0,   PIN_B=PWM
	  Stop     (speed=0)     → PIN_A=0,   PIN_B=0
	"""
	duty = int(max(0.0, min(100.0, speed)) * 2.55)  # 0–100 → 0–255
	if direction == 1:  # forward
		pwm.set_PWM_dutycycle(PIN_B, 0)
		pwm.set_PWM_dutycycle(PIN_A, duty)
	elif direction == 0:  # backward
		pwm.set_PWM_dutycycle(PIN_A, 0)
		pwm.set_PWM_dutycycle(PIN_B, duty)
	else:  # brake / stop
		pwm.set_PWM_dutycycle(PIN_A, 0)
		pwm.set_PWM_dutycycle(PIN_B, 0)


def TOFProcess(tof_front, tof_left, tof_right, tof_rear):
	"""
	Dedicated process for reading VL53L0X sensors.
	Writes latest values into shared memory at full speed.
	DriveProcess reads from shared memory — no blocking waits.
	"""
	print("TOF Process started")
	sensor = VL53L0XArray()
	sensor.init()

	try:
		while True:

			readings = sensor.all()
			time.sleep(0.01)
			# Clamp negatives to 0 for easier comparisons in DriveProcess
			tof_front.value = max(readings["front"], 0)
			tof_left.value = max(readings["left"], 0)
			tof_right.value = max(readings["right"], 0)
			tof_rear.value = max(readings["rear"], 0)
	except Exception as e:
		print(f"TOF Process exception: {e}")
	finally:
		sensor.close()
		print("TOF Process stopped")


# ─────────────────────────────────────────────────────────────────────────────
# PROCESS 1 — Live_Feed  (vision — HSV pipeline)
# ─────────────────────────────────────────────────────────────────────────────


def _serialize_detections(detections):
	out = {}
	for color, dets in detections.items():
		out[color] = []
		for d in dets:
			simplified = cv2.approxPolyDP(d["contour"], 2.0, True)
			out[color].append({"contour": simplified.reshape(-1, 2).tolist(), "area": float(d["area"]), "centroid": [int(d["centroid"][0]), int(d["centroid"][1])], "label": d["label"], "zone": d["zone"]})
	return out


def CameraProcess(red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, orange_l, blue_l, wall_left, wall_right, left_a, right_a, time_video, close_wall, red_area, green_area, outer_wall_left, outer_wall_right, centr_x_close, centr_y_close, head, fps_m, switch_state, park_wall, last_wall, last_wall_2):  # ← VL53L0X shared values (replaces tfmini)
	# time.sleep(1)

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
	frames_dir = None
	ts_log = None
	frame_idx = 0
	date_str = datetime.now().strftime("%d-%m-%y_%H-%M-%S")
	if cap is None:
		return

	if SHOW_WINDOW:
		pass
		WIN = "WRO 2026 — OpenCV Detection  |  Q quit"
		cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
		cv2.resizeWindow(WIN, FRAME_WIDTH, FRAME_HEIGHT)

	frame_count = 0

	try:
		while True:
			# time.sleep(0.01)
			ret, frame = cap.read()
			if not ret or frame is None:
				continue
			frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

			detections = pipeline.detect(frame)

			# ── Filter by zone ────────────────────────────────────────────────
			magenta_list = [d for d in detections["magenta"] if d["zone"] == "main"]
			red_blocks = [d for d in detections["red"] if d["zone"] == "main"]
			green_blocks = [d for d in detections["green"] if d["zone"] == "main"]

			magenta_list_close = [d for d in detections["magenta"] if d["zone"] == "close"]
			red_blocks_close = [d for d in detections["red"] if d["zone"] == "close"]
			green_blocks_close = [d for d in detections["green"] if d["zone"] == "close"]

			orange_detections = [d for d in detections["orange"] if d["zone"] == "line" or d["zone"] == "line_2"]
			blue_detections = [d for d in detections["blue"] if d["zone"] == "line" or d["zone"] == "line_2"]

			black_walls_left = [d for d in detections["black"] if d["zone"] == "wall_inner_left"]
			black_walls_right = [d for d in detections["black"] if d["zone"] == "wall_inner_right"]

			black_outer_walls_left = [d for d in detections["black"] if d["zone"] == "wall_left"]
			black_outer_walls_right = [d for d in detections["black"] if d["zone"] == "wall_right"]

			black_walls_close = [d for d in detections["black"] if d["zone"] == "close_black"]
			black_walls_park = [d for d in detections["black"] if d["zone"] == "parking_wall"]
			black_walls_last = [d for d in detections["black"] if d["zone"] == "last_wall"]
			black_walls_last2 = [d for d in detections["black"] if d["zone"] == "last_wall_2"]

			# Keep only the lowest (largest cy = closest) main block per colour
			if len(red_blocks) > 1:
				red_blocks = [max(red_blocks, key=lambda b: b["centroid"][1])]
			if len(green_blocks) > 1:
				green_blocks = [max(green_blocks, key=lambda b: b["centroid"][1])]

			if len(red_blocks_close) > 1:
				red_blocks_close = [max(red_blocks_close, key=lambda b: b["centroid"][1])]
			if len(green_blocks_close) > 1:
				green_blocks_close = [max(green_blocks_close, key=lambda b: b["centroid"][1])]

			red_present = bool(red_blocks)
			green_present = bool(green_blocks)
			pink_present = bool(magenta_list)

			red_present_close = bool(red_blocks_close)
			green_present_close = bool(green_blocks_close)
			pink_present_close = bool(magenta_list_close)

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
			wall_park_flag = bool(black_walls_park)
			last_wall_flag = bool(black_walls_last)
			last_wall_flag2 = bool(black_walls_last2)

			outer_left_flag = bool(black_outer_walls_left)
			outer_right_flag = bool(black_outer_walls_right)

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

			centr_x.value = 0.0
			centr_y.value = 0.0
			centr_x_red.value = 0.0
			centr_y_red.value = 0.0
			red_b.value = False
			green_b.value = False
			# ── Red + green simultaneously: biggest area wins ─────────────────
			if red_present and green_present:
				best_red = max(red_blocks, key=lambda b: b["area"])
				best_green = max(green_blocks, key=lambda b: b["area"])

				if best_green["centroid"][1] > best_red["centroid"][1]:
					centr_x.value = float(best_green["centroid"][0])
					centr_y.value = float(best_green["centroid"][1])
					green_area.value = True
					# if centr_x.value < 540:
					green_b.value = True
				else:
					centr_x_red.value = float(best_red["centroid"][0])
					centr_y_red.value = float(best_red["centroid"][1])
					red_area.value = True
					# if centr_x_red.value > 100:
					red_b.value = True

			elif red_present:
				best_red = max(red_blocks, key=lambda b: b["area"])
				centr_x_red.value = float(best_red["centroid"][0])
				centr_y_red.value = float(best_red["centroid"][1])
				# if centr_x_red.value > 100:
				red_b.value = True

			elif green_present:
				best_green = max(green_blocks, key=lambda b: b["area"])
				centr_x.value = float(best_green["centroid"][0])
				centr_y.value = float(best_green["centroid"][1])

				green_b.value = True

			# ── Close Blocks ───────────────────────────────────────────────────────
			centr_x_close.value = 0
			centr_y_close.value = 0
			if red_present_close:
				best_red_close = max(red_blocks_close, key=lambda b: b["area"])
				centr_x_close.value = float(best_red_close["centroid"][0])
				centr_y_close.value = float(best_red_close["centroid"][1])
				red_b.value = True

			elif green_present_close:
				best_green_close = max(green_blocks_close, key=lambda b: b["area"])
				centr_x_close.value = float(best_green_close["centroid"][0])
				centr_y_close.value = float(best_green_close["centroid"][1])
				green_b.value = True

			# print(f"close x:{centr_x_close.value} close y:{centr_y_close.value} r:{red_b.value} g:{green_b.value}")

			# ── Walls ─────────────────────────────────────────────────────────
			# print(f"wall_left: {wall_left_flag} wall_right: {wall_right_flag}")
			wall_left_area = 0
			wall_right_area = 0
			wall_left.value = 0.0
			wall_right.value = 0.0
			outer_wall_left.value = 0.0
			outer_wall_right.value = 0.0			
			close_wall.value = 0.0
			park_wall.value = 0.0
			last_wall.value = 0.0
			last_wall_2.value = 0.0

			if wall_left_flag and not wall_right_flag:
				best_black_left = max(black_walls_left, key=lambda b: b["area"])
				wall_left.value = best_black_left["centroid"][0]
				wall_right.value = 0.0
				left_a.value = best_black_left["area"]
			elif wall_right_flag and not wall_left_flag:
				best_black_right = max(black_walls_right, key=lambda b: b["area"])
				wall_right.value = best_black_right["centroid"][0]
				wall_left.value = 0.0
				right_a.value = best_black_right["area"]
			elif wall_left_flag and wall_right_flag:
				best_black_left = max(black_walls_left, key=lambda b: b["area"])
				best_black_right = max(black_walls_right, key=lambda b: b["area"])
				wall_left.value = best_black_left["centroid"][0]
				wall_right.value = best_black_right["centroid"][0]

			if wall_close_flag:
				best_black_close = max(black_walls_close, key=lambda b: b["area"])
				close_wall.value = best_black_close["area"]

			if wall_park_flag:
				best_black_park = max(black_walls_park, key=lambda b: b["area"])
				park_wall.value = best_black_park["area"]

			if last_wall_flag:
				best_last_wall = max(black_walls_last, key=lambda b: b["area"])
				last_wall.value = best_last_wall["area"]
			if last_wall_flag2:
				best_last_wall2 = max(black_walls_last2, key=lambda b: b["area"])
				last_wall_2.value = best_last_wall2["area"]

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

			# ── Annotation & display ──────────────────────────────────────────
			t_now = time.perf_counter()
			fps = 1.0 / max(t_now - t_prev, 1e-6)
			fps_m.value = fps
			t_prev = t_now
			frame_count += 1
			# print(f"left wall: {wall_left.value} right wall: {wall_right.value}")
			# print(f"r: {red_b.value} centr:{centr_x_red.value}")
			# print(f"left area : {left_a.value} right area: {right_a.value}")
			# annotated = pipeline.annotate(frame, detections, USE_LAB, fps, time_video.value, head.value)
			# recorder.write(annotated)
			if video_t > 0:
				time_video.value = time.time() - video_t
			if SHOW_WINDOW:  # only annotate every other frame to save CPU
				# annotated = pipeline.annotate(frame, detections, USE_LAB, fps, time_video.value, head.value)
				# cv2.imshow(WIN, annotated)
				pass
			key = cv2.waitKey(1) & 0xFF
			if key == ord("q"):
				break
			elif key == ord("s"):
				# cv2.imwrite(f"/home/pi/wro_logs/images/image_{date_str}.png", annotated)
				print("Snapshot saved!")
			"""if switch_state.value == 1 and not recording:
				date_str = datetime.now().strftime("%d-%m-%y_%H-%M-%S")
				frames_dir = f"/home/pi/wro_logs/videos/frames_{date_str}"
				os.makedirs(frames_dir, exist_ok=True)
				#ts_log = open(f"/home/pi/wro_logs/videos/timestamps_{date_str}.csv", "w")
				#dets_log = open(f"/home/pi/wro_logs/videos/detections_{date_str}.jsonl", "w")

				frame_idx = 0
				video_t = time.time()
				recording = True
				print(f"Recording started -> {frames_dir}")
			elif switch_state.value == 0 and recording:
				recording = False
				#ts_log.close()
				#dets_log.close()

				print(f"Recording stopped — {frame_idx} frames. Building video...")
				with DelayedKeyboardInterrupt():
					_build_video(date_str, frames_dir)"""
			"""if recording:
				cv2.imwrite(f"{frames_dir}/frame_{frame_idx:06d}.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
				ts_log.write(f"{frame_idx},{time.time():.6f},{head.value:.2f},{fps:.2f}\n")
				dets_log.write(json.dumps(_serialize_detections(detections)) + "\n")
				if frame_idx % 50 == 0:
					ts_log.flush()
					dets_log.flush()
				frame_idx += 1"""
	except KeyboardInterrupt:
		# ts_log.close()
		# dets_log.close()
		cap.release()
		# print(f"Recording stopped — {frame_idx} frames. Building video...")
		# _build_video(date_str, frames_dir)
		pass
	finally:
		"""if recording and ts_log is not None:
		ts_log.close()
		print(f"Recording interrupted — {frame_idx} frames saved (run _build_video manually)")"""
		cap.release()
		cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────────────────────
# PROCESS 2 — servoDrive  (main control loop)
# ─────────────────────────────────────────────────────────────────────────────


def DriveProcess(red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, orange_l, blue_l, wall_left, wall_right, left_a, right_a, time_video, close_wall, red_area, green_area, outer_wall_left, outer_wall_right, centr_x_close, centr_y_close, tof_front, tof_left, tof_right, tof_rear, fps_m, switch_state, park_wall, last_wall, last_wall_2):  # ← VL53L0X shared values (replaces tfmini)
	global kp_s, kd_s  # add this near your other `global` lines at the top
	time.sleep(3)
	pwm = pigpio.pi()
	global servo
	servo = Servo(servo_pin)

	if not pwm.connected:
		print("Could not connect to pigpio daemon")
		exit(1)

	for pin in [blue_led, red_led, green_led]:
		pwm.set_mode(pin, pigpio.OUTPUT)
		pwm.write(pin, 0)
	pwm.set_mode(switch_pin, pigpio.INPUT)
	pwm.set_pull_up_down(switch_pin, pigpio.PUD_DOWN)
	pwm.set_mode(reset_pin, pigpio.OUTPUT)

	tfmini = TFmini(17, 27, 25, 24)
	"""print(f"resetting.....")
	pwm.write(reset_pin, 0)
	time.sleep(0.25)
	pwm.write(reset_pin, 1)
	print(f"reset complete!")"""
	PWM_FREQ = 10000
	# MDD3A motor pin setup
	pwm.set_mode(PIN_A, pigpio.OUTPUT)
	pwm.set_mode(PIN_B, pigpio.OUTPUT)
	pwm.set_PWM_dutycycle(PIN_A, 0)
	pwm.set_PWM_dutycycle(PIN_B, 0)
	pwm.set_PWM_frequency(PIN_A, PWM_FREQ)
	pwm.set_PWM_frequency(PIN_B, PWM_FREQ)
	global imu, corr, corr_pos

	# enc = EncoderCounter()

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
	power = 90
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
	servo.setAngle(60)
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
	block_time = time.time()

	err = 0
	prev_err = 0
	orange_detected = False
	blue_detected = False
	orange_vanished = False
	blue_vanished = False
	before_stop_routine = False
	map_time = 0
	video_t = 0
	closest_heading = 0
	centroid = 0
	obstacle_state = 1
	default_turn_counter = 12
	new_sp = 0
	prev_loop_time_main = time.time()
	stuck = False
	while pwm.read(switch_pin) == 1:
		print("make sure the toggle is off")
	toggle = False
	try:
		while True and not toggle:
			tfmini.getTFminiData()
			load_gains_if_changed()
			imu_head = head.value
			now_main = time.time()
			dt_main = now_main - prev_loop_time_main
			dt_main = max(dt_main, 1e-3)  # avoid divide-by-zero
			dt_main = min(dt_main, 0.2)  # clamp so a stall doesn't spike the derivative
			prev_loop_time_main = now_main
			# ── Read all distance sensors ─────────────────────────────────────
			tf_l = tfmini.distance_head
			tf_h = tof_left.value
			tf_r = tof_right.value

			# x, y = enc.get_position(head.value, counts.value)

			if not init:
				if pink_b.value or red_b.value or green_b.value:
					correctAngle(heading_angle, head.value, 1.5)
					init = True
			# print(f"heading: {head.value} closest sp:{get_closest_setpoint(head.value)}")

			#########################################################################

			#######################################################################

			if not inParkingatStart and not orange_flag and not blue_flag:
				if ((tf_l < tf_r and tf_l > 0) or (outer_wall_left.value > 0 and not outer_wall_right.value > 0)) and head.value > 0:
					print("Right side parking")
					orange_flag = True
					blue_flag = False
					inParkingatStart = True
				elif ((tf_r < tf_l and tf_r > 0) or (outer_wall_right.value > 0 and not outer_wall_left.value > 0)) and head.value > 0:
					print("Left side parking")
					orange_flag = False
					blue_flag = True
					inParkingatStart = True
			# orange_flag = True

			# print(f"parkingatStart: {inParkingatStart} orange:{orange_flag} blue:{blue_flag}")
			# map_time = map_range(tfmini.distance_left, 0, 100, 0, 2)

			# print(f"orange:{orange_l.value} blue:{blue_l.value} left:{tf_l} map_time: {map_time}")

			# print(f"wall left:{wall_left.value} wall right: {wall_right.value}")
			"""if time.time() - last_time > debounce_delay:
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
					power = 95"""
			if pwm.read(switch_pin) == 1:
				button = True
				power = 90
			elif pwm.read(switch_pin) == 0:
				button = False
				power = 0
			switch_state.value = button

			if not button:
				# print(f"last_wall: {last_wall.value} last_wall_2: {last_wall_2.value} imu :{head.value}")
				# print(f"imu:{head.value:.2f} r: {red_b.value} g:{green_b.value} p:{pink_b.value} fps:{fps_m.value:.2f} park_wall:{park_wall.value}")
				print(f" rgb: [{red_b.value}]left: {tfmini.distance_head} orange_flag:{orange_flag} start:{inParkingatStart} imu:{head.value:.2f}")
				pass

			if button:
				print(f"[CALIB] t={time.time():.4f} counts={counts.value}")
				print("-------------------------------------------------")
				print("Switch is ON")

				if red_b.value or green_b.value:
					power = 75
					# block_time = time.time()
				elif not red_b.value and not green_b.value:
					power = 90
					# block_time = 0
				# x, y = enc.get_position(imu_head, counts.value)
				# inParkingatStart = False
				if inParkingatStart:
					prev_time = time.time()
					'''while time.time() - prev_time < 0.3:
						if orange_flag:
							servo.setAngle(115)
						elif blue_flag:
							servo.setAngle(65)
						duty = correctSpeed(40, counts.value, 1)
						total_power = duty*0.01 + prev_power *0.99
						runMotor(pwm, total_power, 1)
						prev_power = total_power
						print(f"correction: {abs(corr)} duty:{duty:.2f} 1")
					runMotor(pwm, 0, 1)
					resetSpeedPID(counts.value)

					prev_time = time.time()
					while time.time() - prev_time < 0.3:
						if orange_flag:
							servo.setAngle(80)
						elif blue_flag:
							servo.setAngle(100)
						duty = correctSpeed(30, counts.value, 0)
						total_power = duty*0.01 + prev_power *0.99
						runMotor(pwm, total_power, 0)
						prev_power = total_power
						print(f"correction: {abs(corr)} duty:{duty:.2f} 2")
					prev_time = time.time()
					resetSpeedPID(counts.value)

					while time.time() - prev_time < 0.3:

						if orange_flag:
							servo.setAngle(115)
						elif blue_flag:
							servo.setAngle(65)
						duty = correctSpeed(40, counts.value, 1)
						total_power = duty*0.01 + prev_power *0.99
						runMotor(pwm, total_power, 1)
						prev_power = total_power
						print(f"correction: {abs(corr)} duty:{duty:.2f} 1")
					resetSpeedPID(counts.value)

					prev_time = time.time()
					while time.time() - prev_time < 0.3:

						if orange_flag:
							servo.setAngle(65)
						elif blue_flag:
							servo.setAngle(115)
						duty = correctSpeed(30, counts.value, 0)
						total_power = duty*0.01 + prev_power *0.99
						runMotor(pwm, total_power, 0)
						prev_power = total_power
					resetSpeedPID(counts.value)

					print(f"correction: {abs(corr)} duty:{duty:.2f} 2")'''
					if orange_flag:
						correctAngle(heading_angle + 60, head.value, 3)
					elif blue_flag:
						correctAngle(heading_angle - 70, head.value, 3)

					while abs(corr) > 2 and not is_switch_off_and_stop(pwm):
						duty = correctSpeed(50, counts.value, 1)
						total_power = duty*0.001 + prev_power *0.999
						runMotor(pwm, total_power, 1)
						prev_power = total_power
						if orange_flag:
							correctAngle(heading_angle + 60, head.value, 3)
						elif blue_flag:
							correctAngle(heading_angle - 70, head.value, 3)

						print(f"correction: {abs(corr)} duty:{duty:.2f} 3")
						# runMotor(pwm, 70, 1)
					runMotor(pwm, 0, 1)

					inParkingatStart = False

				else:

					if parking_flag:
						print(f"PARKING -|-----> distance_head : {tf_h}")
						print("Inside Parking Loop")
						# refresh sensor readings inside parking loop
						tf_h = tof_front.value
						tf_l = tof_left.value
						tf_r = tof_right.value
						if not calc_time:
							c_time = time.time()
							calc_time = True

						if STATE == 1:
							if blue_flag or orange_flag:
								'''while last_wall.value > 1550:
									correctReverseAngle(heading_angle, head.value, 1.5)
									driveAtSpeed(pwm, 35, 0, counts.value)
								resetSpeedPID(counts.value)
								while last_wall.value < 1580:
									correctAngle(heading_angle, head.value, 1.5)
									driveAtSpeed(pwm, 30, 1, counts.value)
								resetSpeedPID(counts.value)'''

								while not pink_b.value:
									correctReverseAngle(heading_angle, head.value, 1.5)
									driveAtSpeed(pwm, 35, 0, counts.value)
								resetSpeedPID(counts.value)
								last_counter
								if last_wall.value < 1500:
									target_count = counts.value + 700
								else:
									target_count = 0
								while counts.value < target_count:
									correctAngle(heading_angle, head.value, 1.5)
									driveAtSpeed(pwm, 30, 1, counts.value)									
								#target_count = counts.value - 20
								'''target_count = counts.value + 100
#								while last_wall.value > 1500:
								while counts.value < target_count:

									correctAngle(heading_angle, head.value, 1.5)
									driveAtSpeed(pwm, 30, 1, counts.value)

								resetSpeedPID(counts.value)'''
								
								'''target_count = counts.value - 150
#								while last_wall.value > 1500:
								while counts.value > target_count:

									correctReverseAngle(heading_angle, head.value, 1.5)
									driveAtSpeed(pwm, 30, 0, counts.value)

								resetSpeedPID(counts.value)'''

								STATE = 2

						if STATE == 2:

							if orange_flag or blue_flag:
								"""tf_h = tof_front.value
								tf_l = tof_left.value
								tf_r = tof_right.value"""
								if parking_right:
									heading_angle -= 90
									parking_distance = tf_l
								elif parking_left:
									heading_angle += 90

									parking_distance = tf_r
								correctReverseAngle(heading_angle, head.value, 3)
								while (abs(corr) > 5) and not is_switch_off_and_stop(pwm):
									tf_h = tof_front.value
									tf_l = tof_left.value
									tf_r = tof_right.value

									parking_distance = tf_l if parking_right else tf_r
									print(f"corr:{abs(corr)} head:{tf_h} left:{tf_l}")
									driveAtSpeed(pwm, 30, 0, counts.value)
									correctReverseAngle(heading_angle, head.value, 3)
								prev_time = time.time()
								while not pink_b.value:
									driveAtSpeed(pwm, 30, 0, counts.value)
									correctReverseAngle(heading_angle, head.value, 3)							

								print(f"last wall:{last_wall.value} last wall 2:{last_wall_2.value}")
							final_park = time.time()
							full_park = True
							resetSpeedPID(counts.value)
							STATE = 4

						if STATE == 3:
							if full_park:
								print("Doing the full park..")
								power = 30
								prev_pow = 0
								prev_pow = 0
								prev_time = time.time()
								while(time.time() - prev_time < 0.65):
									driveAtSpeed(pwm, 35, 1, counts.value)
									#servo.setAngle(80)
									if parking_right:
										#servo.setAngle(85)
										servo.setAngle(105)
									elif parking_left:
										#servo.setAngle(105)
										servo.setAngle(85)
									'''if time.time() - prev_time > 0.15:
										if parking_right:
											servo.setAngle(85)
										elif parking_left:
											servo.setAngle(105)
									else:
										servo.setAngle(95)'''
								driveAtSpeed(pwm, 0, 1, counts.value)

								correctReverseAngle(heading_angle - 90, head.value, 3)
								final_park = time.time()
								prev_time = time.time()


								final_park = time.time()
								prev_time = time.time()
							driveAtSpeed(pwm, 0, 0, counts.value)
							correctReverseAngle(heading_angle - 90, head.value, 3)

							resetSpeedPID(counts.value)
							STATE = 4

						if STATE == 4:
							count_thresh = 400
							heading_thresh = 60
							PID_thresh = 1.5
							if centr_x_pink.value < 320:
								count_thresh = 600
								heading_thresh = 80
								PID_thresh = 3
							target_count = counts.value + count_thresh
#								while last_wall.value > 1500:
							while counts.value < target_count:

								correctAngle(heading_angle + heading_thresh, head.value, PID_thresh)
								driveAtSpeed(pwm, 30, 1, counts.value)

							resetSpeedPID(counts.value)							
							correctReverseAngle(heading_angle + 90, head.value, 3)
							prev_time = time.time()
							while abs(corr) > 8 and not is_switch_off_and_stop(pwm):
								tf_h = tof_front.value
								print(f"ASASSAD")
								print(f"corr:{abs(corr)} head:{tf_h}")
								driveAtSpeed(pwm, 30, 0, counts.value)
								#correctReverseAngle(heading_angle - 90, head.value, 3) ###############################################################################################################
								correctReverseAngle(heading_angle + 90, head.value, 3)
							stuck = False
							resetSpeedPID(counts.value)								

							while centr_y_pink.value < 235:
								print(f"MAKIBNG FINAL ADJUSTMENT")
								#correctAngle(heading_angle - 90, head.value, 3) ###############################################################################################################
								correctAngle(heading_angle + 90, head.value, 3)
								driveAtSpeed(pwm, 30, 1, counts.value)

							power = 0
							prev_power = 0
							pwm.set_PWM_dutycycle(PIN_A, 0)
							pwm.set_PWM_dutycycle(PIN_B, 0)
							servo.setAngle(95)
							sys.exit(0)

					else:

						##################################### HANDLE TURNS ########################################

						if not counter_flag and trigger:
							if time.time() - prev_time > 0:
								counter += 1
								counter_flag = True
								heading_angle = update_heading(counter, heading_angle, blue_flag, orange_flag)

						##################################### HANDLE OBSTACLE AVOIDANCE ###########################

						centroid = 0
						centroid_pink = 0
						err = 0
						multiplier = 1
						if green_b.value and (not red_b.value and time.time() - r_time > 0.01) and not lap_finish:
							g_time = time.time()
							r_time = 0
							centroid = centr_x.value
							print("default green following")
							if obstacle_state == 1:
								print(f"center greeen...")
								if blue_flag:
									if wall_left.value > 0 or outer_wall_left.value > 0:
										err = 320 - centroid
									else:
										err = 340 - centroid
									
								else:									
									err = 340 - centroid
							elif obstacle_state == 2:
								print(f"left greeen...")
								if blue_flag:
									if wall_left.value > 0 or outer_wall_left.value > 0:
										err = 470 - centroid
									else:
										err = 520 - centroid
									
								else:									
									err = 520 - centroid
								#err = 520 - centroid

							'''pwm.write(red_led, 0)
							pwm.write(green_led, 1)'''
						elif red_b.value and (not green_b.value and time.time() - g_time > 0.01) and not lap_finish:
							r_time = time.time()
							g_time = 0
							centroid = centr_x_red.value
							print("default red following")
							if obstacle_state == 1:
								print(f"center red...")
								if orange_flag:
									if wall_right.value > 0 or outer_wall_right.value > 0:
										err = 320 - centroid
									else:
										err = 300 - centroid
								else:									
									err = 290 - centroid
							elif obstacle_state == 2:
								print(f"right red...")
								if orange_flag:
									if wall_right.value > 0 or outer_wall_right.value > 0:
										err = 170 - centroid
									else:
										err = 120 - centroid
								else:									
									err = 100 - centroid
								#err = 120 - centroid

							'''pwm.write(red_led, 1)
							pwm.write(green_led, 0)'''
						elif not red_b.value and not green_b.value and not lap_finish:
							if not outer_wall_left.value > 0 and not outer_wall_right.value > 0:
								if wall_left.value > 0 and not wall_right.value > 0:
									print(f"corrrecting left wall...")
									err = -15
								elif wall_right.value > 0 and not wall_left.value > 0:
									print(f"corrrecting right wall...")
									err = 15
							centroid = 0
							centroid_pink = 0
							'''pwm.write(red_led, 0)
							pwm.write(green_led, 0)
							pwm.write(blue_led, 0)'''

						# print(f"red_b.value: {red_b.value} green_b.value:{green_b.value}")
						# print(f"both diff: {abs(r_time - g_time)}  r_time diff: {time.time() - r_time:.2f} g_time: {time.time() - g_time:.2f} ")
						if pink_b.value and continue_parking:
							p_time = time.time()
							centroid_pink = centr_x_pink.value
							if parking_right:
								err = 540 - centroid_pink
							elif parking_left:
								err = 125 - centroid_pink
							#pwm.write(blue_led, 1)
						##################################################### PID CENTROID ############################################

						servo_angle = err * kp_s + (((err - prev_err) / dt_main) * kd_s)
						servo_angle = max(-30, min(30, servo_angle))
						prev_err = err

						# servo.setAngle(90 - servo_angle)
						print(f"original angle: {servo_angle:.2f}")
						# print(f"servo_angle: {servo_angle}")
						print(f"r_time diff:{time.time() - r_time} r_time: {r_time} g_time diff: {time.time() - g_time} g_time: {g_time}")
						# print(f"multiplier:{multiplier} centroid: {centroid} err:{err} servo_angle:{servo_angle}  black left: {left_a.value} black_right: {right_a.value}")'''
						print(f"left wall cx:{wall_left.value} right wal cx:{wall_right.value} outer left:{outer_wall_left.value} outer right: {outer_wall_right.value}")
						print(f"red centroid: {centr_x_red.value:.2f} {centr_y_red.value:.2f} green centroid: {centr_x.value:.2f} {centr_y.value:.2f}")
						# print(f"centroid : {centroid}")

						##########################################################################################################################
						if not lap_finish:

							if close_wall.value > 0 and counter == 0 and blue_flag:
								correctAngle(heading_angle, head.value, 1.5)

								while abs(corr) > 25 and not is_switch_off_and_stop(pwm):
									print("printing this")
									duty = correctSpeed(power, counts.value, 1)
									runMotor(pwm, duty, 1)
									if counter != last_counter:
										if orange_l.value and orange_flag and not trigger:
											prev_time = time.time()
											map_time = map_range(tf_l, 0, 100, 0, 0.7)
											trigger = True
											counter_flag = False
										elif blue_l.value and blue_flag and not trigger:
											map_time = map_range(tf_r, 0, 100, 0, 0.7)
											prev_time = time.time()
											trigger = True
											counter_flag = False
										elif time.time() - prev_time > 2 and trigger:
											trigger = False
									correctAngle(heading_angle, head.value, 1.5)
									prev_time = time.time()
									trigger = True
									counter_flag = False

							else:
								pass
								# power = 90

							if obstacle_state == 1:
								if ((green_b.value) or (red_b.value)) and not (centr_x_close.value > 0) and ((centr_y.value < 100 and centr_y.value > 0) or (centr_y_red.value < 100 and centr_y_red.value > 0)):
									# print("Avo iding obstacle")
									print("Avoiding obstacle")
									# block_time = time.time()
									servo.setAngle(95 - servo_angle)
								elif centr_x_close.value > 0 or (centr_y.value > 100 or centr_y_red.value > 100):
									block_time = time.time()
									print(f"shifting to state 2..")
									new_sp = head.value
									obstacle_state = 2

								else:
									print("state 1 default")
									if not green_b.value and not red_b.value:
										print(f"no green or red detected, correcting heading...")
										if (outer_wall_left.value > 0) and not outer_wall_right.value > 0 and not close_wall.value > 0:
											print(f"correcting outer left wall...")
											correctAngle(heading_angle + 5, head.value, 1.5)
										elif outer_wall_right.value > 0 and not outer_wall_left.value > 0 and not close_wall.value > 0:
											print(f"correcting outer right wall...")
											correctAngle(heading_angle - 5, head.value, 1.5)
										else:
											print(f"state 1 correcting no/both walls")
											correctAngle(heading_angle, head.value, 1.5)
									

							elif obstacle_state == 2:
								# power = 90
								print(f"state 2")
								print(f"state 2 time :{time.time() - block_time}")

								if time.time() - block_time < 0.08:
									print(f"avoding obstacle_state == 2")
									if centr_y_pink.value > 100 or close_wall.value > 50 :
										print(f"either pink is near or close wall is near shifting back to state 1")
										obstacle_state = 1

									else:
										print(f"still avoding block for {0.05} sec")
										servo.setAngle(95 - servo_angle)

								else:
									print(f"shifting back to 1")
									obstacle_state = 1

							##################################### HANDLE TURNS ########################################

							if counter != last_counter:
								if orange_l.value and orange_flag and not trigger:
									prev_time = time.time()
									map_time = map_range(tf_l, 0, 100, 0, 0.7)
									trigger = True
									counter_flag = False
								elif blue_l.value and blue_flag and not trigger:
									map_time = map_range(tf_r, 0, 100, 0, 0.7)
									prev_time = time.time()
									trigger = True
									counter_flag = False
								elif time.time() - prev_time > 2 and trigger:
									trigger = False

							############################ CHECK PARKING STATUS #########################################

							if counter == last_counter:
								print("REACHED MAXIMUM COUNTS")
								if not before_stop_routine:
									prev_heading = 0
									if orange_flag:
										prev_heading = 270
									elif blue_flag:
										prev_heading = -270
									correctAngle(prev_heading, head.value, 1)
									prev_time = time.time()
									while (abs(corr) > 25 or time.time() - prev_time < 1.7) and not is_switch_off_and_stop(pwm):
										print(f"find closest sp {prev_heading} {abs(corr):.2f} {heading_angle} {head.value:.2f} {time_video.value:.2f}")
										driveAtSpeed(pwm, 50, 1, counts.value)

										correctAngle(prev_heading, head.value, 1)

									prev_time = time.time()
									while (not close_wall.value > 0) and not is_switch_off_and_stop(pwm):
										print(f"find closest sp {prev_heading} {abs(corr):.2f} {heading_angle} {head.value:.2f} {time.time() - prev_time:.2f} ABC")
										driveAtSpeed(pwm, 50, 1, counts.value)
										if time.time() - prev_time > 0.5:
											break
										correctAngle(prev_heading, head.value, 1)

									prev_time = time.time()
									correctReverseAngle(heading_angle, head.value, 3)
									# prev_power  = 0
									while time.time() - prev_time < 3:#abs(corr) > 5 and not is_switch_off_and_stop(pwm):
										print(f"reversing corr {heading_angle} {abs(corr):.2f} {heading_angle} {head.value:.2f} {time_video.value:.2f} {(time.time() - prev_time):.2f}")
										driveAtSpeed(pwm, 50, 0, counts.value)
										'''if time.time() - prev_time > 3:
											break'''
										correctReverseAngle(heading_angle, head.value, 3)
									before_stop_routine = True
									power = 70
									prev_power = 0
									resetSpeedPID(counts.value)

								if not finished:
									if orange_flag:
										if parking_right:
											finish_thresh = 1400
											before_finish = 0
											target_count = counts.value + 18200
										elif parking_left:
											before_finish = 0
											finish_thresh = 1900
											target_count = counts.value + 13800
									elif blue_flag:
										if parking_right:
											finish_thresh = 1900
											before_finish = 0
											target_count = counts.value + 13800
										elif parking_left:
											finish_thresh = 1400
											before_finish = 0
											target_count = counts.value + 18200
									finished = True

								if (counts.value >= target_count) or (counts.value >= target_count - before_finish):
									power = 0
									prev_power = 0
									pink_b.value = False
									runMotor(pwm, power, 1)
									time.sleep(2)
									pink_b.value = False
									prev_power = 0
									lap_finish = True
									print("Vehicle is stopped...")

						if lap_finish and not continue_parking:
							if not counter_reset:
								counter = counter % last_counter
								counter_reset = True
							if orange_flag:
								print("Correcting wall pid orange")
								if parking_STATE == 1:
									
									heading_angle += 90
									correctReverseAngle(heading_angle, head.value, 3)
									while abs(corr) > 5 and not is_switch_off_and_stop(pwm):
										# x, y = enc.get_position(head.value, counts.value)
										tf_h = tof_front.value
										power = 70
										prev_power = 0
										driveAtSpeed(pwm, 40, 0, counts.value)

										print(f"mid turn  {abs(corr)} {park_wall.value}")
										correctReverseAngle(heading_angle, head.value, 3)
									power = 40
									# prev_power = 20
									resetSpeedPID(counts.value)
									prev_time = time.time()
									while time.time() - prev_time < 1:
										# x, y = enc.get_position(head.value, counts.value)
										duty = correctSpeed(40, counts.value, 0)
										total_power = duty * 0.01 + prev_power * 0.99
										runMotor(pwm, total_power, 0)
										prev_power = total_power
										correctReverseAngle(heading_angle, head.value, 1)
										print(f"p state 1 {park_wall.value}")
									prev_power = 0
									while (park_wall.value > 3100) and not is_switch_off_and_stop(pwm):
										# x, y = enc.get_position(head.value, counts.value)
										duty = correctSpeed(40, counts.value, 0)
										total_power = duty * 0.01 + prev_power * 0.99
										runMotor(pwm, total_power, 0)
										prev_power = total_power
										correctReverseAngle(heading_angle, head.value, 1)
										print(f"p state 1 {park_wall.value}")
									runMotor(pwm, 0, 0)

									prev_power = 0
									while (park_wall.value < 3100) and not is_switch_off_and_stop(pwm):
										# x, y = enc.get_position(head.value, counts.value)
										duty = correctSpeed(power, counts.value, 1)
										total_power = duty * 0.01 + prev_power * 0.99
										runMotor(pwm, total_power, 1)
										prev_power = total_power
										correctAngle(heading_angle, head.value, 1)
										print(f"p sltate 1 {park_wall.value}")

									resetSpeedPID(counts.value)
									driveAtSpeed(pwm, 0, 0, counts.value)

									resetSpeedPID(counts.value)
									parking_STATE = 2
								if parking_STATE == 2:
									if parking_right:
										heading_angle += 95
									elif parking_left:
										heading_angle -= 95
									correctReverseAngle(heading_angle, head.value, 3)
									power = 40
									# prev_power = 50
									print("corr is", abs(corr))
									while abs(corr) > 5 and not is_switch_off_and_stop(pwm):
										# x, y = enc.get_position(head.value, counts.value)
										# tf_h = tof_front.value
										duty = correctSpeed(50, counts.value, 0)
										total_power = duty * 0.01 + prev_power * 0.99
										runMotor(pwm, total_power, 0)
										prev_power = total_power
										correctReverseAngle(heading_angle, head.value, 3)
										print("P state 2")
									p_flag = True
									continue_parking = True
									parking_STATE = 3
									power = 50
									prev_power = 0
									resetSpeedPID(counts.value)
									pink_time = time.time()

							elif blue_flag:
								print("Correcting wall pid blue")
								heading_angle -= 90
								if parking_STATE == 1:
									correctReverseAngle(heading_angle, head.value, 3)
									while abs(corr) > 5 and not is_switch_off_and_stop(pwm):
										# x, y = enc.get_position(head.value, counts.value)
										tf_h = tof_front.value
										power = 70
										prev_power = 0
										driveAtSpeed(pwm, 40, 0, counts.value)

										print(f"mid turn  {abs(corr)} {park_wall.value}")
										correctReverseAngle(heading_angle, head.value, 3)
									power = 40
									# prev_power = 20
									resetSpeedPID(counts.value)
									prev_time = time.time()
									while time.time() - prev_time < 1:
										# x, y = enc.get_position(head.value, counts.value)
										duty = correctSpeed(40, counts.value, 0)
										total_power = duty * 0.01 + prev_power * 0.99
										runMotor(pwm, total_power, 0)
										prev_power = total_power
										correctReverseAngle(heading_angle, head.value, 1)
										print(f"p state 1 {park_wall.value}")
									prev_power = 0
									while (park_wall.value > 3100) and not is_switch_off_and_stop(pwm):
										# x, y = enc.get_position(head.value, counts.value)
										duty = correctSpeed(40, counts.value, 0)
										total_power = duty * 0.01 + prev_power * 0.99
										runMotor(pwm, total_power, 0)
										prev_power = total_power
										correctReverseAngle(heading_angle, head.value, 1)
										print(f"p state 1 {park_wall.value}")
									runMotor(pwm, 0, 0)
									time.sleep(0.2)
									while (park_wall.value < 3100) and not is_switch_off_and_stop(pwm):
										# x, y = enc5get_position(head.value, counts.value)
										duty = correctSpeed(power, counts.value, 1)
										total_power = duty * 0.01 + prev_power * 0.99
										runMotor(pwm, total_power, 1)
										prev_power = total_power
										correctAngle(heading_angle, head.value, 1)
										print(f"p sltate 1 {park_wall.value}")

									resetSpeedPID(counts.value)
									driveAtSpeed(pwm, 0, 0, counts.value)

									resetSpeedPID(counts.value)
									parking_STATE = 2
								if parking_STATE == 2:
									if parking_right:
										heading_angle += 95
									elif parking_left:
										heading_angle -= 95
									correctReverseAngle(heading_angle, head.value, 3)
									power = 50
									# prev_power = 50
									print("corr is", abs(corr))
									while abs(corr) > 5 and not is_switch_off_and_stop(pwm):
										# x, y = enc.get_position(head.value, counts.value)
										# tf_h = tof_front.value
										duty = correctSpeed(40, counts.value, 0)
										total_power = duty * 0.01 + prev_power * 0.99
										runMotor(pwm, total_power, 0)
										prev_power = total_power
										correctReverseAngle(heading_angle, head.value, 3)
										print("P state 2")
									p_flag = True
									continue_parking = True
									parking_STATE = 3
									power = 50
									prev_power = 0
									resetSpeedPID(counts.value)
									pink_time = time.time()

						if continue_parking and not parking_flag:
							# correctAngle(heading_angle, head.value, 1)
							power = 40


							if last_wall.value > 1570 and (abs(heading_angle - head.value) < 15 or abs(heading_angle - head.value) > 345) and not pink_b.value:
								runMotor(pwm, 0, 1)
								time.sleep(0.5)
								print(f"parking_flag is true")
								parking_flag = True
								resetSpeedPID(counts.value)

							if not pink_b.value or (centr_y_pink.value > 150 and centr_x_pink.value > 530):
								print(f"correcting angle pink is not seen {abs(heading_angle - head.value):.2f}")
								correctAngle(heading_angle, head.value, 1)

							else:
								servo_angle = servo_angle + servo_angle * 3
								servo_angle = max(-25, min(25, servo_angle))
								print(f"Followuing pink wall... {servo_angle:.2f} corr:{abs(heading_angle - head.value):.2f}")
								servo.setAngle(95 - servo_angle)

						###########################################################################################################

				duty = correctSpeed(power, counts.value, 1)
				total_power = duty * 0.01 + prev_power * 0.99
				runMotor(pwm, total_power, 1)
				prev_power = total_power
				print(f"fps:{fps_m.value}")
				print(f"state:{obstacle_state} DUTY:{duty:.2f} target_power:{power:.2f}")
				print(f"close x: {centr_x_close.value} close y: {centr_y_close.value} close wall: {close_wall.value} park_wall: {park_wall.value}")
				print(f"centroid val: {centroid} servo_angle = {servo_angle:.2f} err = {err}")
				print(f"pink: {pink_b.value} green:{green_b.value} red:{red_b.value} ")
				print(f"tigger :{trigger} counter_flag: {counter_flag} block_time :{time.time() - block_time:.2f} ")
				

				# print(f"OBSTACLE_STATE:{OBSTACLE_STATE} RESET_STATE:{RESET_STATE}")
				print("---------------------------------------------------")

			else:
				# print(f"Switch is off {head.value}")
				power = 0
				pwm.set_PWM_dutycycle(PIN_A, 0)
				pwm.set_PWM_dutycycle(PIN_B, 0)
				counter = 0
				heading_angle = 0
				prev_power = 0
				resetSpeedPID(counts.value)
				correctAngle(heading_angle, head.value, 1.5)

			if exit_flag and button:
				print("Shutting down the program")
				sys.exit(0)
			# time.sleep(0.0002)

	except Exception as e:
		print(f"Exception: {e}")
		tb = traceback.extract_tb(e.__traceback__)
		filename, lineno, func, text = tb[-1]
		print(f"⚠️ Exception in {filename}, line {lineno}, in {func}")
		if isinstance(e, KeyboardInterrupt):
			power = 0
			pwm.set_PWM_dutycycle(PIN_A, 0)
			pwm.set_PWM_dutycycle(PIN_B, 0)
			heading_angle = 0
			counter = 0
			correctAngle(heading_angle, head.value, 1.5)
			red_b.value = False
			green_b.value = False
	finally:
		pwm.set_PWM_dutycycle(PIN_A, 0)
		pwm.set_PWM_dutycycle(PIN_B, 0)
		print("Motors stopped safely.")
		pwm.stop()


# ─────────────────────────────────────────────────────────────────────────────
# PROCESS 3 — Encoder / IMU reader
# ─────────────────────────────────────────────────────────────────────────────
"""def reset_esp32():
	print("Sending software reset to ESP32...")
	ser.write(b"R")
	time.sleep(1)              # wait for ESP32 to fully reboot and BNO085 to init
	ser.reset_input_buffer()   # flush boot garbage
	print("ESP32 reset complete")"""


def IMUandEncoder(counts, head):
	ser = serial.Serial("/dev/XIAO_USB", 115200, timeout=0.1)

	print("Sending software reset to ESP32...")
	ser.write(b"R")
	time.sleep(1)  # wait for ESP32 to fully reboot and BNO085 to init
	ser.reset_input_buffer()  # flush boot garbage
	print("ESP32 reset complete")

	ser.write(b"1")  # reset encoder counts

	print("Command sent: b'1'")

	ser.reset_input_buffer()  # flush boot garbage
	try:
		while True:
			line = ser.readline().decode("utf-8", errors="ignore").strip()
			esp_data = line.split()
			# print(esp_data)
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

		T = multiprocessing.Process(target=TOFProcess, args=(tof_front, tof_left, tof_right, tof_rear))

		P = multiprocessing.Process(target=CameraProcess, args=(red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, orange_l, blue_l, wall_left, wall_right, left_a, right_a, time_video, close_wall, red_area, green_area, outer_wall_left, outer_wall_right, centr_x_close, centr_y_close, head, fps_m, switch_state, park_wall, last_wall, last_wall_2))  # ← VL53L0X shared values (replaces tfmini)
		S = multiprocessing.Process(target=DriveProcess, args=(red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink, centr_y_pink, head, orange_l, blue_l, wall_left, wall_right, left_a, right_a, time_video, close_wall, red_area, green_area, outer_wall_left, outer_wall_right, centr_x_close, centr_y_close, tof_front, tof_left, tof_right, tof_rear, fps_m, switch_state, park_wall, last_wall, last_wall_2))  # ← VL53L0X shared values (replaces tfmini)
		E = multiprocessing.Process(target=IMUandEncoder, args=(counts, head))
		P.start()
		E.start()
		S.start()
		T.start()

	except KeyboardInterrupt:
		# ser.close()
		E.terminate()
		S.terminate()
		P.terminate()
		T.terminate()
		E.join()
		S.join()
		P.join()
		T.join()
		# tof.close()
