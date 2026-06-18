"""
Obstacle_Challenge_OpenCV.py  —  WRO 2025  |  OpenCV HSV Detection
====================================================================
Full obstacle-challenge rewrite with:
  - EdgeTPU / pycoral model REMOVED
  - Pure OpenCV HSV detection (ported from detection_opencv_clean.py)
  - Dual telemetry: clean VS Code DEBUG CONSOLE panel + rich terminal
  - All driving/parking/avoidance/PID logic preserved 1-to-1

PROCESSES
  P  → Live_Feed_OpenCV     (camera → shared multiprocessing values)
  S  → servoDrive           (main driving brain)
  E  → runEncoder           (serial IMU + encoder reader)
  L  → read_lidar           (RPLidar subprocess reader)

SSH / VS Code — see bottom of file for lines to comment.

RUN (on Pi):
    export DISPLAY=:0
    export QT_QPA_PLATFORM=xcb          # comment out if running headless via SSH
    source /home/pi/coral-py39-env/bin/activate
    python3 /home/pi/WRO_2025_PI/Obstacle_Challenge_OpenCV.py
"""

# ─────────────────────────────────────────────────────────────────────────────
#  STANDARD IMPORTS
# ─────────────────────────────────────────────────────────────────────────────
import os
import sys
import time
import math
import serial
import logging
import datetime
import traceback
import subprocess
import multiprocessing
from collections import deque
from ctypes import c_float

import cv2
import numpy as np
import pigpio

from Encoder import EncoderCounter
from Servo import Servo
from TFmini import TFmini
from datetime import datetime


# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING  (set up FIRST so even daemon errors are captured)
# ─────────────────────────────────────────────────────────────────────────────
timestamp = datetime.now().strftime('%d%m%y_%H%M')
LOG_DIR   = '/home/pi/WRO_2025_PI/logs'
os.makedirs(LOG_DIR, exist_ok=True)

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(f"{LOG_DIR}/obstacle_{timestamp}.txt"),
        logging.StreamHandler(sys.stderr),   # VS Code debug console
    ]
)
log = logging.getLogger("WRO")

# ─────────────────────────────────────────────────────────────────────────────
#  PIGPIO DAEMON  — kill any stale instance, start fresh, wait with retry
# ─────────────────────────────────────────────────────────────────────────────
def _start_pigpiod(retries: int = 15, interval: float = 1.0) -> pigpio.pi:
    """
    Kill any existing pigpiod, launch a fresh one, then retry connecting
    up to `retries` times with `interval` seconds between attempts.
    Exits the program if the daemon never becomes reachable.
    """
    log.info("Stopping any existing pigpiod…")
    os.system("sudo pigpiod -t 0 -p 8888 2>/dev/null || true")   # ignore 'not running'
    time.sleep(0.3)
    os.system("sudo pkill -9 pigpiod 2>/dev/null || true")
    time.sleep(0.5)

    log.info("Starting pigpiod…")
    ret = os.system("sudo pigpiod -t 0 -p 8888")
    if ret != 0:
        log.warning(f"pigpiod launch returned exit code {ret} — will still retry connect")

    for attempt in range(1, retries + 1):
        time.sleep(interval)
        pi = pigpio.pi()
        if pi.connected:
            log.info(f"pigpiod connected on attempt {attempt}/{retries}")
            return pi
        log.warning(f"pigpiod not ready yet (attempt {attempt}/{retries}) — retrying…")

    log.critical(
        "pigpiod failed to start after all retries.\n"
        "  • Try manually:  sudo pigpiod && sudo pigpiod -p 8888\n"
        "  • Check logs:    journalctl -u pigpiod -n 30\n"
        "  • Check port:    sudo lsof -i :8888"
    )
    sys.exit(1)

_pigpio_handle = _start_pigpiod()

# ─────────────────────────────────────────────────────────────────────────────
#  HARDWARE PINS
# ─────────────────────────────────────────────────────────────────────────────
RX_Head      = 23
RX_Left      = 24
RX_Right     = 25
RX_Back      = 27
button_pin   = 5
exit_pin     = 7
servo_pin    = 8
blue_led     = 26
red_led      = 10
green_led    = 6
reset_pin    = 19
pwm_pin      = 12
direction_pin = 20

# ─────────────────────────────────────────────────────────────────────────────
#  HARDWARE INIT (main process — before spawning children)
# ─────────────────────────────────────────────────────────────────────────────
# NOTE: Serial port is opened ONLY inside runEncoder() child process.
# Opening it here at module level caused the parent and child to share the
# same file descriptor, which corrupted reads with garbled / merged lines.
UART_PORT  = '/dev/UART_USB'
UART_BAUD  = 115200

# Reuse the handle that _start_pigpiod() already verified is connected
pwm_main = _pigpio_handle

for pin in [reset_pin, blue_led, red_led, green_led]:
    pwm_main.set_mode(pin, pigpio.OUTPUT)
    pwm_main.write(pin, 0)

pwm_main.set_mode(button_pin, pigpio.INPUT)
pwm_main.set_pull_up_down(button_pin, pigpio.PUD_UP)

log.info("Resetting Arduino…")
pwm_main.write(reset_pin, 0)
pwm_main.write(green_led, 1)
time.sleep(1)
pwm_main.write(reset_pin, 1)
pwm_main.write(green_led, 0)
time.sleep(1)
log.info("Reset complete")

servo   = Servo(servo_pin)
tfmini  = TFmini(RX_Head, RX_Left, RX_Right, RX_Back)

# ─────────────────────────────────────────────────────────────────────────────
#  OPENCV HSV CALIBRATION  (tune these for your lighting)
# ─────────────────────────────────────────────────────────────────────────────
RED_LOWER_1  = np.array([  0, 120,  60], dtype=np.uint8)
RED_UPPER_1  = np.array([ 10, 255, 255], dtype=np.uint8)
RED_LOWER_2  = np.array([170, 120,  60], dtype=np.uint8)
RED_UPPER_2  = np.array([180, 255, 255], dtype=np.uint8)

GREEN_LOWER  = np.array([ 40,  80,  40], dtype=np.uint8)
GREEN_UPPER  = np.array([ 90, 255, 200], dtype=np.uint8)

PINK_LOWER   = np.array([135,  70,  60], dtype=np.uint8)
PINK_UPPER   = np.array([175, 255, 255], dtype=np.uint8)

RED_MIN_AREA   = 400
GREEN_MIN_AREA = 400
PINK_MIN_AREA  = 800

CAM_INDEX    = 0          # change to 1 if needed
CAP_W, CAP_H = 640, 360
SMOOTH_N     = 2

# ─────────────────────────────────────────────────────────────────────────────
#  MULTIPROCESSING SHARED MEMORY
# ─────────────────────────────────────────────────────────────────────────────
counts          = multiprocessing.Value('i',   0)
lane_counter    = multiprocessing.Value('i',   0)
red_b           = multiprocessing.Value('b',   False)
green_b         = multiprocessing.Value('b',   False)
pink_b          = multiprocessing.Value('b',   False)
centr_y         = multiprocessing.Value('f',   0.0)   # green cy
centr_x         = multiprocessing.Value('f',   0.0)   # green cx
centr_y_red     = multiprocessing.Value('f',   0.0)
centr_x_red     = multiprocessing.Value('f',   0.0)
centr_x_pink    = multiprocessing.Value('f',   0.0)
centr_y_pink    = multiprocessing.Value('f',   0.0)
head            = multiprocessing.Value('f',   0.0)
sp_angle        = multiprocessing.Value('i',   0)
turn_trigger    = multiprocessing.Value('b',   False)
lidar_angle     = multiprocessing.Value('d',   0.0)
lidar_distance  = multiprocessing.Value('d',   0.0)
lidar_f         = multiprocessing.Value('d',   0.0)
lidar_l         = multiprocessing.Value('d',   0.0)
lidar_r         = multiprocessing.Value('d',   0.0)
previous_angle  = multiprocessing.Value('d',   0.0)
left_f          = multiprocessing.Value('b',   False)
right_f         = multiprocessing.Value('b',   False)
stop_evt        = multiprocessing.Event()

# ─────────────────────────────────────────────────────────────────────────────
#  PID GLOBALS  (used inside servoDrive)
# ─────────────────────────────────────────────────────────────────────────────
kp,  ki,  kd   = 0.6, 0.0, 0.1
kp_e, ki_e, kd_e = 3.0, 0.0, 40.0
prevError = 0.0
totalError = 0.0
corr      = 0
corr_pos  = 0


# ═══════════════════════════════════════════════════════════════════════════════
#  OPENCV DETECTION HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

class FrameSmoother:
    """Majority-vote smoother over N frames per colour."""
    def __init__(self, n: int):
        self.n   = max(1, n)
        self._buf: dict = {}

    def update(self, name: str, detected: bool) -> bool:
        if name not in self._buf:
            self._buf[name] = deque(maxlen=self.n)
        self._buf[name].append(detected)
        return sum(self._buf[name]) > (len(self._buf[name]) // 2)


def _get_best_blob(mask: np.ndarray, min_area: int) -> "dict | None":
    """
    Largest contour above min_area.
    Returns dict {cx, cy, area, x1, y1, x2, y2} or None.
    """
    k     = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    conts, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not conts:
        return None
    best = max(conts, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < min_area:
        return None
    M = cv2.moments(best)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    x, y, w, h = cv2.boundingRect(best)
    return {"cx": cx, "cy": cy, "area": area,
            "x1": x,  "y1": y, "x2": x+w, "y2": y+h}


def _detect_all(frame_bgr: np.ndarray) -> dict:
    """Returns {red, green, pink} → best blob dict or None."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    m_r = cv2.bitwise_or(
        cv2.inRange(hsv, RED_LOWER_1,  RED_UPPER_1),
        cv2.inRange(hsv, RED_LOWER_2,  RED_UPPER_2),
    )
    m_g = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    m_p = cv2.inRange(hsv, PINK_LOWER,  PINK_UPPER)
    return {
        "red"  : _get_best_blob(m_r, RED_MIN_AREA),
        "green": _get_best_blob(m_g, GREEN_MIN_AREA),
        "pink" : _get_best_blob(m_p, PINK_MIN_AREA),
    }


def _draw_detections(canvas: np.ndarray, dets: dict, fps: float, avg: float):
    """Minimal clean overlay matching detection_opencv_clean.py style."""
    CLR = {"red": (0,0,220), "green": (0,210,0), "pink": (220,0,220)}
    W   = canvas.shape[1]
    cv2.putText(canvas, f"FPS {fps:.1f} avg {avg:.1f}",
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255,255,255), 1, cv2.LINE_AA)
    cv2.line(canvas, (0, 240), (W, 240), (55,55,55), 1)
    for name, d in dets.items():
        if d is None:
            continue
        col = CLR[name]
        cx, cy = int(d["cx"]), int(d["cy"])
        cv2.rectangle(canvas, (d["x1"], d["y1"]), (d["x2"], d["y2"]), col, 2)
        cv2.circle(canvas, (cx, cy), 5, col,       -1)
        cv2.circle(canvas, (cx, cy), 5, (255,255,255), 1)
        cv2.putText(canvas, name.upper(), (d["x1"]+3, d["y1"]-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0,0,0), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"A:{d['area']:.0f}", (d["x1"], d["y2"]+15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)


# ═══════════════════════════════════════════════════════════════════════════════
#  PROCESS 1 — LIVE FEED  (OpenCV HSV, replaces EdgeTPU Live_Feed)
# ═══════════════════════════════════════════════════════════════════════════════

def Live_Feed_OpenCV(red_b, green_b, pink_b,
                     centr_y, centr_x,
                     centr_y_red, centr_x_red,
                     centr_x_pink, centr_y_pink):
    """
    Camera loop: HSV-detect red / green / pink, update shared values.
    Replaces the pycoral EdgeTPU interpreter completely.

    ── SSH / VS Code headless: comment out the cv2.imshow / cv2.waitKey lines ──
    """
    # ── optional display ─────────────────────────────────────────── SSH NOTE ──
    SHOW_WINDOW = True     # ← set False (or comment out imshow block) over SSH
    # ─────────────────────────────────────────────────────────────────────────

    # Force xcb backend so OpenCV Qt window works over SSH / without Wayland
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    os.environ.setdefault("DISPLAY", ":0")

    smoother = FrameSmoother(SMOOTH_N)
    fps_hist = deque(maxlen=30)
    t_prev   = time.perf_counter()

    # Auto-scan for a working camera index (video0 on Pi is often not a capture device)
    cap = None
    for _idx in range(6):
        _c = cv2.VideoCapture(_idx)
        if _c.isOpened():
            ret_test, _ = _c.read()
            if ret_test:
                cap = _c
                log.info(f"[Live_Feed] using camera index {_idx}")
                break
            _c.release()
    if cap is None:
        log.error("[Live_Feed] No working camera found on indices 0-5 — check connection")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,   CAP_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  CAP_H)
    cap.set(cv2.CAP_PROP_FPS,           120)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE,      -6)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,    1)
    date_str = datetime.now().strftime("%d-%m-%y")
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Actual resolution: {actual_w}x{actual_h}")

    recorder = cv2.VideoWriter(
        f"/home/pi/WRO_2025_PI/videos/obstacle_challenge_{date_str}.avi",
        cv2.VideoWriter_fourcc(*"MJPG"),
        20.0,
        (actual_w, actual_h),  # use actual size, not hardcoded
    )

    if SHOW_WINDOW:                                              # ← SSH: comment
        WIN = "WRO 2025 — OpenCV Detection  |  Q quit"          # ← SSH: comment
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)                 # ← SSH: comment
        cv2.resizeWindow(WIN, CAP_W, CAP_H)                     # ← SSH: comment

    try:
        while True:
            cap.grab()
            ret, frame_bgr = cap.retrieve()
            if not ret:
                continue

            raw_dets = _detect_all(frame_bgr)

            # Smooth per colour
            for c in ("red", "green", "pink"):
                if not smoother.update(c, raw_dets[c] is not None):
                    raw_dets[c] = None

            rd = raw_dets["red"]
            gd = raw_dets["green"]
            pk = raw_dets["pink"]

            # ── update shared values (mirrors original pycoral logic exactly) ──
            if pk and not rd and not gd:
                # only pink
                pink_b.value = True; red_b.value = False; green_b.value = False
                centr_x_pink.value = pk["cx"]; centr_y_pink.value = pk["cy"]
                centr_x.value = centr_y.value = centr_x_red.value = centr_y_red.value = 0

            elif pk and rd and not gd:
                # pink + red
                red_b.value = True; pink_b.value = True; green_b.value = False
                centr_x_red.value  = rd["cx"]; centr_y_red.value  = rd["cy"]
                centr_x_pink.value = pk["cx"]; centr_y_pink.value = pk["cy"]
                centr_x.value = centr_y.value = 0

            elif pk and gd and not rd:
                # pink + green
                green_b.value = True; pink_b.value = True; red_b.value = False
                centr_x.value      = gd["cx"]; centr_y.value      = gd["cy"]
                centr_x_pink.value = pk["cx"]; centr_y_pink.value = pk["cy"]
                centr_x_red.value = centr_y_red.value = 0

            elif gd and not pk:
                # green only (or green + red — follow priority)
                green_b.value = True; red_b.value = False; pink_b.value = False
                centr_x.value = gd["cx"]; centr_y.value = gd["cy"]
                centr_x_red.value = centr_y_red.value = 0
                centr_x_pink.value = centr_y_pink.value = 0

            elif rd and not pk:
                # red only
                red_b.value = True; green_b.value = False; pink_b.value = False
                centr_x_red.value = rd["cx"]; centr_y_red.value = rd["cy"]
                centr_x.value = centr_y.value = 0
                centr_x_pink.value = centr_y_pink.value = 0

            else:
                # nothing detected
                red_b.value = green_b.value = pink_b.value = False
                centr_x.value = centr_y.value = 0
                centr_x_red.value = centr_y_red.value = 0
                centr_x_pink.value = centr_y_pink.value = 0

            # ── FPS ──
            t_now  = time.perf_counter()
            fps    = 1.0 / max(t_now - t_prev, 1e-6)
            t_prev = t_now
            fps_hist.append(fps)
            avg = sum(fps_hist) / len(fps_hist)
            canvas = frame_bgr.copy()                           # ← SSH: comment
            _draw_detections(canvas, raw_dets, fps, avg)        # ← SSH: comment

            recorder.write(canvas)

            # ── optional window ──────────────────────────────────── SSH NOTE ──
            if SHOW_WINDOW:                                          # ← SSH: comment
                cv2.imshow(WIN, canvas)                             # ← SSH: comment
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):        # ← SSH: comment
                    break                                           # ← SSH: comment
            # ─────────────────────────────────────────────────────────────────

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()   # ← SSH: comment if SHOW_WINDOW=False
        log.info("[Live_Feed_OpenCV] stopped")


# ═══════════════════════════════════════════════════════════════════════════════
#  PID / STEERING FUNCTIONS  (unchanged from original)
# ═══════════════════════════════════════════════════════════════════════════════

def correctAngle(setPoint_gyro: float, heading: float, multiplier: float = 1.0):
    global corr
    error_gyro = heading - setPoint_gyro
    if error_gyro > 180:
        error_gyro -= 360
    corr = error_gyro
    correction = kp * error_gyro * multiplier + kd * error_gyro
    correction = max(-60, min(60, correction)) if multiplier == 3 else max(-30, min(30, correction))
    servo.setAngle(90 - correction)


def correctReverseAngle(setPoint_gyro: float, heading: float, multiplier: float = 1.0):
    global corr
    error_gyro = heading - setPoint_gyro
    if error_gyro > 180:
        error_gyro -= 360
    corr = error_gyro
    correction = kp * error_gyro * multiplier + kd * error_gyro
    correction = max(-55, min(55, correction)) if multiplier == 3 else max(-30, min(30, correction))
    servo.setAngle(90 + correction)


def correctWall(setPoint_distance: float, dist: float,
                sp_h: float, imu_h: float,
                orange: bool, blue: bool,
                pink_l: int, counter: int,
                left: bool, right: bool):
    if right:
        error_d = dist - setPoint_distance
    elif left:
        error_d = setPoint_distance - dist
    else:
        error_d = 0
    correction = max(-40, min(40, 2.5 * error_d))
    # Guard: too close — don't over-correct
    if (dist < 30 and setPoint_distance == 35) or \
       (setPoint_distance == 60 and dist <= 25) or \
       (setPoint_distance == 45 and dist < 20):
        correction = 0
    mult = 1.0 if counter % 4 == pink_l else 1.8
    correctAngle(sp_h + correction, imu_h, mult)


def correctPosition(setPoint: float, head_sp: float,
                    x: float, y: float,
                    counter: int, blue: bool, orange: bool,
                    heading: float,
                    centr_x_p: float, centr_x_r: float,
                    centr_x_g: float, centr_y_g: float,
                    centr_y_r: float, centr_y_p: float,
                    distance_h: float, distance_l: float, distance_r: float,
                    red: bool, green: bool,
                    pink_l: int, corr_h: float, last_c: int):
    global prevError, totalError, corr_pos
    lane = counter % 4
    error = 0.0

    if   lane == 0:
        error = setPoint - y
    elif lane == 1:
        error = (x - (100 - setPoint)) if orange else ((100 + setPoint) - x)
    elif lane == 2:
        error = (y - (200 - setPoint)) if orange else (y - (-200 - setPoint))
    elif lane == 3:
        error = ((setPoint - 100) - x) if orange else (x + (100 + setPoint))

    corr_pos  = error
    correction = (kp_e * error) + (kd_e * (error - prevError))
    correction = max(-45, min(45, correction))

    # wall-proximity override
    n_head = normalize_angle(heading, blue, orange, lane)
    if setPoint <= -35 and lidar_l.value <= 250:
        correction = -25 if counter == last_c else 0
    elif setPoint >= 35 and lidar_r.value <= 250:
        correction = 25  if counter == last_c else 0

    prevError = error
    tfmini.getTFminiData()
    mult = 1.0 if counter % 4 == pink_l else 1.5
    correctAngle(head_sp + correction, heading, mult)

    log.debug(
        f"[correctPosition] lane:{lane} err:{error:.2f} sp:{setPoint} "
        f"x:{x:.1f} y:{y:.1f} corr:{correction:.2f}"
    )


def update_heading(counter: int, heading_angle: float, blue: bool, orange: bool) -> float:
    if blue:
        return -((90 * counter) % 360)
    elif orange:
        return  (90 * counter) % 360
    return heading_angle


def normalize_angle(angle: float, blue: bool, orange: bool, lane: int) -> float:
    if blue:
        return (angle + 360) if (angle < 180 and lane == 0) else angle
    elif orange:
        return (angle - 360) if (angle > 180 and lane == 0) else angle
    return angle


def reset_coordinates(distance: float, lane: int,
                      orange: bool, blue: bool,
                      x: float, y: float):
    if lane == 1: return (150 - distance) - 5, y
    if lane == 2:
        if orange: return x, (250 - distance) - 5
        if blue:   return x, (distance - 250) + 5
    if lane == 3: return (distance - 150) + 5, y
    if lane == 0:
        if orange: return x, (distance - 50) + 5
        if blue:   return x, (50 - distance) - 5
    return x, y


# ═══════════════════════════════════════════════════════════════════════════════
#  PROCESS 2 — ENCODER / IMU READER  (unchanged)
# ═══════════════════════════════════════════════════════════════════════════════

def runEncoder(counts, head, lane_counter, left_f, right_f):
    log.info("[Encoder] process started")
    time.sleep(2)
    # Open serial here — inside the child process only — to avoid fd sharing
    try:
        _ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        log.info(f"[Encoder] serial opened: {UART_PORT} @ {UART_BAUD}")
    except Exception as e:
        log.error(f"[Encoder] cannot open serial port: {e}")
        return
    try:
        while True:
            line     = _ser.readline().decode("utf-8", errors="ignore").strip()
            esp_data = line.split()
            if len(esp_data) >= 2:
                try:
                    lc = lane_counter.value
                    if right_f.value:
                        head.value = float(esp_data[0]) + (0.57 * lc)
                    elif left_f.value:
                        head.value = float(esp_data[0]) - (0.57 * lc)
                    else:
                        head.value = float(esp_data[0])
                    counts.value = int(esp_data[1])
                except ValueError:
                    log.warning(f"[Encoder] malformed data: {esp_data}")
            else:
                log.warning(f"[Encoder] incomplete data: {esp_data}")
    except Exception as e:
        log.error(f"[Encoder] exception: {e}")
    finally:
        try:
            _ser.close()
            log.info("[Encoder] serial closed")
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════════
#  PROCESS 3 — LIDAR READER  (unchanged)
# ═══════════════════════════════════════════════════════════════════════════════

def read_lidar(lidar_angle, lidar_distance, sp_angle, turn_trigger,
               lidar_f, lidar_l, lidar_r, left_f, right_f, head):
    LIDAR_BIN = "/home/pi/rplidar_sdk/output/Linux/Release/ultra_simple"
    rplidar   = [None] * 360
    prev_dist = 0
    F = L = R = 0
    prev_a = 0

    log.info("[LIDAR] waiting for output…")
    if not os.path.isfile(LIDAR_BIN):
        log.error(f"[LIDAR] binary not found: {LIDAR_BIN}")
        return

    proc = subprocess.Popen(
        [LIDAR_BIN, "--channel", "--serial", "/dev/LIDAR_USB", "460800"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )

    imu_r = sp = 0
    for line in proc.stdout:
        line = line.strip()
        if "theta" in line and "Dist" in line:
            try:
                parts     = line.split()
                angle_raw = float(parts[parts.index("theta:") + 1])
                dist_raw  = float(parts[parts.index("Dist:")  + 1])
                angle_i   = int(angle_raw)
                sp        = -(sp_angle.value % 360)
                imu_r     = int(head.value)
            except Exception as e:
                log.debug(f"[LIDAR] parse error: {e}")
                continue
        else:
            continue

        if prev_a != angle_i:
            while abs(angle_i - prev_a) > 1:
                a = (prev_a + 1) % 360
                rplidar[a] = prev_dist
                prev_a     = a
                if a == (0   + imu_r + sp) % 360:
                    F = 0.2*F + 0.8*prev_dist if F else prev_dist
                    lidar_f.value = F
                if a == (90  + imu_r + sp) % 360:
                    L = 0.2*L + 0.8*prev_dist if L else prev_dist
                    lidar_l.value = L
                if a == (270 + imu_r + sp) % 360:
                    R = 0.2*R + 0.8*prev_dist if R else prev_dist
                    lidar_r.value = R
                if (F<=950 and R>=1500) and right_f.value and not left_f.value:
                    turn_trigger.value = True
                elif (F<=950 and L>=1500) and left_f.value and not right_f.value:
                    turn_trigger.value = True
                else:
                    turn_trigger.value = False

        if dist_raw != 0:
            with lidar_angle.get_lock(), lidar_distance.get_lock(), head.get_lock():
                lidar_angle.value    = angle_i
                lidar_distance.value = dist_raw
                prev_dist            = dist_raw
                prev_a               = angle_i
                rplidar[angle_i]     = dist_raw
                if angle_i == (0   + imu_r + sp) % 360:
                    F = 0.2*F + 0.8*dist_raw if F else dist_raw;  lidar_f.value = F
                if angle_i == (90  + imu_r + sp) % 360:
                    L = 0.2*L + 0.8*dist_raw if L else dist_raw;  lidar_l.value = L
                if angle_i == (270 + imu_r + sp) % 360:
                    R = 0.2*R + 0.8*dist_raw if R else dist_raw;  lidar_r.value = R
                if (F<=950 and R>=1500) and right_f.value and not left_f.value:
                    turn_trigger.value = True
                elif (F<=950 and L>=1500) and left_f.value and not right_f.value:
                    turn_trigger.value = True
                else:
                    turn_trigger.value = False


# ═══════════════════════════════════════════════════════════════════════════════
#  MOTOR HELPER
# ═══════════════════════════════════════════════════════════════════════════════

def runMotor(pwm_h, speed: float, direction: int):
    """direction: 1 = forward, 0 = reverse"""
    pwm_h.set_PWM_dutycycle(pwm_pin, int(speed * 2.55))
    pwm_h.write(direction_pin, direction)


# ═══════════════════════════════════════════════════════════════════════════════
#  TELEMETRY PRINT  (terminal + VS Code debug console via logging)
# ═══════════════════════════════════════════════════════════════════════════════

def print_telemetry(
    counter, heading_angle, imu_head, corr,
    x, y, enc_counts,
    tf_h, tf_l, tf_r,
    l_front, l_left, l_right,
    g_flag, r_flag, p_flag,
    g_past, r_past, p_past,
    button, orange_flag, blue_flag,
    setPointL, setPointR, setPointC,
    power, trigger, reset_f,
    red_bv, green_bv, pink_bv,
    cx_g, cy_g, cx_r, cy_r, cx_p, cy_p
):
    sep = "─" * 70

    # Plain terminal print (goes to stdout / log file)
    print(sep)
    print(f"  COUNTER:{counter:2d}  HEADING_SP:{heading_angle:7.2f}  IMU:{imu_head:7.2f}  CORR:{corr:6.2f}")
    print(f"  DIR: {'ORANGE' if orange_flag else 'BLUE  ' if blue_flag else '???   '}  "
          f"BUTTON:{'ON ' if button else 'OFF'}  "
          f"TRIGGER:{trigger}  RESET:{reset_f}")
    print(f"  POS  x:{x:7.2f}  y:{y:7.2f}  ENC:{enc_counts}")
    print(f"  TFMINI  H:{tf_h:6.1f}  L:{tf_l:5.1f}  R:{tf_r:5.1f}")
    print(f"  LIDAR   F:{l_front:7.1f}  L:{l_left:7.1f}  R:{l_right:7.1f}")
    print(f"  DET  red:{red_bv}  green:{green_bv}  pink:{pink_bv}")
    print(f"  CX   G:{cx_g:5.0f}  R:{cx_r:5.0f}  P:{cx_p:5.0f}  "
          f"CY  G:{cy_g:5.0f}  R:{cy_r:5.0f}  P:{cy_p:5.0f}")
    print(f"  FLAGS  g_flag:{g_flag}  r_flag:{r_flag}  p_flag:{p_flag}  "
          f"g_past:{g_past}  r_past:{r_past}  p_past:{p_past}")
    print(f"  SP   L:{setPointL:5.1f}  R:{setPointR:5.1f}  C:{setPointC:5.1f}  POWER:{power:5.1f}")
    print(sep)

    # log.debug mirrors the same to VS Code debug console
    log.debug(
        f"CTR:{counter} HDG:{heading_angle:.1f} IMU:{imu_head:.1f} CORR:{corr:.2f} | "
        f"x:{x:.1f} y:{y:.1f} | "
        f"TF H:{tf_h:.0f} L:{tf_l:.0f} R:{tf_r:.0f} | "
        f"LID F:{l_front:.0f} L:{l_left:.0f} R:{l_right:.0f} | "
        f"det R:{red_bv} G:{green_bv} P:{pink_bv} | "
        f"flags g:{g_flag} r:{r_flag} p:{p_flag} | "
        f"PWR:{power:.0f}"
    )


# ═══════════════════════════════════════════════════════════════════════════════
#  PROCESS 4 — SERVO DRIVE  (main robot brain, fully preserved logic)
# ═══════════════════════════════════════════════════════════════════════════════

def servoDrive(red_b, green_b, pink_b,
               counts, centr_y, centr_x,
               centr_y_red, centr_x_red,
               centr_x_pink, centr_y_pink,
               head, sp_angle, turn_trigger,
               lidar_f, lidar_l, lidar_r,
               left_f, right_f, lane_counter):

    pwm_h = pigpio.pi()
    global corr, corr_pos

    pwm_h.set_mode(pwm_pin,       pigpio.OUTPUT)
    pwm_h.set_mode(direction_pin, pigpio.OUTPUT)
    # Pin 12 shares the main clock — hardware_PWM unavailable; use software PWM
    pwm_h.set_PWM_frequency(pwm_pin, 1000)
    pwm_h.set_PWM_dutycycle(pwm_pin, 0)

    enc = EncoderCounter()

    # ── flags ──────────────────────────────────────────────────────────────
    button = trigger = reset_f = False
    blue_flag = orange_flag = False
    g_flag = r_flag = p_flag = False
    g_past = r_past = p_past = False
    calc_time = lap_finish = continue_parking = False
    parking_flag = parking_heading = False
    turn_flag = reset_flags = counter_reset = False
    finished = exit_flag = False
    red_time = green_time = 0
    parking_right = True
    parking_left  = False
    inParkingatStart = False
    reverse_true = False
    parking_heading_reverse = False
    avoided = False

    # ── scalars ────────────────────────────────────────────────────────────
    setPointL = -35.0
    setPointR =  35.0
    setPointC =   0.0
    power     =  95.0
    prev_power = 0.0
    last_counter     = 12
    counter          = 0
    heading_angle    = 0.0
    lap_finish_time  = 0.0
    prev_distance    = 0.0
    turn_trigger_distance = 0
    target_count     = 0
    offset           = 0
    button_STATE     = exit_STATE = 0
    time_p = prev_time = itr_prev_time = 0.0
    fps_time2 = 0.0
    debounce_delay   = 0.1
    last_time        = exit_last_time = 0.0
    avoided_time     = time.time()
    reverse_until    = 0.0
    encoder_counts_value = 0
    off       = 6
    rev_count = 0
    parking_flag = False
    trigger_enc  = 0
    startPark    = 0
    exitPark     = False
    init         = False
    timer        = 0.0
    norm_head    = 0.0
    lane_reset   = 0
    pink_time    = 0.0
    finish_t     = time.time()
    inside_pink  = time.time()
    STATE        = 1
    parking_count = 500
    parking_count_current = 0
    full_park    = False
    timer_t      = time.time()
    STATE_INIT   = 1
    enc_count    = 0
    parking_STATE = 1
    OBSTACLE_STATE = 1
    RESET_STATE  = 1
    pink_thresh  = 0.0
    blue_lap     = False
    avoid_thres  = 1.5
    buff_time    = time.time()
    start_enc_thresh = 0
    corr_thresh  = 0
    parking_distance = 0
    pink_wall_lane   = 0
    forward_time = time.time()
    reset_lane   = 0
    final_park   = time.time()
    front_thresh = 950
    parking_timeout = 1.3
    finish_thresh   = 1000
    before_finish   = 0
    last_counter_timer = 0
    initBot      = True
    p_pass       = 0

    servo.setAngle(40)

    try:
        while True:
            imu_head = head.value
            fps_time2 = time.time()

            tfmini.getTFminiData()
            tf_h    = lidar_f.value
            l_left  = lidar_l.value
            l_right = lidar_r.value
            tf_l    = tfmini.distance_left
            tf_r    = tfmini.distance_right

            # ── servo init ─────────────────────────────────────────────────
            if not init:
                if (tf_h > 0 and head.value > 0) or \
                   pink_b.value or green_b.value or red_b.value:
                    correctAngle(heading_angle, head.value, 1.5)
                    log.debug("[servoDrive] servo init correction applied")
                    init = True
                else:
                    servo.setAngle(45)

            # ── LED indicators ────────────────────────────────────────────
            if green_b.value:
                pwm_h.write(red_led, 0); pwm_h.write(green_led, 1)
            elif red_b.value:
                pwm_h.write(red_led, 1); pwm_h.write(green_led, 0)
            elif pink_b.value:
                pwm_h.write(blue_led, 1)
            else:
                pwm_h.write(red_led, 0)
                pwm_h.write(green_led, 0)
                pwm_h.write(blue_led, 0)

            # ── starting-position parking detection ───────────────────────
            if not inParkingatStart and not left_f.value and not right_f.value:
                if tf_l < 25 and tf_h < 250 and tf_l > 0 and \
                   tf_h > 0 and lidar_l.value > 0:
                    enc.x = 0
                    enc.y = (lidar_l.value - 400) / 10
                    right_f.value = True
                    inParkingatStart = True
                    log.info("[servoDrive] start: parking RIGHT side")
                elif tf_r < 25 and tf_h < 250 and tf_h > 0 and \
                     tf_r > 0 and lidar_r.value > 0:
                    enc.x = 0
                    enc.y = (400 - lidar_r.value) / 10
                    left_f.value = True
                    inParkingatStart = True
                    log.info("[servoDrive] start: parking LEFT side")

            if right_f.value and not orange_flag:
                orange_flag = True; blue_flag = False
            elif left_f.value and not blue_flag:
                blue_flag = True; orange_flag = False

            # ── 12-lap stop condition ──────────────────────────────────────
            if counter == last_counter and not lap_finish:
                reset_f = False
                log.info(f"[servoDrive] MAXIMUM LAPS — target:{target_count} front:{lidar_f.value:.0f}")
                if not finished:
                    if orange_flag:
                        finish_thresh = 1100 if parking_right else 1300
                        before_finish = 1000
                        target_count  = counts.value + (27000 if parking_right else 22500)
                    elif blue_flag:
                        finish_thresh = 1300 if parking_right else 1100
                        before_finish = 1000
                        target_count  = counts.value + (22500 if parking_right else 27000)
                    finished = True

                if (counts.value >= target_count) or \
                   (counts.value >= target_count - before_finish and
                    lidar_f.value < finish_thresh):
                    power = 0
                    pink_b.value = False
                    runMotor(pwm_h, power, 1)
                    time.sleep(1)
                    pink_b.value = False
                    power = 70; prev_power = 0
                    lap_finish = True
                    log.info("[servoDrive] vehicle stopped — lap_finish")

            # ── lap-finish parking manoeuvre ───────────────────────────────
            if lap_finish and not continue_parking:
                correctAngle(heading_angle, head.value, 1)

                if orange_flag:
                    while lidar_f.value > 1100:
                        itr_prev_time = time.time()
                        power = 50; prev_power = 0
                        x, y = enc.get_position(head.value, counts.value)
                        tfmini.getTFminiData()
                        correctAngle(heading_angle, head.value, 1)
                        runMotor(pwm_h, power, 1)
                    if parking_STATE == 1:
                        heading_angle += 90
                        correctReverseAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            pwm_h.set_PWM_dutycycle(pwm_pin, int(1.2 * 70))
                            pwm_h.write(direction_pin, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        while tfmini.distance_head < 55:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 33, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        parking_STATE = 2
                    if parking_STATE == 2:
                        heading_angle += 90 if parking_right else -90
                        correctAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            pwm_h.set_PWM_dutycycle(pwm_pin, int(1.2 * 70))
                            pwm_h.write(direction_pin, 1)
                            correctAngle(heading_angle, head.value, 3)
                        p_flag = True; continue_parking = True
                        parking_STATE = 3; pink_time = time.time()

                elif blue_flag:
                    while lidar_f.value > 1600:
                        itr_prev_time = time.time()
                        power = 60; prev_power = 0
                        x, y = enc.get_position(head.value, counts.value)
                        tfmini.getTFminiData()
                        correctAngle(heading_angle, head.value, 1)
                        runMotor(pwm_h, power, 1)
                    heading_angle -= 90
                    if parking_STATE == 1:
                        correctReverseAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 36, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        while tfmini.distance_head < 50:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 36, 0)
                            correctReverseAngle(heading_angle, head.value, 3)
                        parking_STATE = 2
                    if parking_STATE == 2:
                        heading_angle += 90 if parking_right else -90
                        correctAngle(heading_angle, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 36, 1)
                            correctAngle(heading_angle, head.value, 3)
                        p_flag = True; continue_parking = True
                        parking_STATE = 3; pink_time = time.time()

            # ── lane-0 pink-wall setpoint decisions ───────────────────────
            if counter % 4 == pink_wall_lane:
                if orange_flag:
                    if 0 < centr_x_red.value < 200:
                        red_b.value = False
                elif blue_flag:
                    norm_head = head.value - 360 if head.value > 180 else head.value
                    if (centr_x_red.value > 390 and
                            abs(heading_angle - norm_head) > 10 and
                            not r_flag):
                        red_b.value = False

            # ── setpoint drift while avoiding ─────────────────────────────
            if g_flag and not continue_parking:
                setPointL -= 1
                setPointL  = min(0, max((-100 if orange_flag else -35), setPointL))
                setPointR  = 35
            elif r_flag and not continue_parking:
                setPointR += 1
                setPointR  = max(0, min((35 if orange_flag else 100), setPointR))
                setPointL  = -35

            # ── button / exit debounce ────────────────────────────────────
            if time.time() - last_time > debounce_delay:
                prev_bs  = button_STATE
                button_STATE = pwm_h.read(button_pin)
                if prev_bs == 1 and button_STATE == 0:
                    button = not button
                    last_time = time.time()
                    power = 95
                    log.info(f"[servoDrive] Drive {'STARTED' if button else 'STOPPED'}")

            if time.time() - exit_last_time > debounce_delay:
                prev_es  = exit_STATE
                exit_STATE = pwm_h.read(exit_pin)
                if prev_es == 1 and exit_STATE == 0:
                    exit_flag = not exit_flag
                    exit_last_time = time.time()

            if exit_flag and button:
                log.info("[servoDrive] Exit triggered — shutting down")
                sys.exit(0)

            # ═════════════════════════════════════════════════════════════
            #  MAIN DRIVING BLOCK  (button pressed)
            # ═════════════════════════════════════════════════════════════
            if button:
                if initBot:
                    time.sleep(0.2); initBot = False

                x, y = enc.get_position(imu_head, counts.value)

                # ── start-position parking exit ───────────────────────────
                if inParkingatStart:
                    if STATE_INIT == 1:
                        if orange_flag: correctAngle(heading_angle + 90, head.value, 3)
                        else:           correctAngle(heading_angle - 90, head.value, 3)
                        while abs(corr) > 5:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 60, 1)
                            if orange_flag: correctAngle(heading_angle + 90, head.value, 3)
                            else:           correctAngle(heading_angle - 90, head.value, 3)
                        forward_time = time.time()
                        while time.time() - forward_time < 0.5:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 60, 1)
                            if orange_flag: correctAngle(heading_angle + 90, head.value, 3)
                            else:           correctAngle(heading_angle - 90, head.value, 3)
                        while tfmini.distance_head > 24:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 30, 1)
                            if orange_flag: correctAngle(heading_angle + 90, head.value, 3)
                            else:           correctAngle(heading_angle - 90, head.value, 3)
                        while tfmini.distance_head < 24:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 30, 0)
                            if orange_flag: correctAngle(heading_angle + 90, head.value, 3)
                            else:           correctAngle(heading_angle - 90, head.value, 3)
                        STATE_INIT = 2

                    if STATE_INIT == 2:
                        if orange_flag:
                            correctReverseAngle(heading_angle + 10, head.value, 3)
                            enc_count = counts.value
                            start_enc_thresh = 6000 if parking_right else 9000
                            corr_thresh = 8
                        elif blue_flag:
                            correctReverseAngle(heading_angle, head.value, 3)
                            enc_count = counts.value
                            start_enc_thresh = 9000 if parking_right else 6000
                            corr_thresh = 8
                        while abs(corr) > corr_thresh or counts.value > enc_count - start_enc_thresh:
                            itr_prev_time = time.time()
                            x, y = enc.get_position(head.value, counts.value)
                            tfmini.getTFminiData()
                            runMotor(pwm_h, 60, 0)
                            if orange_flag: correctReverseAngle(heading_angle + 10, head.value, 3)
                            else:           correctReverseAngle(heading_angle,      head.value, 3)
                        STATE_INIT = 3
                        RESET_STATE = 0
                        inParkingatStart = False

                # ── stall-recovery (iteration took > 1 s) ─────────────────
                if time.time() - itr_prev_time > 1:
                    correctReverseAngle(heading_angle, head.value, 1)
                    while abs(corr) > 8:
                        runMotor(pwm_h, 50, 0)
                        correctReverseAngle(heading_angle, head.value, 1.5)
                    tfmini.getTFminiData()
                    x, y = enc.get_position(head.value, counts.value)
                    if orange_flag:
                        turn_trigger_distance = round(
                            tfmini.distance_left * math.cos(math.radians(abs(corr))))
                    elif blue_flag:
                        turn_trigger_distance = round(
                            tfmini.distance_right * math.cos(math.radians(abs(corr))))
                    enc.x, enc.y = reset_coordinates(
                        turn_trigger_distance, reset_lane, orange_flag, blue_flag, x, y)

                # ── brief stop on obstacle spotted ────────────────────────
                while time.time() < avoided_time:
                    pwm_h.set_PWM_dutycycle(pwm_pin, 0)
                    pwm_h.write(direction_pin, 0)
                    itr_prev_time = time.time()
                    x, y = enc.get_position(head.value, counts.value)
                    tfmini.getTFminiData()

                while avoided_time > 0 and time.time() < reverse_until:
                    itr_prev_time = time.time()
                    x, y = enc.get_position(head.value, counts.value)
                    tfmini.getTFminiData()
                    pwm_h.set_PWM_dutycycle(pwm_pin, int(1.3 * 70))
                    pwm_h.write(direction_pin, 0)
                    if r_flag and not reset_f:   servo.setAngle(70)
                    elif g_flag and not reset_f: servo.setAngle(110)

                # ── parking state machine ─────────────────────────────────
                if parking_flag:
                    log.info(f"[servoDrive] PARKING STATE:{STATE} head_d:{tfmini.distance_head:.1f}")
                    tfmini.getTFminiData()
                    if not calc_time:
                        c_time = time.time(); calc_time = True

                    if STATE == 1:
                        if blue_flag or orange_flag:
                            while tfmini.distance_right > 22:
                                itr_prev_time = time.time()
                                tfmini.getTFminiData()
                                power = 20; prev_power = 0
                                correctAngle(heading_angle, head.value, 1)
                                runMotor(pwm_h, power, 1)
                        parking_count_current = counts.value
                        tfmini.getTFminiData()
                        if orange_flag or blue_flag:
                            parking_count = 5000
                        while counts.value <= parking_count_current + parking_count:
                            itr_prev_time = time.time()
                            runMotor(pwm_h, 28, 1)
                            correctAngle(heading_angle, head.value, 1)
                            tfmini.getTFminiData()
                        full_park = True
                        STATE = 2

                    if STATE == 2:
                        if blue_flag:
                            tfmini.getTFminiData()
                            heading_angle += -90 if parking_right else 90
                            parking_distance = (tfmini.distance_left if parking_right
                                                else tfmini.distance_right)
                            correctReverseAngle(heading_angle, head.value, 3)
                            while parking_distance > 20 or abs(corr) > 20:
                                itr_prev_time = time.time()
                                tfmini.getTFminiData()
                                parking_distance = (tfmini.distance_left if parking_right
                                                    else tfmini.distance_right)
                                runMotor(pwm_h, 36, 0)
                                correctReverseAngle(heading_angle, head.value, 3)
                        if orange_flag:
                            tfmini.getTFminiData()
                            heading_angle += -90 if parking_right else 90
                            parking_distance = (tfmini.distance_left if parking_right
                                                else tfmini.distance_right)
                            correctReverseAngle(heading_angle, head.value, 3)
                            while parking_distance > 20 or abs(corr) > 20:
                                itr_prev_time = time.time()
                                tfmini.getTFminiData()
                                parking_distance = (tfmini.distance_left if parking_right
                                                    else tfmini.distance_right)
                                runMotor(pwm_h, 36, 0)
                                correctReverseAngle(heading_angle, head.value, 3)
                        STATE = 3

                    if STATE == 3 and full_park:
                        if blue_flag:
                            tfmini.getTFminiData()
                            heading_angle += 95 if parking_right else -95
                            parking_distance = (tfmini.distance_right if parking_right
                                                else tfmini.distance_left)
                            final_park = time.time()
                            correctReverseAngle(heading_angle, head.value, 3)
                            while (abs(corr) > 5) and ((abs(corr) > 15) or lidar_f.value < 85):
                                itr_prev_time = time.time()
                                tfmini.getTFminiData()
                                runMotor(pwm_h, 36, 0)
                                correctReverseAngle(heading_angle, head.value, 3)
                        if orange_flag:
                            tfmini.getTFminiData()
                            heading_angle += 95 if parking_right else -95
                            parking_distance = (tfmini.distance_right if parking_right
                                                else tfmini.distance_left)
                            final_park = time.time()
                            correctReverseAngle(heading_angle, head.value, 3)
                            while (abs(corr) > 5) and ((abs(corr) > 15) or lidar_f.value < 85):
                                itr_prev_time = time.time()
                                tfmini.getTFminiData()
                                runMotor(pwm_h, 36, 0)
                                correctReverseAngle(heading_angle, head.value, 3)

                        correctAngle(heading_angle + 2, head.value, 1)
                        while abs(corr) > 15 or lidar_f.value > 80:
                            correctAngle(heading_angle + 2, head.value, 1)
                            power = 15; prev_power = 0
                            runMotor(pwm_h, power, 1)
                            tfmini.getTFminiData()
                        STATE = 4

                    if STATE == 4:
                        power = 0; prev_power = 0
                        pwm_h.set_PWM_dutycycle(pwm_pin, power)
                        log.info("[servoDrive] PARKED — exiting")
                        sys.exit(0)

                else:
                    # ── heading reset after a turn ─────────────────────────
                    if reset_f and counter != last_counter:
                        g_past = r_past = g_flag = r_flag = False
                        rev_count = 0

                        if RESET_STATE == 1:
                            timer_t   = time.time()
                            norm_head = normalize_angle(head.value, blue_flag, orange_flag, lane_reset)
                            correctAngle(heading_angle, head.value, 1.5)
                            while (abs(corr) > 8 or tfmini.distance_head > 60) and \
                                  time.time() - timer_t < 1.5:
                                itr_prev_time = time.time()
                                norm_head = normalize_angle(head.value, blue_flag, orange_flag, lane_reset)
                                x, y = enc.get_position(head.value, counts.value)
                                tfmini.getTFminiData()
                                correctAngle(heading_angle, head.value, 1.5)
                                power = 60; prev_power = 0
                                runMotor(pwm_h, power, 1)
                            RESET_STATE = 2

                        if RESET_STATE == 2:
                            tfmini.getTFminiData()
                            thresh_distance = (tfmini.distance_right if blue_flag
                                               else tfmini.distance_left)
                            if thresh_distance > 45:
                                timer_t = time.time()
                                angle_off = 20 if blue_flag else -20
                                correctAngle(heading_angle + angle_off, head.value, 1.5)
                                while (tfmini.distance_head > 15 or abs(corr) > 5) and \
                                      time.time() - timer_t < 1.8:
                                    itr_prev_time = time.time()
                                    tfmini.getTFminiData()
                                    correctAngle(heading_angle + angle_off, head.value, 1.5)
                                    power = 100; prev_power = 0
                                    runMotor(pwm_h, power, 1)
                                    x, y = enc.get_position(head.value, counts.value)
                            else:
                                timer_t = time.time()
                                angle_off = -20 if blue_flag else 20
                                correctAngle(heading_angle + angle_off, head.value, 1.5)
                                while (tfmini.distance_head > 22 or abs(corr) > 5) and \
                                      time.time() - timer_t < 1.8:
                                    itr_prev_time = time.time()
                                    tfmini.getTFminiData()
                                    correctAngle(heading_angle + angle_off, head.value, 1.5)
                                    power = 100; prev_power = 0
                                    runMotor(pwm_h, power, 1)
                                    x, y = enc.get_position(head.value, counts.value)
                            RESET_STATE = 3

                        if RESET_STATE == 3:
                            x, y = enc.get_position(imu_head, counts.value)
                            counter     += 1
                            lane_reset   = counter % 4
                            heading_angle = update_heading(counter, heading_angle,
                                                           blue_flag, orange_flag)
                            sp_angle.value = heading_angle
                            timer_v   = time.time()
                            norm_head = normalize_angle(head.value, blue_flag, orange_flag, lane_reset)
                            correctReverseAngle(heading_angle, head.value, 1)

                            while (abs(corr) > 5 or time.time() - timer_v < 1.2) and \
                                  time.time() - timer_v < 3:
                                itr_prev_time = time.time()
                                norm_head = normalize_angle(head.value, blue_flag, orange_flag, lane_reset)
                                x, y = enc.get_position(head.value, counts.value)
                                tfmini.getTFminiData()
                                correctReverseAngle(heading_angle, head.value, 2)
                                power = 100; prev_power = 0
                                runMotor(pwm_h, power, 0)

                            if counter == last_counter:
                                last_counter_timer = time.time()
                                while time.time() - last_counter_timer < 1.5:
                                    itr_prev_time = time.time()
                                    x, y = enc.get_position(head.value, counts.value)
                                    tfmini.getTFminiData()
                                    correctReverseAngle(heading_angle, head.value, 2)
                                    power = 85; prev_power = 0
                                    runMotor(pwm_h, power, 0)

                            power = 0; prev_power = 0
                            runMotor(pwm_h, power, 0)
                            reset_f = False
                            RESET_STATE = 1
                            log.info(f"[servoDrive] turn complete — counter:{counter} heading:{heading_angle}")

                    # ── turn trigger ───────────────────────────────────────
                    if turn_trigger.value and not reset_f and counter != last_counter:
                        reset_f = True
                        RESET_STATE = 1
                        log.info("[servoDrive] TURN TRIGGER fired")

                    # ── pink-wall (parking) approach ───────────────────────
                    if lap_finish:
                        sp_angle.value = heading_angle
                        if p_flag and continue_parking and not parking_flag:
                            power = 55
                            wall_sp = (tfmini.distance_left if parking_right
                                       else tfmini.distance_right)
                            correctWall(60, wall_sp, heading_angle, head.value,
                                        orange_flag, blue_flag, pink_wall_lane,
                                        counter,
                                        parking_right, parking_left)
                            pink_thresh = 0.75
                            if time.time() - pink_time > pink_thresh:
                                tfmini.getTFminiData()
                                p_flag = True
                                if parking_right and lidar_f.value < 1600:
                                    p_pass = 2
                                    if p_pass == 2:
                                        p_past = False; parking_flag = True
                                elif parking_left and lidar_r.value > 1000:
                                    p_pass = 2
                                    if p_pass == 2:
                                        p_past = False; parking_flag = True

                    # ── obstacle detect / avoidance logic (main) ──────────
                    elif not lap_finish:
                        if green_b.value and not r_flag and not g_flag and \
                           centr_y.value > 240:
                            if rev_count < 1:
                                if 0 < centr_x.value < 320:
                                    avoided_time  = time.time() + 0.3
                                    reverse_until = avoided_time + 0.7
                                    rev_count += 1
                            g_flag = True; g_past = True
                            pwm_h.write(red_led, 0); pwm_h.write(green_led, 0)

                        elif g_past and not continue_parking and not reset_f:
                            g_flag = True
                            if green_b.value:
                                green_time = time.time()
                            avoid_thres = 1.7
                            if counter % 4 != pink_wall_lane:
                                if (tf_r <= 35 and tf_r > 0 and not green_b.value) or \
                                   time.time() - green_time > avoid_thres:
                                    g_flag = False; g_past = False
                            elif counter % 4 == pink_wall_lane:
                                if orange_flag:
                                    avoid_thres = 0.75
                                    if (tf_r <= 20 and tf_r > 0) or \
                                       time.time() - green_time > avoid_thres:
                                        g_flag = False; g_past = False
                                elif blue_flag:
                                    if time.time() - green_time > 0.85:
                                        g_flag = False; g_past = False

                        elif red_b.value and not g_flag and not r_flag and \
                             centr_y_red.value > 240:
                            r_flag = True; r_past = True
                            if rev_count < 1:
                                if centr_x_red.value > 400:
                                    avoided_time  = time.time() + 0.3
                                    reverse_until = avoided_time + 0.7
                                rev_count += 1
                            pwm_h.write(red_led, 0); pwm_h.write(green_led, 0)

                        elif r_past and not continue_parking and not reset_f:
                            r_flag = True
                            if red_b.value:
                                red_time = time.time()
                            avoid_thres = 1.7
                            if counter % 4 != pink_wall_lane:
                                if (tf_l <= 35 and tf_l > 0 and not red_b.value) or \
                                   time.time() - red_time > avoid_thres:
                                    r_flag = False; r_past = False
                            elif counter % 4 == pink_wall_lane:
                                if orange_flag:
                                    avoid_thres = 0.85
                                    if time.time() - red_time > avoid_thres:
                                        r_flag = False; r_past = False
                                elif blue_flag:
                                    if (tf_l <= 25 and tf_l > 0) or \
                                       time.time() - red_time > 0.75:
                                        r_flag = False; r_past = False
                        else:
                            g_flag = r_flag = p_flag = False
                            r_past = g_past = p_past = False

                        pwm_h.write(red_led, 0); pwm_h.write(green_led, 0)

                    # ── steering setpoint selection ────────────────────────
                    if g_flag:
                        if counter % 4 == pink_wall_lane and orange_flag:
                            correctWall(35, tfmini.distance_left,
                                        heading_angle, head.value,
                                        orange_flag, blue_flag, pink_wall_lane,
                                        counter, True, False)
                        else:
                            correctPosition(
                                setPointL, heading_angle, x, y,
                                counter, blue_flag, orange_flag, head.value,
                                centr_x_pink.value, centr_x_red.value,
                                centr_x.value, centr_y.value,
                                centr_y_red.value, centr_y_pink.value,
                                tf_h, tf_l, tf_r,
                                red_b.value, green_b.value,
                                pink_wall_lane, abs(corr), last_counter)
                    elif r_flag:
                        if counter % 4 == pink_wall_lane and blue_flag:
                            correctWall(35, tfmini.distance_right,
                                        heading_angle, head.value,
                                        orange_flag, blue_flag, pink_wall_lane,
                                        counter, False, True)
                        else:
                            correctPosition(
                                setPointR, heading_angle, x, y,
                                counter, blue_flag, orange_flag, head.value,
                                centr_x_pink.value, centr_x_red.value,
                                centr_x.value, centr_y.value,
                                centr_y_red.value, centr_y_pink.value,
                                tf_h, tf_l, tf_r,
                                red_b.value, green_b.value,
                                pink_wall_lane, abs(corr), last_counter)
                    else:
                        correctPosition(
                            setPointC, heading_angle, x, y,
                            counter, blue_flag, orange_flag, head.value,
                            centr_x_pink.value, centr_x_red.value,
                            centr_x.value, centr_y.value,
                            centr_y_red.value, centr_y_pink.value,
                            tf_h, tf_l, tf_r,
                            red_b.value, green_b.value,
                            pink_wall_lane, abs(corr), last_counter)

                # ── motor drive  ───────────────────────────────────────────
                total_power = (power * 0.1) + (prev_power * 0.9)
                prev_power  = total_power
                pwm_h.set_PWM_dutycycle(pwm_pin, int(2.55 * total_power))
                pwm_h.write(direction_pin, 1)

                itr_prev_time = time.time()
                lane_counter.value = counter

                # ── telemetry print ────────────────────────────────────────
                print_telemetry(
                    counter, heading_angle, imu_head, corr,
                    x, y, counts.value,
                    tf_h, tf_l, tf_r,
                    lidar_f.value, lidar_l.value, lidar_r.value,
                    g_flag, r_flag, p_flag,
                    g_past, r_past, p_past,
                    button, orange_flag, blue_flag,
                    setPointL, setPointR, setPointC,
                    total_power, trigger, reset_f,
                    red_b.value, green_b.value, pink_b.value,
                    centr_x.value, centr_y.value,
                    centr_x_red.value, centr_y_red.value,
                    centr_x_pink.value, centr_y_pink.value,
                )

            else:
                # button not pressed — hold still
                power = 0
                pwm_h.set_PWM_dutycycle(pwm_pin, 0)  # stop motor (was hardware_PWM)
                counter = 0

    except Exception as e:
        log.error(f"[servoDrive] exception: {e}")
        tb = traceback.extract_tb(e.__traceback__)
        fn, ln, fc, tx = tb[-1]
        log.error(f"  ↳ {fn} line {ln} in {fc}: {tx}")
    finally:
        pwm_h.set_PWM_dutycycle(pwm_pin, 0)
        pwm_h.write(direction_pin, 0)
        log.info("[servoDrive] motors stopped safely")
        pwm_h.stop()


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    log.info("=" * 65)
    log.info("  WRO 2025  —  Obstacle Challenge  |  OpenCV HSV Edition")
    log.info("=" * 65)

    try:
        P = multiprocessing.Process(
            target=Live_Feed_OpenCV,
            args=(red_b, green_b, pink_b,
                  centr_y, centr_x,
                  centr_y_red, centr_x_red,
                  centr_x_pink, centr_y_pink),
            name="P_LiveFeed"
        )
        S = multiprocessing.Process(
            target=servoDrive,
            args=(red_b, green_b, pink_b,
                  counts, centr_y, centr_x,
                  centr_y_red, centr_x_red,
                  centr_x_pink, centr_y_pink,
                  head, sp_angle, turn_trigger,
                  lidar_f, lidar_l, lidar_r,
                  left_f, right_f, lane_counter),
            name="S_ServoDrive"
        )
        E = multiprocessing.Process(
            target=runEncoder,
            args=(counts, head, lane_counter, left_f, right_f),
            name="E_Encoder"
        )
        L = multiprocessing.Process(
            target=read_lidar,
            args=(lidar_angle, lidar_distance, sp_angle, turn_trigger,
                  lidar_f, lidar_l, lidar_r, left_f, right_f, head),
            name="L_Lidar"
        )

        P.start(); E.start(); L.start(); S.start()
        log.info("All processes started — waiting…")

        P.join(); E.join(); L.join(); S.join()

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt — terminating all processes")
        for proc in (S, P, E, L):
            proc.terminate()
        for proc in (S, P, E, L):
            proc.join()
        pwm_main.set_PWM_dutycycle(pwm_pin, 0)  # stop motor on shutdown
        pwm_main.stop()
        # tfmini uses bb_serial_read — close each pin individually and ignore
        # 'no serial in progress' errors (GPIO was never opened in main process)
        for _rx_pin in (RX_Head, RX_Left, RX_Right, RX_Back):
            try:
                pwm_main.bb_serial_read_close(_rx_pin)
            except Exception:
                pass
        # Serial port is owned by runEncoder child — already closed when it exited
        log.info("Clean shutdown complete")


# ═══════════════════════════════════════════════════════════════════════════════
#  SSH / VS CODE HEADLESS — LINES TO COMMENT OUT
# ═══════════════════════════════════════════════════════════════════════════════
#
#  When running over SSH (no display / no Qt), comment out these lines
#  inside Live_Feed_OpenCV() to disable the camera preview window:
#
#    SHOW_WINDOW = True          →  change to:  SHOW_WINDOW = False
#
#  That single change suppresses ALL of the following automatically:
#    - cv2.namedWindow(...)
#    - cv2.resizeWindow(...)
#    - canvas = frame_bgr.copy()
#    - _draw_detections(...)
#    - cv2.imshow(...)
#    - cv2.waitKey(...)
#    - cv2.destroyAllWindows()   (in the finally block)
#
#  Additionally, comment out or remove these two lines at the TOP of the file
#  if Qt / DISPLAY is not configured on your SSH session:
#
#    os.environ["DISPLAY"]         = ":0"        ← only needed for local display
#    os.environ["QT_QPA_PLATFORM"] = "xcb"       ← only needed for local display
#
#  Everything else (detection, shared values, telemetry, drive logic) works
#  identically over SSH with no further changes.
#
# ═══════════════════════════════════════════════════════════════════════════════