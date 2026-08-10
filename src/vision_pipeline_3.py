"""
vision_pipeline.py
------------------
All OpenCV / colour-detection logic for WRO 2025 Obstacle Challenge.
Import into the main file; never run directly.

Usage:
    from vision_pipeline import VisionPipeline, FrameSmoother

ROI Layout (640x360 frame)
--------------------------
Detection only runs on the frame slice from y=100 downward (_GLOBAL_Y_OFFSET).
The main_blocks zone uses an irregular polygon (not a rectangle) to exclude
a trapezoidal dead zone at the bottom-centre of the frame -- this prevents
false positives from close-up floor content directly ahead of the robot.

    x=0          x=250  x=390       x=640
     |              |      |             |
y=100+--------------------------------------------+  <- detection starts
     |                                             |
     |           MAIN BLOCKS ROI                  |  (red / green / magenta)
     |                                             |
y=295|              +------+                       |  <- notch top (platform)
     |             /        \\                      |
y=360+------------/          \\---------------------+  <- frame bottom
     x=180                   x=460

     +--------- dead zone (notch) ----------------+
       blocks here are too close / on
       the floor and should be ignored
"""

from __future__ import annotations

import os
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# TUNABLE CONSTANTS
# Edit values here only — never scatter magic numbers through the rest of the
# code.  All coordinates are in full-frame pixels (origin = top-left of the
# 640×360 image).
# ─────────────────────────────────────────────────────────────────────────────

USE_LAB = False  # False → HSV detection (faster, good indoors)
# True  → LAB detection (more lighting-robust)

FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2
SMOOTH_N = 0  # majority-vote window for FrameSmoother

# ── HSV colour ranges ─────────────────────────────────────────────────────────
# Format: np.array([H, S, V]).  H ∈ [0,180], S/V ∈ [0,255] in OpenCV HSV.
# Red wraps around H=0/180, so two ranges are ORed together.
HSV_RANGES = {
    'LOWER_RED_1':   np.array([ 11,  43,  55]),  'UPPER_RED_1':   np.array([  8, 218, 224]),
    'LOWER_RED_2':   np.array([170,  43,  55]),  'UPPER_RED_2':   np.array([180, 218, 224]),
    'LOWER_GREEN':   np.array([ 29,  65,  15]),  'UPPER_GREEN':   np.array([ 98, 252, 197]),
    'LOWER_BLACK':   np.array([  0,   0,  15]),  'UPPER_BLACK':   np.array([134,  79, 116]),
    'LOWER_ORANGE':  np.array([  8,  41, 174]),  'UPPER_ORANGE':  np.array([ 17, 170, 255]),
    'LOWER_MAGENTA': np.array([157, 138,  62]),  'UPPER_MAGENTA': np.array([169, 210, 255]),
    'LOWER_BLUE':    np.array([ 89,  28,  45]),  'UPPER_BLUE':    np.array([134, 194, 220]),
}

LAB_RANGES = {
    'LOWER_RED_1':   np.array([ 11,  43,  55]),  'UPPER_RED_1':   np.array([  8, 218, 224]),
    'LOWER_RED_2':   np.array([170,  43,  55]),  'UPPER_RED_2':   np.array([180, 218, 224]),
    'LOWER_GREEN':   np.array([ 29,  65,  15]),  'UPPER_GREEN':   np.array([ 98, 252, 197]),
    'LOWER_BLACK':   np.array([  0,   0,  15]),  'UPPER_BLACK':   np.array([134,  79, 116]),
    'LOWER_ORANGE':  np.array([  8,  41, 174]),  'UPPER_ORANGE':  np.array([ 17, 170, 255]),
    'LOWER_MAGENTA': np.array([157, 138,  62]),  'UPPER_MAGENTA': np.array([169, 210, 255]),
    'LOWER_BLUE':    np.array([ 89,  28,  45]),  'UPPER_BLUE':    np.array([134, 194, 220]),
}

# ── Rectangular ROI zones ─────────────────────────────────────────────────────
# Format: (x, y, width, height) in full-frame pixels.
# These are used for walls, the floor line strip, and close-range blocks.
# The main_blocks zone is a POLYGON — see MAIN_BLOCKS_POLY below.
ROI = {
    "wall_left": (0, 160, 135, 150),
    "wall_right": (505, 160, 135, 150),
    "wall_inner_left": (0, 285, 100, 100),
    "wall_inner_right": (505, 285, 135, 100),
    "line": (150, 280, 60, 20),#(250, 220, 155, 20),
    "line_2": (430, 280, 60, 20),
    "close_block": (250, 100, 140, 40),
    "close_black": (250, 100, 140, 40),
    "parking_wall": (250, 100, 140, 40),
    "last_wall": (280, 20, 80, 20),
    "last_wall_2": (260, 90, 80, 20),

}

# ── Polygonal ROI for main block detection ────────────────────────────────────
# All coordinates are in FRAME-SLICE space (y values have _GLOBAL_Y_OFFSET
# already subtracted, because the polygon mask is applied to frame_slice).
#
# Shape: full-width rectangle with a trapezoidal notch cut from the bottom
# centre.  The notch excludes the area directly in front of the robot where
# floor content / very-close blocks cause false detections.
#
#   Slice coords (y = frame_y − 100):
#
#   (0,0) ──────────────────────────────── (640,0)
#     │                                        │
#     │           detection zone               │
#     │                                        │
#   (0,260) ─── (180,260)          (460,260) ── (640,260)
#                        \        /
#                    (250,195)──(390,195)
#                         notch top
#
# To adjust the notch:
#   • x=180 / x=460  →  width of notch at the bottom (wider = more excluded)
#   • x=250 / x=390  →  width of the raised platform (narrower = steeper walls)
#   • y=195           →  height of the notch (lower value = taller notch)
MAIN_BLOCKS_POLY = np.array(
    [
        [  0,   0],   # ① top-left
        [640,   0],   # ② top-right
        [640, 360],   # ③ bottom-right
        [425, 360],   # ④ notch: bottom-right corner
        [425, 250],   # ⑤ notch: top-right corner
        [215, 250],   # ⑥ notch: top-left corner
        [215, 360],   # ⑦ notch: bottom-left corner
        [  0, 360],   # ⑧ bottom-left
    ],
    dtype=np.int32,
)

# ── Minimum contour areas ─────────────────────────────────────────────────────
# Contours smaller than these thresholds are discarded as noise.
MIN_AREA = {
    "wall": 800 ,  # black wall segments
    "block": 800,  # red / green blocks in main zone
    "close_block": 100,  # blocks in close_block zone (larger apparent size)
    "magenta": 500,  # magenta parking marker
    "line": 100,  # floor line strip (very thin ROI → small area)
    "LastWall": 100,
    "LastWall2": 200,
    
}

# ── Draw colours (BGR) ───────────────────────────────────────────────────────
COLORS = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "orange": (0, 165, 255),
    "blue": (255, 0, 0),
    "black": (255, 255, 255),
    "magenta": (255, 0, 255),
    "roi": (255, 255, 0),
    "text": (255, 255, 255),
}

# ── Global detection Y bounds ─────────────────────────────────────────────────
# Detection only runs on frame[_GLOBAL_Y_OFFSET : _GLOBAL_Y_END, :].
# Anything above y=100 is ignored (sky / top of arena wall with no useful info).
_GLOBAL_Y_OFFSET = 10
_GLOBAL_Y_END = FRAME_HEIGHT  # 360 for 640×360



# ─────────────────────────────────────────────────────────────────────────────
# MORPHOLOGY KERNEL  (shared — matches hsv_calibrate_all.py exactly)
# ─────────────────────────────────────────────────────────────────────────────

_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))



# ─────────────────────────────────────────────────────────────────────────────
# FRAME SMOOTHER
# ─────────────────────────────────────────────────────────────────────────────


class FrameSmoother:
    """
    Majority-vote temporal smoother over the last N frames per colour.

    Prevents single-frame flickers from triggering downstream logic.
    update() returns True only if more than half of the buffered frames
    reported a detection for that colour name.
    """

    def __init__(self, n: int) -> None:
        self.n = max(1, n)
        self._buf: Dict[str, deque] = {}

    def update(self, name: str, detected: bool) -> bool:
        if name not in self._buf:
            self._buf[name] = deque(maxlen=self.n)
        self._buf[name].append(detected)
        return sum(self._buf[name]) > (len(self._buf[name]) // 2)


# ─────────────────────────────────────────────────────────────────────────────
# VISION PIPELINE
# ─────────────────────────────────────────────────────────────────────────────


class VisionPipeline:
    """
    Full colour-detection pipeline for one camera frame.

    Encapsulates:
        detect()        — run all colour masks, return structured detections
        annotate()      — draw bounding boxes, labels, and debug overlay
        find_biggest()  — largest contour above min_area
        find_all_above() — all contours above min_area
        convert_frame() — BGR → HSV or LAB
        red_mask()      — dual-range red mask (handles H-channel wrap)
        open_camera()   — scan indices 0-5, configure and return VideoCapture

    Typical usage inside Live_Feed:
        pipeline   = VisionPipeline(USE_LAB)
        detections = pipeline.detect(frame)
        annotated  = pipeline.annotate(frame, detections, USE_LAB, fps)
        cap        = pipeline.open_camera()
    """

    def __init__(self, use_lab: bool = USE_LAB) -> None:
        self.use_lab = use_lab
        self.color_ranges = LAB_RANGES if use_lab else HSV_RANGES

        # Unpack colour bounds to instance attributes for fast lookup
        r = self.color_ranges
        self.LOWER_RED_1 = r["LOWER_RED_1"]
        self.UPPER_RED_1 = r["UPPER_RED_1"]
        self.LOWER_RED_2 = r["LOWER_RED_2"]
        self.UPPER_RED_2 = r["UPPER_RED_2"]
        self.LOWER_GREEN = r["LOWER_GREEN"]
        self.UPPER_GREEN = r["UPPER_GREEN"]
        self.LOWER_BLACK = r["LOWER_BLACK"]
        self.UPPER_BLACK = r["UPPER_BLACK"]
        self.LOWER_ORANGE = r["LOWER_ORANGE"]
        self.UPPER_ORANGE = r["UPPER_ORANGE"]
        self.LOWER_MAGENTA = r["LOWER_MAGENTA"]
        self.UPPER_MAGENTA = r["UPPER_MAGENTA"]
        self.LOWER_BLUE = r["LOWER_BLUE"]
        self.UPPER_BLUE = r["UPPER_BLUE"]

        # Pre-build the polygon mask at init time so detect() doesn't allocate
        # a new array every frame.  Shape is (slice_height, FRAME_WIDTH).
        slice_h = _GLOBAL_Y_END - _GLOBAL_Y_OFFSET  # 260 px
        self._main_poly_mask = np.zeros((slice_h, FRAME_WIDTH), dtype=np.uint8)
        cv2.fillPoly(self._main_poly_mask, [MAIN_BLOCKS_POLY], 255)

    # ── Colour-space helpers ──────────────────────────────────────────────────

    def convert_frame(self, frame: np.ndarray) -> np.ndarray:
        """Convert BGR frame to HSV or LAB depending on self.use_lab."""
        if self.use_lab:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    @staticmethod
    def _clean_mask(mask: np.ndarray) -> np.ndarray:
        """
        Apply the same open→close morphology used in hsv_calibrate_all.py
        so that detection matches exactly what the calibration preview shows.
            MORPH_OPEN  — removes small noise specks (erode then dilate)
            MORPH_CLOSE — fills small holes inside blobs (dilate then erode)
        """
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _MORPH_KERNEL, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL, iterations=1)
        return mask


    def red_mask(self, converted_crop: np.ndarray) -> np.ndarray:
        """
        Build a binary red mask.

        In HSV, red wraps around H=0 so two inRange() calls are ORed.
        In LAB only the first range is used (update LAB_RANGES as needed).
        """
        m1 = cv2.inRange(converted_crop, self.LOWER_RED_1, self.UPPER_RED_1)
        if self.use_lab:
            return self._clean_mask(m1)
        m2 = cv2.inRange(converted_crop, self.LOWER_RED_2, self.UPPER_RED_2)
        return self._clean_mask(cv2.bitwise_or(m1, m2))

    # ── Contour helpers ───────────────────────────────────────────────────────

    @staticmethod
    def find_biggest(mask: np.ndarray, min_area: int) -> Optional[Tuple]:
        """
        Return (contour, area, cx, cy) for the single largest contour whose
        area exceeds min_area, or None if nothing qualifies.
        """
        if cv2.countNonZero(mask) == 0:
            return None
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < min_area:
            return None
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return c, area, cx, cy

    @staticmethod
    def find_all_above(mask: np.ndarray, min_area: int) -> List[Tuple]:
        """
        Return a list of (contour, area, cx, cy) for every contour whose
        area exceeds min_area.  Used for wall zones where multiple segments
        can appear simultaneously.
        """
        results = []
        if cv2.countNonZero(mask) == 0:
            return results
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            results.append((c, area, cx, cy))
        return results

    # ── Main detection pipeline ───────────────────────────────────────────────

    def detect(self, frame: np.ndarray) -> Dict[str, List[Dict]]:
        """
        Run the full detection pipeline on one (already-resized) BGR frame.

        Pipeline order
        ──────────────
        1. Main blocks  — red, green, magenta inside the polygon ROI
        2. Close blocks — same colours in the narrow close_block strip
        3. Floor lines  — orange and blue in the thin line strip
        4. Walls        — black in the four wall zones (red/green subtracted)
        5. Close black  — wall directly ahead in the close_black tripwire

        Post-filters
        ────────────
        • Red centroids with cx < 50 are discarded (extreme left edge noise)
        • Red detections within 200 px (x) / 130 px (y) of a magenta detection
          are discarded (magenta parking marker mis-classified as red)

        Returns
        ───────
        dict mapping colour name → list of detection dicts, each containing:
            contour  : np.ndarray  contour points in FULL-FRAME coordinates
            area     : float       contour area in px²
            centroid : (int, int)  (cx, cy) in full-frame coordinates
            label    : str         human-readable label + area for the overlay
            zone     : str         which ROI zone this detection came from
        """
        results = {
            k: [] for k in ("red", "green", "orange", "blue", "black", "magenta")
        }

        # ── Slice the frame to the active detection region ─────────────────
        # Everything above _GLOBAL_Y_OFFSET is skipped (no useful info there).
        G_Y0, G_Y1 = _GLOBAL_Y_OFFSET, _GLOBAL_Y_END
        frame_slice = frame[G_Y0:G_Y1, :]

        # Vertical Gaussian blur reduces horizontal stripe noise from lighting
        # without blurring left-right edges that define block boundaries.
        frame_slice = cv2.GaussianBlur(frame_slice, (1, 7), 0)

        # Convert to the working colour space once; reuse for all masks
        conv = self.convert_frame(frame_slice)

        # ── Helper: crop a rectangular ROI from the converted slice ────────
        def crop(roi_key: str):
            """
            Return (cropped_conv, offset_x, offset_y_fullframe) for a
            rectangular ROI defined in ROI[].  The y offset is adjusted for
            the global slice so the crop aligns with frame_slice.
            """
            rx, ry, rw, rh = ROI[roi_key]
            ry_slice = max(0, ry - G_Y0)
            return conv[ry_slice : ry_slice + rh, rx : rx + rw], rx, ry

        # ── Helper: find biggest contour and push into results ──────────────
        def push(color_name, mask, ox, oy, min_area, zone, label_prefix):
            """
            Run find_biggest() on mask, offset the contour and centroid back
            to full-frame coordinates, and append to results[color_name].
            ox / oy are the pixel offsets to add (roi_x, roi_y for rect crops;
            0, G_Y0 for polygon-masked full-slice detections).
            """
            found = self.find_biggest(mask, min_area)
            if found:
                c, area, cx, cy = found
                results[color_name].append(
                    {
                        "contour": c + [ox, oy],
                        "area": area,
                        "centroid": (cx + ox, cy + oy),
                        "label": f"{label_prefix} {area:.0f}px",
                        "zone": zone,
                    }
                )

        # ══════════════════════════════════════════════════════════════════════
        # 1. MAIN BLOCKS — red, green, magenta
        # ══════════════════════════════════════════════════════════════════════
        # Detection is restricted to the irregular polygon defined by
        # MAIN_BLOCKS_POLY (a rectangle with a trapezoidal notch cut from
        # the bottom centre).  The pre-built mask self._main_poly_mask is
        # ANDed with each colour mask so nothing outside the polygon fires.
        #
        # Offsets back to full-frame: ox=0, oy=G_Y0 (no x-crop, y was sliced)

        mask_red_main = cv2.bitwise_and(self.red_mask(conv), self._main_poly_mask)
        mask_green_main = cv2.bitwise_and(self._clean_mask(cv2.inRange(conv, self.LOWER_GREEN,   self.UPPER_GREEN)), self._main_poly_mask)
        mask_mag_main   = cv2.bitwise_and(self._clean_mask(cv2.inRange(conv, self.LOWER_MAGENTA, self.UPPER_MAGENTA)), self._main_poly_mask)

        ox_main, oy_main = 0, G_Y0  # slice has no x-offset; y needs G_Y0 added

        push(
            "red",
            mask_red_main,
            ox_main,
            oy_main,
            MIN_AREA["block"],
            "main",
            "red block",
        )
        push(
            "green",
            mask_green_main,
            ox_main,
            oy_main,
            MIN_AREA["block"],
            "main",
            "green block",
        )
        push(
            "magenta",
            mask_mag_main,
            ox_main,
            oy_main,
            MIN_AREA["magenta"],
            "main",
            "magenta",
        )

        # ══════════════════════════════════════════════════════════════════════
        # 2. CLOSE BLOCKS — red, green, magenta (lower min_area threshold)
        # ══════════════════════════════════════════════════════════════════════
        # A narrow horizontal strip just below the main zone catches blocks
        # that are very close to the robot.  Because they fill more of the
        # frame at short range, the min_area threshold is much lower (100 px²).

        close_crop, ox2, oy2 = crop("close_block")
        mask_red_close = self.red_mask(close_crop)
        mask_green_close = cv2.inRange(close_crop, self.LOWER_GREEN, self.UPPER_GREEN)
        mask_mag_close = cv2.inRange(close_crop, self.LOWER_MAGENTA, self.UPPER_MAGENTA)

        push(
            "red",
            mask_red_close,
            ox2,
            oy2,
            MIN_AREA["close_block"],
            "close",
            "CLOSE red",
        )
        push(
            "green",
            mask_green_close,
            ox2,
            oy2,
            MIN_AREA["close_block"],
            "close",
            "CLOSE green",
        )
        push(
            "magenta",
            mask_mag_close,
            ox2,
            oy2,
            MIN_AREA["close_block"],
            "close",
            "CLOSE magenta",
        )

        # ══════════════════════════════════════════════════════════════════════
        # 3. FLOOR LINES — orange, blue
        # ══════════════════════════════════════════════════════════════════════
        # The WRO mat has orange and blue lines that signal direction changes.
        # They are detected in a 140×10 px strip near the bottom of the frame.
        # min_area is only 20 px² because the ROI is tiny.

        line_crop, lox, loy = crop("line")
        mask_orange = self._clean_mask(cv2.inRange(line_crop, self.LOWER_ORANGE, self.UPPER_ORANGE))
        mask_blue = self._clean_mask(cv2.inRange(line_crop, self.LOWER_BLUE, self.UPPER_BLUE))

        push("orange", mask_orange, lox, loy, MIN_AREA["line"], "line", "orange line")
        push("blue", mask_blue, lox, loy, MIN_AREA["line"], "line", "blue line")

        line_crop_2, lox_2, loy_2 = crop("line_2")
        mask_orange_2 = self._clean_mask(cv2.inRange(line_crop_2, self.LOWER_ORANGE, self.UPPER_ORANGE))
        mask_blue_2 = self._clean_mask(cv2.inRange(line_crop_2, self.LOWER_BLUE, self.UPPER_BLUE))

        push("orange", mask_orange_2, lox_2, loy_2, MIN_AREA["line"], "line_2", "orange line")
        push("blue", mask_blue_2, lox_2, loy_2, MIN_AREA["line"], "line_2", "blue line")


        # ══════════════════════════════════════════════════════════════════════
        # 4. WALLS — black
        # ══════════════════════════════════════════════════════════════════════
        # Build a black mask over the full slice, then subtract pixels already
        # claimed by red or green main-block masks.  This prevents the dark
        # shadow under a coloured block from being mis-classified as a wall.
        #
        # The surviving "pure black" mask is then restricted to each of the
        # four wall ROI zones individually so we know which side the wall is on.

        mask_black_full = cv2.inRange(conv, self.LOWER_BLACK, self.UPPER_BLACK)

        # Subtract red+green from the polygon-masked detection area only.
        # Using the already-masked versions ensures we only suppress black
        # pixels that overlap with known block regions inside the polygon.
        combined_rg = cv2.bitwise_and(
            cv2.bitwise_or(mask_red_main, mask_green_main), self._main_poly_mask
        )
        pure_black = cv2.bitwise_and(mask_black_full, cv2.bitwise_not(combined_rg))

        # Iterate over the four wall zones, create a sub-mask for each, and
        # collect all qualifying contours (there may be more than one segment).
        for zone_name in (
            "wall_left",
            "wall_right",
            "wall_inner_left",
            "wall_inner_right",
        ):
            wx, wy, ww, wh = ROI[zone_name]
            wys = max(0, wy - G_Y0)  # convert ROI y to slice-space

            # Stamp the zone rectangle onto a blank mask, then AND with pure_black
            sub_mask = np.zeros_like(pure_black)
            cv2.rectangle(sub_mask, (wx, wys), (wx + ww, wys + wh), 255, -1)
            zone_mask = cv2.bitwise_and(pure_black, sub_mask)

            for c, area, wcx, wcy in self.find_all_above(zone_mask, MIN_AREA["wall"]):
                results["black"].append(
                    {
                        # Contour y is in slice space; add G_Y0 to return to full frame
                        "contour": c + [0, G_Y0],
                        "area": area,
                        "centroid": (wcx, wcy + G_Y0),
                        "label": f"{zone_name} {area:.0f}px",
                        "zone": zone_name,
                    }
                )

        # ══════════════════════════════════════════════════════════════════════
        # 5. CLOSE BLACK — wall directly ahead
        # ══════════════════════════════════════════════════════════════════════
        # A thin 360×10 px horizontal tripwire at y=140 detects a wall that
        # the robot is about to hit head-on.  Triggers an emergency stop or
        # early turn in the drive logic.

        cbx, cby, cbw, cbh = ROI["close_black"]
        cbys = max(0, cby - G_Y0)  # slice-space y

        close_black_roi_mask = np.zeros_like(pure_black)
        cv2.rectangle(
            close_black_roi_mask, (cbx, cbys), (cbx + cbw, cbys + cbh), 255, -1
        )
        final_close_black = cv2.bitwise_and(pure_black, close_black_roi_mask)

        for c, area, cbcx, cbcy in self.find_all_above(
            final_close_black, MIN_AREA["wall"]
        ):
            results["black"].append(
                {
                    "contour": c + [0, G_Y0],
                    "area": area,
                    "centroid": (cbcx, cbcy + G_Y0),
                    "label": f"close wall {area:.0f}px",
                    "zone": "close_black",
                }
            )

        # ══════════════════════════════════════════════════════════════════════
        # 5. PARKING BLACK — wall directly ahead
        # ══════════════════════════════════════════════════════════════════════
        # A thin 360×10 px horizontal tripwire at y=140 detects a wall that
        # the robot is about to hit head-on.  Triggers an emergency stop or
        # early turn in the drive logic.

        cbx, cby, cbw, cbh = ROI["parking_wall"]
        cbys = max(0, cby - G_Y0)  # slice-space y

        park_black_roi_mask = np.zeros_like(pure_black)
        cv2.rectangle(
            park_black_roi_mask, (cbx, cbys), (cbx + cbw, cbys + cbh), 255, -1
        )
        final_park_black = cv2.bitwise_and(pure_black, park_black_roi_mask)

        for c, area, cbcx, cbcy in self.find_all_above(
            final_park_black, MIN_AREA["wall"]
        ):
            results["black"].append(
                {
                    "contour": c + [0, G_Y0],
                    "area": area,
                    "centroid": (cbcx, cbcy + G_Y0),
                    "label": f"park wall {area:.0f}px",
                    "zone": "parking_wall",
                }
            )


        # ══════════════════════════════════════════════════════════════════════
        # 5. LAST BLACK — wall directly ahead
        # ══════════════════════════════════════════════════════════════════════
        # A thin 360×10 px horizontal tripwire at y=140 detects a wall that
        # the robot is about to hit head-on.  Triggers an emergency stop or
        # early turn in the drive logic.

        cbx, cby, cbw, cbh = ROI["last_wall"]
        cbys = max(0, cby - G_Y0)  # slice-space y

        last_black_roi_mask = np.zeros_like(pure_black)
        cv2.rectangle(
            last_black_roi_mask, (cbx, cbys), (cbx + cbw, cbys + cbh), 255, -1
        )
        final_last_black = cv2.bitwise_and(pure_black, last_black_roi_mask)

        for c, area, cbcx, cbcy in self.find_all_above(
            final_last_black, MIN_AREA["LastWall"]
        ):
            results["black"].append(
                {
                    "contour": c + [0, G_Y0],
                    "area": area,
                    "centroid": (cbcx, cbcy + G_Y0),
                    "label": f"last wall {area:.0f}px",
                    "zone": "last_wall",
                }
            )

        cbx, cby, cbw, cbh = ROI["last_wall_2"]
        cbys = max(0, cby - G_Y0)  # slice-space y

        last_black_roi_mask2 = np.zeros_like(pure_black)
        cv2.rectangle(
            last_black_roi_mask2, (cbx, cbys), (cbx + cbw, cbys + cbh), 255, -1
        )
        final_last_black2 = cv2.bitwise_and(pure_black, last_black_roi_mask2)

        for c, area, cbcx, cbcy in self.find_all_above(
            final_last_black2, MIN_AREA["LastWall2"]
        ):
            results["black"].append(
                {
                    "contour": c + [0, G_Y0],
                    "area": area,
                    "centroid": (cbcx, cbcy + G_Y0),
                    "label": f"last wall 2 {area:.0f}px",
                    "zone": "last_wall_2",
                }
            )



        # ══════════════════════════════════════════════════════════════════════
        # POST-DETECTION FILTERS
        # ══════════════════════════════════════════════════════════════════════

        # Filter 1: discard red detections whose centroid is within 50 px of
        # the left edge.  This removes partial reflections off the left wall.
        if results["red"]:
            results["red"] = [rd for rd in results["red"] if rd["centroid"][0] >= 50]

        # Filter 2: discard red detections that are spatially close to a
        # magenta detection.  The magenta parking marker has some red in its
        # spectrum; this prevents it from registering as a red obstacle block.
        if results["red"] and results["magenta"]:
            filtered_red = []
            for rd in results["red"]:
                rcx, rcy = rd["centroid"]
                too_close = any(
                    abs(rcx - md["centroid"][0]) < 200
                    and abs(rcy - md["centroid"][1]) < 130
                    for md in results["magenta"]
                )
                if not too_close:
                    filtered_red.append(rd)
            results["red"] = filtered_red

        return results

    # ── Annotation overlay ────────────────────────────────────────────────────

    def annotate(
        self, frame: np.ndarray, detections: Dict, use_lab: bool, fps: float, time, heading
    ) -> np.ndarray:
        """
        Draw all detections, ROI zones, and a debug panel onto a copy of frame.

        The polygon main_blocks ROI outline is drawn in addition to the
        standard rectangular ROI boxes so it is visible in the live feed.

        Returns the annotated frame (does not modify the input in place).
        """
        out = frame.copy()

        # ── Draw rectangular ROI boxes ─────────────────────────────────────
        for zone_name, (rx, ry, rw, rh) in ROI.items():
            cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), COLORS["roi"], 1)
            cv2.putText(
                out,
                zone_name,
                (rx + 2, ry + 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.28,
                COLORS["roi"],
                1,
                cv2.LINE_AA,
            )

        # ── Draw the polygonal main_blocks ROI outline ─────────────────────
        # Shift MAIN_BLOCKS_POLY back to full-frame coordinates for drawing
        poly_full_frame = MAIN_BLOCKS_POLY.copy()
        poly_full_frame[:, 1] += _GLOBAL_Y_OFFSET  # add Y offset back
        cv2.polylines(
            out,
            [poly_full_frame],
            isClosed=True,
            color=(0, 220, 220),
            thickness=1,
            lineType=cv2.LINE_AA,
        )
        cv2.putText(
            out,
            "main_blocks (poly)",
            (poly_full_frame[0][0] + 2, poly_full_frame[0][1] + 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.28,
            (0, 220, 220),
            1,
            cv2.LINE_AA,
        )

        # ── Centre midpoint line ───────────────────────────────────────────
        cv2.line(
            out,
            (FRAME_MIDPOINT_X, 0),
            (FRAME_MIDPOINT_X, FRAME_HEIGHT),
            (80, 80, 80),
            1,
        )

        # ── Draw contours and centroids for each detected colour ───────────
        for color_name, detects in detections.items():
            draw_color = COLORS.get(color_name, (200, 200, 200))
            for d in detects:
                cv2.drawContours(out, [d["contour"]], -1, draw_color, 2)
                cx, cy = d["centroid"]
                cv2.circle(out, (cx, cy), 5, draw_color, -1)
                label_y = max(cy - 8, 15)
                cv2.putText(
                    out,
                    d["label"],
                    (cx - 20, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    draw_color,
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(out, ("("+str(cx)+", "+str(cy)+")"), (cx - 20, label_y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, draw_color, 1, cv2.LINE_AA)


        # ── Debug panel (top-left semi-transparent overlay) ───────────────
        '''panel_lines = [
            f"FPS: {fps:.1f}",
            f"Mode: {'LAB' if use_lab else 'HSV'}  [L to toggle]",
            f"Red blocks:   {len(detections['red'])}",
            f"Green blocks: {len(detections['green'])}",
            f"Orange lines: {len(detections['orange'])}",
            f"Blue lines:   {len(detections['blue'])}",
            f"Walls:        {len(detections['black'])}",
            f"Magenta:      {len(detections['magenta'])}",
        ]
        panel_x, panel_y = 5, 18
        overlay = out.copy()
        cv2.rectangle(overlay, (0, 0), (200, len(panel_lines) * 16 + 6), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, out, 0.5, 0, out)
        for i, line in enumerate(panel_lines):
            cv2.putText(
                out,
                line,
                (panel_x, panel_y + i * 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                COLORS["text"],
                1,
                cv2.LINE_AA,
            )'''
        # Debug panel (top-left)
        panel_x, panel_y = 5, 18
        cv2.putText(out, f"time: {time:.2f}", (panel_x, panel_y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,  (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(out, f"heading: {heading:.2f}", (panel_x, panel_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.42,  (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(out, f"fps: {fps:.1f}", (panel_x, panel_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.42,  (255, 0, 0), 1, cv2.LINE_AA)

        return out

    def open_camera(self) -> Optional[cv2.VideoCapture]:
        """Scan indices 0-5 for the first working capture device, configure and return it."""
        for _idx in range(10):
            _c = cv2.VideoCapture(_idx)
            if _c.isOpened():
                ret_test, _ = _c.read()
                if ret_test:
                    _c.set(cv2.CAP_PROP_FRAME_WIDTH,   FRAME_WIDTH)
                    _c.set(cv2.CAP_PROP_FRAME_HEIGHT,  FRAME_HEIGHT)
                    _c.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                    _c.set(cv2.CAP_PROP_FPS,           120)
                    _c.set(cv2.CAP_PROP_BUFFERSIZE,    1)
                    _c.set(cv2.CAP_PROP_EXPOSURE,      100)
                    _c.set(cv2.CAP_PROP_BRIGHTNESS, 230)
                    actual_w = int(_c.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(_c.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    print(f"[Live_Feed] using camera index {_idx}  ({actual_w}×{actual_h})")
                    return _c
                _c.release()
        print("[Live_Feed] No working camera found on indices 0-5 — check connection")
        return None
