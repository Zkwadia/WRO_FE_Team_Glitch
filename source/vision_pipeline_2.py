"""
vision_pipeline.py
──────────────────
All OpenCV / colour-detection logic for WRO 2025 Obstacle Challenge.
Import into the main file; never run directly.

Usage:
    from vision_pipeline import VisionPipeline, FrameSmoother
"""

from __future__ import annotations

import os
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTS  (same names as original globals — edit here only)
# ─────────────────────────────────────────────────────────────────────────────

USE_LAB = False
 
FRAME_WIDTH      = 640
FRAME_HEIGHT     = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2
SMOOTH_N         = 2

HSV_RANGES = {
    'LOWER_RED_1':   np.array([  6,  97,  41]),  'UPPER_RED_1':   np.array([  4, 255, 168]),
    'LOWER_RED_2':   np.array([170,  97,  41]),  'UPPER_RED_2':   np.array([180, 255, 168]),
    'LOWER_GREEN':   np.array([ 41,  82,  10]),  'UPPER_GREEN':   np.array([ 96, 255, 227]),
    'LOWER_BLACK':   np.array([  0,   0,   0]),  'UPPER_BLACK':   np.array([179,  65,  97]),
    'LOWER_ORANGE':  np.array([  8,  84,  20]),  'UPPER_ORANGE':  np.array([ 16, 255, 255]),
    'LOWER_MAGENTA': np.array([128,  98,  35]),  'UPPER_MAGENTA': np.array([169, 237, 188]),
    'LOWER_BLUE':    np.array([ 88,  56,  58]),  'UPPER_BLUE':    np.array([130, 226, 185]),
}

LAB_RANGES = {
    'LOWER_RED_1':   np.array([  6,  97,  41]),  'UPPER_RED_1':   np.array([  4, 255, 168]),
    'LOWER_RED_2':   np.array([170,  97,  41]),  'UPPER_RED_2':   np.array([180, 255, 168]),
    'LOWER_GREEN':   np.array([ 41,  82,  10]),  'UPPER_GREEN':   np.array([ 96, 255, 227]),
    'LOWER_BLACK':   np.array([  0,   0,   0]),  'UPPER_BLACK':   np.array([179,  65,  97]),
    'LOWER_ORANGE':  np.array([  8,  84,  20]),  'UPPER_ORANGE':  np.array([ 16, 255, 255]),
    'LOWER_MAGENTA': np.array([128,  98,  35]),  'UPPER_MAGENTA': np.array([169, 237, 188]),
    'LOWER_BLUE':    np.array([ 88,  56,  58]),  'UPPER_BLUE':    np.array([130, 226, 185]),
}

ROI = {
    "wall_left":        ( 0, 160, 135, 150),
    "wall_right":       (505, 160, 135, 150),
    "wall_inner_left":  (0, 285, 120, 100),
    "wall_inner_right": (520, 285, 120, 100),
    "line":             (250, 320, 180,  40), #140
    "close_block":      (250, 220, 140,  40),#(250, 200, 140,  40),
    "main_blocks":      ( 0, 100, 640, 320),
    "close_black":      (250, 270, 140,  40),
}

MIN_AREA = {
    "wall":        300,
    "block":       1000,
    "close_block": 100,
    "magenta":     500,
    "line":        500,
}

COLORS = {
    "red":     (  0,   0, 255),
    "green":   (  0, 255,   0),
    "orange":  (  0, 165, 255),
    "blue":    (255,   0,   0),
    "black":   ( 255,  255,  255),
    "magenta": (255,   0, 255),
    "roi":     (0, 255,   255),
    "text":    (255, 255, 255),
}

_GLOBAL_Y_OFFSET = 100
_GLOBAL_Y_END    = FRAME_HEIGHT   # 360 for 640×360

_GLOBAL_X_OFFSET = 0
_GLOBAL_X_END    = FRAME_WIDTH   # 360 for 640×360

# ─────────────────────────────────────────────────────────────────────────────
# MORPHOLOGY KERNEL  (shared — matches hsv_calibrate_all.py exactly)
# ─────────────────────────────────────────────────────────────────────────────

_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# ─────────────────────────────────────────────────────────────────────────────
# FRAME SMOOTHER
# ─────────────────────────────────────────────────────────────────────────────

class FrameSmoother:
    """Majority-vote smoother over N frames per colour."""

    def __init__(self, n: int) -> None:
        self.n    = max(1, n)
        self._buf: Dict[str, deque] = {}

    def update(self, name: str, detected: bool) -> bool:
        if name not in self._buf:
            self._buf[name] = deque(maxlen=self.n)
        self._buf[name].append(detected)
        return sum(self._buf[name]) > (len(self._buf[name]) // 2)


# ─────────────────────────────────────────────────────────────────────────────
# VISION PIPELINE CLASS
# ─────────────────────────────────────────────────────────────────────────────

class VisionPipeline:
    """
    Encapsulates detect(), annotate(), find_biggest(), find_all_above(),
    convert_frame(), red_mask(), and open_camera() — all formerly free
    functions inside Live_Feed.

    Usage inside Live_Feed:
        pipeline   = VisionPipeline(USE_LAB)
        detections = pipeline.detect(frame)
        annotated  = pipeline.annotate(frame, detections, USE_LAB, fps)
        cap        = pipeline.open_camera()
    """

    def __init__(self, use_lab: bool = USE_LAB) -> None:
        self.use_lab      = use_lab
        self.color_ranges = LAB_RANGES if use_lab else HSV_RANGES

        # Unpack to instance attrs — same names as the original module-level vars
        r = self.color_ranges
        self.LOWER_RED_1   = r['LOWER_RED_1'];   self.UPPER_RED_1   = r['UPPER_RED_1']
        self.LOWER_RED_2   = r['LOWER_RED_2'];   self.UPPER_RED_2   = r['UPPER_RED_2']
        self.LOWER_GREEN   = r['LOWER_GREEN'];   self.UPPER_GREEN   = r['UPPER_GREEN']
        self.LOWER_BLACK   = r['LOWER_BLACK'];   self.UPPER_BLACK   = r['UPPER_BLACK']
        self.LOWER_ORANGE  = r['LOWER_ORANGE'];  self.UPPER_ORANGE  = r['UPPER_ORANGE']
        self.LOWER_MAGENTA = r['LOWER_MAGENTA']; self.UPPER_MAGENTA = r['UPPER_MAGENTA']
        self.LOWER_BLUE    = r['LOWER_BLUE'];    self.UPPER_BLUE    = r['UPPER_BLUE']

    # ── colour-space helpers ──────────────────────────────────────────────────

    def convert_frame(self, frame: np.ndarray) -> np.ndarray:
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
        m1 = cv2.inRange(converted_crop, self.LOWER_RED_1, self.UPPER_RED_1)
        if self.use_lab:
            return self._clean_mask(m1)
        m2 = cv2.inRange(converted_crop, self.LOWER_RED_2, self.UPPER_RED_2)
        return self._clean_mask(cv2.bitwise_or(m1, m2))

    # ── contour helpers ───────────────────────────────────────────────────────

    @staticmethod
    def find_biggest(mask: np.ndarray, min_area: int) -> Optional[Tuple]:
        """Return (contour, area, cx, cy) of the biggest contour above min_area, or None."""
        if cv2.countNonZero(mask) == 0:
            return None
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c    = max(contours, key=cv2.contourArea)
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
        """Return list of (contour, area, cx, cy) for ALL contours above min_area."""
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

    # ── detect ────────────────────────────────────────────────────────────────

    def detect(self, frame: np.ndarray) -> Dict[str, List[Dict]]:
        """
        Run full detection pipeline on one (already resized) frame.
        Returns dict: color → list of dicts {contour, area, centroid, label, zone}
        """
        results = {k: [] for k in ("red", "green", "orange", "blue", "black", "magenta")}

        G_Y0, G_Y1 = _GLOBAL_Y_OFFSET, _GLOBAL_Y_END
        G_X0, G_X1 = _GLOBAL_X_OFFSET, _GLOBAL_X_END

        frame_slice = frame[G_Y0:G_Y1, :]
        frame_slice = cv2.GaussianBlur(frame_slice, (3, 3), 0)
        conv        = self.convert_frame(frame_slice)

        def crop(roi_key: str):
            """Crop from the global slice. Returns (crop, offset_x, offset_y_global)."""
            rx, ry, rw, rh = ROI[roi_key]
            ry_slice = max(0, ry - G_Y0)
            return conv[ry_slice: ry_slice + rh, rx: rx + rw], rx, ry

        def push(color_name, mask, ox, oy, min_area, zone, label_prefix):
            found = self.find_biggest(mask, min_area)
            if found:
                c, area, cx, cy = found
                results[color_name].append({
                    "contour":  c + [ox, oy],
                    "area":     area,
                    "centroid": (cx + ox, cy + oy),
                    "label":    f"{label_prefix} {area:.0f}px",
                    "zone":     zone,
                })

        # ── 1. Main blocks (red, green, magenta) ──────────────────────────────
        main_crop, ox, oy = crop("main_blocks")
        # red_mask() applies _clean_mask internally
        mask_red_main   = self.red_mask(main_crop)
        mask_green_main = self._clean_mask(cv2.inRange(main_crop, self.LOWER_GREEN,   self.UPPER_GREEN))
        mask_mag_main   = self._clean_mask(cv2.inRange(main_crop, self.LOWER_MAGENTA, self.UPPER_MAGENTA))

        push("red",     mask_red_main,   ox, oy, MIN_AREA["block"],   "main",  "red block")
        push("green",   mask_green_main, ox, oy, MIN_AREA["block"],   "main",  "green block")
        push("magenta", mask_mag_main,   ox, oy, MIN_AREA["magenta"], "main",  "magenta")

        # ── 2. Close blocks (red, green, magenta) ─────────────────────────────
        close_crop, ox2, oy2 = crop("close_block")
        # red_mask() applies _clean_mask internally
        mask_red_close   = self.red_mask(close_crop)
        mask_green_close = self._clean_mask(cv2.inRange(close_crop, self.LOWER_GREEN,   self.UPPER_GREEN))
        mask_mag_close   = self._clean_mask(cv2.inRange(close_crop, self.LOWER_MAGENTA, self.UPPER_MAGENTA))

        push("red",     mask_red_close,   ox2, oy2, MIN_AREA["close_block"], "close", "CLOSE red")
        push("green",   mask_green_close, ox2, oy2, MIN_AREA["close_block"], "close", "CLOSE green")
        push("magenta", mask_mag_close,   ox2, oy2, MIN_AREA["close_block"], "close", "CLOSE magenta")

        # ── 3. Floor lines (orange, blue) ─────────────────────────────────────
        line_crop, lox, loy = crop("line")
        mask_orange = cv2.inRange(line_crop, self.LOWER_ORANGE, self.UPPER_ORANGE)
        mask_blue   = cv2.inRange(line_crop, self.LOWER_BLUE,   self.UPPER_BLUE)

        push("orange", mask_orange, lox, loy, MIN_AREA["line"], "line", "orange line")
        push("blue",   mask_blue,   lox, loy, MIN_AREA["line"], "line", "blue line")

        # ── 4. Walls (black) ──────────────────────────────────────────────────
        # NOTE: _clean_mask is applied per zone_mask (not on the full-frame
        # mask_black_full) to avoid merging black regions across zone boundaries.
        mask_black_full = cv2.inRange(conv, self.LOWER_BLACK, self.UPPER_BLACK)

        mx, my, mw, mh = ROI["main_blocks"]
        mys = max(0, my - G_Y0)
        combined_rg = np.zeros_like(mask_black_full)
        combined_rg[mys: mys + mh, mx: mx + mw] = cv2.bitwise_or(mask_red_main, mask_green_main)
        pure_black = cv2.bitwise_and(mask_black_full, cv2.bitwise_not(combined_rg))

        for zone_name in ("wall_left", "wall_right", "wall_inner_left", "wall_inner_right"):
            wx, wy, ww, wh = ROI[zone_name]
            wys = max(0, wy - G_Y0)
            sub_mask = np.zeros_like(pure_black)
            cv2.rectangle(sub_mask, (wx, wys), (wx + ww, wys + wh), 255, -1)
            zone_mask = cv2.bitwise_and(pure_black, sub_mask)
            zone_mask = self._clean_mask(zone_mask)   # ← morph applied per-zone
            for c, area, wcx, wcy in self.find_all_above(zone_mask, MIN_AREA["wall"]):
                results["black"].append({
                    "contour":  c + [0, G_Y0],
                    "area":     area,
                    "centroid": (wcx, wcy + G_Y0),
                    "label":    f"{zone_name} {area:.0f}px",
                    "zone":     zone_name,
                })

        # ── 5. Close black (wall dead ahead) ──────────────────────────────────
        cbx, cby, cbw, cbh = ROI["close_black"]
        cbys = max(0, cby - G_Y0)
        close_black_roi_mask = np.zeros_like(pure_black)
        cv2.rectangle(close_black_roi_mask, (cbx, cbys), (cbx + cbw, cbys + cbh), 255, -1)
        final_close_black = cv2.bitwise_and(pure_black, close_black_roi_mask)
        final_close_black = self._clean_mask(final_close_black)   # ← morph applied per-zone

        for c, area, cbcx, cbcy in self.find_all_above(final_close_black, MIN_AREA["wall"]):
            results["black"].append({
                "contour":  c + [0, G_Y0],
                "area":     area,
                "centroid": (cbcx, cbcy + G_Y0),
                "label":    f"close wall {area:.0f}px",
                "zone":     "close_black",
            })

        # ── 6. Red centroid edge filter ────────────────────────────────────────
        if results["red"]:
            filtered_red_centr = []
            for rd in results["red"]:
                rcx, rcy = rd["centroid"]
                too_close_centr = False  # rcx < 10
                if not too_close_centr:
                    filtered_red_centr.append(rd)
            results["red"] = filtered_red_centr

        # ── 7. Proximity check — discard red if magenta detected nearby ────────
        '''if results["red"] and results["magenta"]:
            filtered_red = []
            for rd in results["red"]:
                rcx, rcy = rd["centroid"]
                too_close = any(
                    abs(rcx - md["centroid"][0]) < 200 and abs(rcy - md["centroid"][1]) < 130 and md["centroid"][0] < 320
                    for md in results["magenta"]
                )
                if not too_close:
                    filtered_red.append(rd)
            results["red"] = filtered_red'''

        return results

    # ── annotate ──────────────────────────────────────────────────────────────

    def annotate(self, frame: np.ndarray, detections: Dict, use_lab: bool, fps: float, time) -> np.ndarray:
        out = frame.copy()

        # Draw all ROI boxes
        for zone_name, (rx, ry, rw, rh) in ROI.items():
            cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), COLORS["roi"], 1)
            cv2.putText(out, zone_name, (rx + 2, ry + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, COLORS["roi"], 1, cv2.LINE_AA)

        # Centre midpoint line
        cv2.line(out, (FRAME_MIDPOINT_X, 0), (FRAME_MIDPOINT_X, FRAME_HEIGHT), (80, 80, 80), 1)

        # Contours + labels
        for color_name, detects in detections.items():
            draw_color = COLORS.get(color_name, (200, 200, 200))
            for d in detects:
                #print(f"{color_name} contour min_x={d['contour'][:,:,0].min()} max_x={d['contour'][:,:,0].max()}")
                min_x = d["contour"][:,:,0].min()
                max_x = d["contour"][:,:,0].max()
                if min_x < 20: 
                    # draw bounding rect snapped to left edge
                    x, y, w, h = cv2.boundingRect(d["contour"].reshape(-1,1,2).astype(np.int32))
                    cv2.rectangle(out, (0, y), (x + w, y + h), draw_color, 2)
                elif max_x > 620:
                    # draw bounding rect snapped to left edge
                    x, y, w, h = cv2.boundingRect(d["contour"].reshape(-1,1,2).astype(np.int32))
                    cv2.rectangle(out, (x, y), (x + w, y + h), draw_color, 2)
                else:
                    cv2.drawContours(out, [d["contour"].reshape(-1,1,2).astype(np.int32)], -1, draw_color, 2)
                #cv2.drawContours(out, [d["contour"].reshape(-1,1,2).astype(np.int32)], -1, draw_color, 2)
                cx, cy = d["centroid"]
                cv2.circle(out, (cx, cy), 5, draw_color, -1)
                label_y = max(cy - 8, 15)
                cv2.putText(out, d["label"], (cx - 20, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, draw_color, 1, cv2.LINE_AA)
                cv2.putText(out, ("("+str(cx)+", "+str(cy)+")"), (cx - 20, label_y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, draw_color, 1, cv2.LINE_AA)


        # Debug panel (top-left)
        panel_x, panel_y = 5, 18
        cv2.putText(out, str(time), (panel_x, panel_y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,  (255, 0, 0), 1, cv2.LINE_AA)
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
            cv2.putText(out, line, (panel_x, panel_y + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, COLORS["text"], 1, cv2.LINE_AA)'''

        # Legend (bottom-left)
        '''legend = [
            ("Red block",    COLORS["red"]),
            ("Green block",  COLORS["green"]),
            ("Orange line",  COLORS["orange"]),
            ("Blue line",    COLORS["blue"]),
            ("Wall (black)", (80, 80, 80)),
            ("Magenta",      COLORS["magenta"]),
            ("ROI zones",    COLORS["roi"]),
        ]
        leg_x       = 5
        leg_y_start = FRAME_HEIGHT - len(legend) * 16 - 4
        overlay2    = out.copy()
        cv2.rectangle(overlay2, (0, leg_y_start - 4), (130, FRAME_HEIGHT), (0, 0, 0), -1)
        cv2.addWeighted(overlay2, 0.5, out, 0.5, 0, out)
        for i, (name, col) in enumerate(legend):
            y = leg_y_start + i * 16
            cv2.rectangle(out, (leg_x, y - 9), (leg_x + 12, y + 1), col, -1)
            cv2.putText(out, name, (leg_x + 16, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, COLORS["text"], 1, cv2.LINE_AA)'''

        return out

    # ── camera factory ────────────────────────────────────────────────────────

    def open_camera(self) -> Optional[cv2.VideoCapture]:
        """Scan indices 0-5 for the first working capture device, configure and return it."""
        for _idx in range(6):
            _c = cv2.VideoCapture(_idx)
            if _c.isOpened():
                ret_test, _ = _c.read()
                if ret_test:
                    _c.set(cv2.CAP_PROP_FRAME_WIDTH,   FRAME_WIDTH)
                    _c.set(cv2.CAP_PROP_FRAME_HEIGHT,  FRAME_HEIGHT)
                    _c.set(cv2.CAP_PROP_FPS,           120)
                    _c.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                    _c.set(cv2.CAP_PROP_EXPOSURE,      -3)
                    _c.set(cv2.CAP_PROP_BRIGHTNESS, 200)
                    _c.set(cv2.CAP_PROP_BUFFERSIZE,    1)
                    actual_w = int(_c.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(_c.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    print(f"[Live_Feed] using camera index {_idx}  ({actual_w}×{actual_h})")
                    return _c
                _c.release()
        print("[Live_Feed] No working camera found on indices 0-5 — check connection")
        return None
