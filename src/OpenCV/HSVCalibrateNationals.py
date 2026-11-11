"""
HSVCalibrateNationals.py
======================
Faster live HSV calibration for all 6 colours
Controls:
    1  →  RED
    2  →  GREEN
    3  →  PINK / MAGENTA
    4  →  ORANGE
    5  →  BLUE
    6  →  BLACK
    click on video (left pane)  →  sample HSV at that point, auto-set trackbars
    TAB      →  cycle which bound is "active" for fine-tuning (shown in label)
    - / =    →  nudge active bound by 1   (fine adjustment)
    [ / ]    →  nudge active bound by 5   (coarse adjustment)
    a        →  toggle tiled "all colours" overview
    s        →  write current HSV values directly into both pipeline files
    q        →  quit

Typical flow: click on the object to get close instantly, then TAB to the
bound that's clipping the mask and nudge with -/=/[/] until the mask is
clean. Mouse-dragging the trackbars still works too, for big changes.
"""

import re
import cv2
import numpy as np

# pipeline paths
PIPELINE_PATH_2 = r"/home/pi/WRO_2026/versionTest/vision_pipeline_3.py" #replace with your own open pipeline path or use ours
PIPELINE_PATH_3 = r"/home/pi/WRO_2026/versionTest/vision_pipeline_new.py" #replace with your own obstacle pipeline path or use ours

CAM_INDEX = 0
W, H = 640, 360

# How wide a window (in HSV units) to draw around a clicked pixel.
# Tune these if click-to-sample windows feel too tight/loose.
CLICK_MARGIN_H = 10
CLICK_MARGIN_S = 60
CLICK_MARGIN_V = 60
SAMPLE_BOX = 4  # sample an SxS box around the click and average, not just 1px


def nothing(x):
    pass


# Hardcoded fallback defaults - only used if we can't read prior values back from either pipeline file (e.g. first calibration ever, or file missing).
DEFAULT_COLOURS = {
    "1_RED": {"lower": [0, 120, 60], "upper": [10, 255, 255], "display": (0, 0, 220)},
    "2_GREEN": {"lower": [40, 80, 40], "upper": [90, 255, 200], "display": (0, 220, 0)},
    "3_MAGENTA": {"lower": [116, 172, 52], "upper": [168, 240, 162], "display": (220, 0, 220)},
    "4_ORANGE": {"lower": [6, 70, 20], "upper": [26, 255, 255], "display": (0, 140, 255)},
    "5_BLUE": {"lower": [94, 45, 58], "upper": [140, 226, 185], "display": (255, 0, 0)},
    "6_BLACK": {"lower": [0, 0, 0], "upper": [180, 255, 40], "display": (255, 0, 0)},
}
KEY_MAP = {ord("1"): "1_RED", ord("2"): "2_GREEN", ord("3"): "3_MAGENTA", ord("4"): "4_ORANGE", ord("5"): "5_BLUE", ord("6"): "6_BLACK"}

# Maps each colour key → which HSV_RANGES keys to update in vision_pipeline.py
PIPELINE_KEY_MAP = {
    "1_RED": [("LOWER_RED_1", lambda l, u: l), ("UPPER_RED_1", lambda l, u: u), ("LOWER_RED_2", lambda l, u: [170, l[1], l[2]]), ("UPPER_RED_2", lambda l, u: [180, u[1], u[2]])],
    "2_GREEN": [("LOWER_GREEN", lambda l, u: l), ("UPPER_GREEN", lambda l, u: u)],
    "3_MAGENTA": [("LOWER_MAGENTA", lambda l, u: l), ("UPPER_MAGENTA", lambda l, u: u)],
    "4_ORANGE": [("LOWER_ORANGE", lambda l, u: l), ("UPPER_ORANGE", lambda l, u: u)],
    "5_BLUE": [("LOWER_BLUE", lambda l, u: l), ("UPPER_BLUE", lambda l, u: u)],
    "6_BLACK": [("LOWER_BLACK", lambda l, u: l), ("UPPER_BLACK", lambda l, u: u)],
}

# Reverse map: for loading, we only need the PRIMARY (non-red-wrap) key per colour to recover lower/upper. Red is special-cased.
LOAD_KEY_MAP = {"1_RED": ("LOWER_RED_1", "UPPER_RED_1"), "2_GREEN": ("LOWER_GREEN", "UPPER_GREEN"), "3_MAGENTA": ("LOWER_MAGENTA", "UPPER_MAGENTA"), "4_ORANGE": ("LOWER_ORANGE", "UPPER_ORANGE"), "5_BLUE": ("LOWER_BLUE", "UPPER_BLUE"), "6_BLACK": ("LOWER_BLACK", "UPPER_BLACK")}


def _find_dict_block(source: str, var_name: str):
    """
    Find the span of a top-level dict literal like `HSV_RANGES = { ... }`.
    Returns (start, end) character indices for the CONTENT between the
    braces (exclusive), or None if not found. Handles nested brackets
    (e.g. np.array([...])) correctly by only brace-matching on { }.
    """
    m = re.search(r"\b" + re.escape(var_name) + r"\s*=\s*\{", source)
    if not m:
        return None
    start = m.end()
    depth = 1
    i = start
    while i < len(source) and depth > 0:
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
        i += 1
    if depth != 0:
        return None  # unbalanced braces — bail out safely
    end = i - 1
    return start, end


def _extract_array(source: str, key_name: str):
    """Pull the [h, s, v] list out of a line like:
       'LOWER_RED_1': np.array([0, 120, 60]),
    Returns a list of 3 ints, or None if not found / unparsable.
    NOTE: caller is responsible for passing only the HSV_RANGES block
    (not the full file source) so LAB_RANGES entries with the same key
    names aren't picked up by accident.
    """
    pattern = r"[\"']" + re.escape(key_name) + r"[\"']\s*:\s*np\.array\(\[([^\]]*)\]"
    m = re.search(pattern, source)
    if not m:
        return None
    try:
        vals = [int(v.strip()) for v in m.group(1).split(",")]
        if len(vals) == 3:
            return vals
    except ValueError:
        pass
    return None


# Which files to try loading FROM, in order, and what to call them in logs. Change if your path is different
LOAD_SOURCES = [(PIPELINE_PATH_2, "vision_pipeline_3.py (open_2.py)"), (PIPELINE_PATH_3, "vision_pipeline_new.py (obstacle_3.py)")]


def load_last_calibration():
    """
    Try to read the last-saved HSV values back out of the two pipeline
    files actually used in practice (vision_pipeline_3 / vision_pipeline_new),
    so students start from where they left off instead of hardcoded
    defaults. Falls back to DEFAULT_COLOURS per-colour (or entirely) on
    any failure. Tries each colour independently against each source in
    order, so a partially-edited file still yields a full load.
    """
    colours = {k: {"lower": list(v["lower"]), "upper": list(v["upper"]), "display": v["display"]} for k, v in DEFAULT_COLOURS.items()}

    sources = []
    for path, label in LOAD_SOURCES:
        try:
            with open(path, "r", newline="") as f:
                sources.append((label, f.read()))
        except FileNotFoundError:
            print(f"[INFO] {label} not found — skipping as a load source.")

    if not sources:
        print("[INFO] No pipeline files found — using hardcoded defaults for all colours.")
        return colours

    loaded, skipped = [], []
    for colour_key, (lo_key, up_key) in LOAD_KEY_MAP.items():
        found = False
        for label, source in sources:
            block = _find_dict_block(source, "HSV_RANGES")
            if block is None:
                continue  # couldn't find HSV_RANGES in this file — try next source
            block_text = source[block[0] : block[1]]
            lo = _extract_array(block_text, lo_key)
            up = _extract_array(block_text, up_key)
            if lo is not None and up is not None:
                colours[colour_key]["lower"] = lo
                colours[colour_key]["upper"] = up
                loaded.append(f"{colour_key} (from {label})")
                found = True
                break
        if not found:
            skipped.append(colour_key)

    if loaded:
        print("[LOADED] Starting from last-saved values:")
        for entry in loaded:
            print(f"          - {entry}")
    if skipped:
        print(f"[INFO] Using hardcoded defaults for: {', '.join(skipped)} (not found in any pipeline file)")
    return colours


def write_to_pipeline(colour_key: str, lower: list, upper: list) -> bool:
    """
    Surgically overwrite the np.array([...]) values for the given colour
    inside the HSV_RANGES block ONLY (not LAB_RANGES, even though it shares
    the same key names) in both pipeline files.
    """
    for path in (PIPELINE_PATH_2, PIPELINE_PATH_3):
        try:
            # newline="" preserves the file's existing CRLF/LF endings so a save
            # touches only the lines we changed, not all 1000 in git.
            with open(path, "r", newline="") as f:
                source = f.read()
        except FileNotFoundError:
            print(f"[ERROR] Could not find {path}")
            return False

        block = _find_dict_block(source, "HSV_RANGES")
        if block is None:
            print(f"[ERROR] Could not locate HSV_RANGES block in {path} — file left untouched")
            return False
        b_start, b_end = block
        block_text = source[b_start:b_end]

        missing = []
        for dict_key, val_fn in PIPELINE_KEY_MAP[colour_key]:
            vals = val_fn(lower, upper)
            pattern = r"([\"'])" + re.escape(dict_key) + r"\1(\s*:\s*np\.array\()\[[^\]]*\]"
            replacement = r"\g<1>" + dict_key + r"\g<1>\g<2>" + f"[{vals[0]:3d}, {vals[1]:3d}, {vals[2]:3d}]"
            new_block_text, n = re.subn(pattern, replacement, block_text)
            if n == 0:
                missing.append(dict_key)
            elif n > 1:
                print(f"[ERROR] Key '{dict_key}' appears {n}x in HSV_RANGES in {path} — refusing to write, fix the duplicate first")
                return False
            else:
                block_text = new_block_text

        # Don't half-write. If any key for this colour is missing, the file
        # doesn't match what we expect — leave it alone and say so loudly.
        if missing:
            print(f"[ERROR] SAVE FAILED for {colour_key} in {path}")
            print(f"        Missing key(s) in HSV_RANGES: {', '.join(missing)}")
            print(f"        File left UNTOUCHED. Check the key names/quoting in HSV_RANGES.")
            return False

        source = source[:b_start] + block_text + source[b_end:]

        with open(path, "w", newline="") as f:
            f.write(source)
        print(f"[SAVED] {colour_key} → {path}")

    return True


# ── State ─────────────────────────────────────────────────────────────────
COLOURS = load_last_calibration()
current = "1_RED"
WIN = "HSV Calibrate (fast)"
show_overview = False
last_click_hsv = None  # for on-screen feedback

# ── Keyboard fine-tuning ─────────────────────────────────────────────────
# TAB cycles through these; -/= nudge by 1, [/] nudge by 5.
NUDGE_CHANNELS = ["H_low", "H_high", "S_low", "S_high", "V_low", "V_high"]
NUDGE_MAX = {"H_low": 179, "H_high": 179, "S_low": 255, "S_high": 255, "V_low": 255, "V_high": 255}
active_channel_idx = 0


def nudge_active(delta):
    name = NUDGE_CHANNELS[active_channel_idx]
    cur = cv2.getTrackbarPos(name, WIN)
    new = max(0, min(NUDGE_MAX[name], cur + delta))
    cv2.setTrackbarPos(name, WIN, new)


cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, 200)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 255)
cap.set(cv2.CAP_PROP_FPS, 20)
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, W * 2, H + 60)


def set_trackbars(name):
    d = COLOURS[name]
    cv2.setTrackbarPos("H_low", WIN, d["lower"][0])
    cv2.setTrackbarPos("H_high", WIN, d["upper"][0])
    cv2.setTrackbarPos("S_low", WIN, d["lower"][1])
    cv2.setTrackbarPos("S_high", WIN, d["upper"][1])
    cv2.setTrackbarPos("V_low", WIN, d["lower"][2])
    cv2.setTrackbarPos("V_high", WIN, d["upper"][2])


cv2.createTrackbar("H_low", WIN, 0, 179, nothing)
cv2.createTrackbar("H_high", WIN, 179, 179, nothing)
cv2.createTrackbar("S_low", WIN, 0, 255, nothing)
cv2.createTrackbar("S_high", WIN, 255, 255, nothing)
cv2.createTrackbar("V_low", WIN, 0, 255, nothing)
cv2.createTrackbar("V_high", WIN, 255, 255, nothing)
set_trackbars(current)


def on_mouse(event, x, y, flags, param):
    """Click-to-sample: click anywhere on the LEFT (video) pane to set the
    trackbars from the HSV of the pixels under the click."""
    global last_click_hsv
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    if x >= W or y >= H:  # click was on the mask pane or label bar — ignore
        return
    frame = param["frame"]
    if frame is None:
        return
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    half = SAMPLE_BOX // 2
    y0, y1 = max(0, y - half), min(H, y + half + 1)
    x0, x1 = max(0, x - half), min(W, x + half + 1)
    patch = hsv[y0:y1, x0:x1].reshape(-1, 3)
    h, s, v = [int(round(c)) for c in patch.mean(axis=0)]
    last_click_hsv = (h, s, v)

    hl = max(0, h - CLICK_MARGIN_H)
    hh = min(179, h + CLICK_MARGIN_H)
    sl = max(0, s - CLICK_MARGIN_S)
    sh = min(255, s + CLICK_MARGIN_S)
    vl = max(0, v - CLICK_MARGIN_V)
    vh = min(255, v + CLICK_MARGIN_V)

    cv2.setTrackbarPos("H_low", WIN, hl)
    cv2.setTrackbarPos("H_high", WIN, hh)
    cv2.setTrackbarPos("S_low", WIN, sl)
    cv2.setTrackbarPos("S_high", WIN, sh)
    cv2.setTrackbarPos("V_low", WIN, vl)
    cv2.setTrackbarPos("V_high", WIN, vh)
    print(f"[SAMPLE] Clicked pixel HSV=({h},{s},{v}) → window set for {current}")


mouse_param = {"frame": None}
cv2.setMouseCallback(WIN, on_mouse, mouse_param)

print("Keys: 1=RED 2=GREEN 3=MAGENTA 4=ORANGE 5=BLUE 6=BLACK | click=sample | a=overview | s=save | q=quit")


def make_mask_for(colour_key, hsv):
    d = COLOURS[colour_key]
    lo, up = np.array(d["lower"], dtype=np.uint8), np.array(d["upper"], dtype=np.uint8)
    if colour_key == "1_RED":
        mask = cv2.bitwise_or(cv2.inRange(hsv, lo, up), cv2.inRange(hsv, np.array([170, d["lower"][1], d["lower"][2]]), np.array([180, d["upper"][1], d["upper"][2]])))
    else:
        mask = cv2.inRange(hsv, lo, up)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask


# ── Main loop ────────────────────────────────────────────────────────────
while True:
    cap.grab()
    ok, frame = cap.retrieve()
    if not ok:
        continue
    frame = cv2.resize(frame, (W, H))
    proc = cv2.GaussianBlur(frame, (1, 7), 0)

    mouse_param["frame"] = proc

    hl = cv2.getTrackbarPos("H_low", WIN)
    hh = cv2.getTrackbarPos("H_high", WIN)
    sl = cv2.getTrackbarPos("S_low", WIN)
    sh = cv2.getTrackbarPos("S_high", WIN)
    vl = cv2.getTrackbarPos("V_low", WIN)
    vh = cv2.getTrackbarPos("V_high", WIN)
    COLOURS[current]["lower"] = [hl, sl, vl]
    COLOURS[current]["upper"] = [hh, sh, vh]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if show_overview:
        # 3x2 tiled grid of all 6 masks so can triage at a glance.
        tile_w, tile_h = W // 3, H // 2
        tiles = []
        for key in ["1_RED", "2_GREEN", "3_MAGENTA", "4_ORANGE", "5_BLUE", "6_BLACK"]:
            m = make_mask_for(key, hsv)
            m_small = cv2.resize(m, (tile_w, tile_h))
            m_bgr = cv2.cvtColor(m_small, cv2.COLOR_GRAY2BGR)
            label = key.split("_")[1]
            cv2.putText(m_bgr, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOURS[key]["display"], 1)
            if key == current:
                cv2.rectangle(m_bgr, (0, 0), (tile_w - 1, tile_h - 1), (0, 255, 255), 2)
            tiles.append(m_bgr)
        row1 = np.hstack(tiles[0:3])
        row2 = np.hstack(tiles[3:6])
        grid = np.vstack([row1, row2])
        bar = np.zeros((22, grid.shape[1], 3), dtype=np.uint8)
        cv2.putText(bar, "OVERVIEW — press 'a' to go back to single-colour view", (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        combined = np.vstack([bar, grid])
        cv2.imshow(WIN, combined)
    else:
        mask = make_mask_for(current, hsv)
        overlay = frame.copy()
        col = COLOURS[current]["display"]
        conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in conts:
            if cv2.contourArea(c) > 300:
                cv2.drawContours(overlay, [c], -1, col, 2)
                M = cv2.moments(c)
                if M["m00"]:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(overlay, (cx, cy), 6, (255, 255, 255), -1)
                    cv2.putText(overlay, f"A:{int(cv2.contourArea(c))}", (cx - 20, cy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(overlay, f"({cx}, {cy})", (cx - 20, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        sample_txt = f" last-click:{last_click_hsv}" if last_click_hsv else ""
        active_name = NUDGE_CHANNELS[active_channel_idx]
        bounds_str = ""
        for name, val in [("H_low", hl), ("H_high", hh), ("S_low", sl), ("S_high", sh), ("V_low", vl), ("V_high", vh)]:
            bounds_str += f"[{val}]" if name == active_name else f" {val} "
            bounds_str += " "
        label = f"  [{current.split('_')[1]}]  {bounds_str}" f"| TAB=select -/=nudge1 [/]nudge5 click=sample a=overview s=save q=quit{sample_txt}"
        cv2.rectangle(overlay, (0, 0), (W, 22), (30, 30, 30), -1)
        cv2.putText(overlay, label, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
        cv2.putText(overlay, f"active: {active_name}", (4, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([overlay, mask_bgr])
        cv2.imshow(WIN, combined)

    key = cv2.waitKey(1) & 0xFF

    if key in KEY_MAP:
        current = KEY_MAP[key]
        set_trackbars(current)
        last_click_hsv = None
        print(f"Switched to: {current}")

    elif key == 9:  # TAB
        active_channel_idx = (active_channel_idx + 1) % len(NUDGE_CHANNELS)
        print(f"[FINE-TUNE] Active bound: {NUDGE_CHANNELS[active_channel_idx]}")

    elif key == ord("-"):
        nudge_active(-1)
    elif key == ord("="):
        nudge_active(+1)
    elif key == ord("["):
        nudge_active(-5)
    elif key == ord("]"):
        nudge_active(+5)

    elif key == ord("a"):
        show_overview = not show_overview
        print(f"[VIEW] Overview mode {'ON' if show_overview else 'OFF'}")

    elif key == ord("s"):
        lower_list = COLOURS[current]["lower"]
        upper_list = COLOURS[current]["upper"]
        ok = write_to_pipeline(current, lower_list, upper_list)
        if ok:
            print(f"[SAVED] {current} → {PIPELINE_PATH_2}")
            print(f"[SAVED] {current} → {PIPELINE_PATH_3}")
        else:
            print(f"[FAILED] Could not write {current} to one or more pipeline files")

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
