"""HSV video calibrator for WRO Future Engineers.

Set VIDEO_DIR to the folder containing recorded runs.  The script loads the
most recently modified supported video in that folder, then starts PAUSED so
you can click an object and calibrate it frame by frame.

Controls
--------
Mouse buttons: PLAY, PAUSE, -2s, +2s
Left / right arrows: skip backward / forward by two seconds
Space or P: play/pause
1-6: choose RED, GREEN, MAGENTA, ORANGE, BLUE, or BLACK
Click left video pane (only while paused): sample HSV and set all sliders
Tab: select an HSV limit; -/= nudge it by 1; [/] nudge it by 5
A: show/hide all-colour mask overview
S: save the selected colour into both vision pipelines
Q or Esc: quit
"""

from __future__ import annotations

import re
from pathlib import Path

import cv2
import numpy as np


# Paste the folder containing your recorded runs here.  The newest video in this folder is selected automatically.  Leave it blank until you are ready.
VIDEO_DIR = r"/home/pi/wro_logs/videos"

# These are intentionally the same pipeline targets and HSV_RANGES-only saving
# method used by hsv_calibrate_fast.py.
PIPELINE_PATHS = (
    Path(r"/home/pi/WRO_2026/versionTest/vision_pipeline_3.py"), #replace this with your open challenge pipeline path
    Path(r"/home/pi/WRO_2026/versionTest/vision_pipeline_new.py"), #replace this with your obstacle challenge pipeline path
)

VIDEO_EXTENSIONS = {".mp4", ".avi", ".mov", ".mkv", ".m4v", ".webm"}
W, H = 640, 360
SKIP_SECONDS = 2.0
CLICK_MARGIN_H = 10
CLICK_MARGIN_S = 60
CLICK_MARGIN_V = 60
SAMPLE_BOX = 4
WIN = "HSV Video Calibrator"

DEFAULT_COLOURS = {
    "1_RED": {"lower": [0, 120, 60], "upper": [10, 255, 255], "display": (0, 0, 220)},
    "2_GREEN": {"lower": [40, 80, 40], "upper": [90, 255, 200], "display": (0, 220, 0)},
    "3_MAGENTA": {"lower": [116, 172, 52], "upper": [168, 240, 162], "display": (220, 0, 220)},
    "4_ORANGE": {"lower": [6, 70, 20], "upper": [26, 255, 255], "display": (0, 140, 255)},
    "5_BLUE": {"lower": [94, 45, 58], "upper": [140, 226, 185], "display": (255, 0, 0)},
    "6_BLACK": {"lower": [0, 0, 0], "upper": [180, 255, 40], "display": (120, 120, 120)},
}

KEY_MAP = {
    ord("1"): "1_RED", ord("2"): "2_GREEN", ord("3"): "3_MAGENTA",
    ord("4"): "4_ORANGE", ord("5"): "5_BLUE", ord("6"): "6_BLACK",
}

# Each entry gives the HSV_RANGES values to update for that colour.  Red has
# two ranges, so its high-hue wrap range updates with the same S and V bounds.
PIPELINE_KEY_MAP = {
    "1_RED": [
        ("LOWER_RED_1", lambda lo, hi: lo),
        ("UPPER_RED_1", lambda lo, hi: hi),
        ("LOWER_RED_2", lambda lo, hi: [170, lo[1], lo[2]]),
        ("UPPER_RED_2", lambda lo, hi: [180, hi[1], hi[2]]),
    ],
    "2_GREEN": [("LOWER_GREEN", lambda lo, hi: lo), ("UPPER_GREEN", lambda lo, hi: hi)],
    "3_MAGENTA": [("LOWER_MAGENTA", lambda lo, hi: lo), ("UPPER_MAGENTA", lambda lo, hi: hi)],
    "4_ORANGE": [("LOWER_ORANGE", lambda lo, hi: lo), ("UPPER_ORANGE", lambda lo, hi: hi)],
    "5_BLUE": [("LOWER_BLUE", lambda lo, hi: lo), ("UPPER_BLUE", lambda lo, hi: hi)],
    "6_BLACK": [("LOWER_BLACK", lambda lo, hi: lo), ("UPPER_BLACK", lambda lo, hi: hi)],
}
LOAD_KEY_MAP = {
    "1_RED": ("LOWER_RED_1", "UPPER_RED_1"),
    "2_GREEN": ("LOWER_GREEN", "UPPER_GREEN"),
    "3_MAGENTA": ("LOWER_MAGENTA", "UPPER_MAGENTA"),
    "4_ORANGE": ("LOWER_ORANGE", "UPPER_ORANGE"),
    "5_BLUE": ("LOWER_BLUE", "UPPER_BLUE"),
    "6_BLACK": ("LOWER_BLACK", "UPPER_BLACK"),
}

NUDGE_CHANNELS = ["H_low", "H_high", "S_low", "S_high", "V_low", "V_high"]
NUDGE_MAX = {"H_low": 179, "H_high": 179, "S_low": 255, "S_high": 255, "V_low": 255, "V_high": 255}

# cv2.waitKeyEx arrow codes are different for different operating systems, change it if required.
LEFT_ARROW_KEYS = {81, 2424832, 65361}
RIGHT_ARROW_KEYS = {83, 2555904, 65363}

TOOLBAR_H = 44
BUTTONS = {
    "PLAY": (8, 5, 96, 38),
    "PAUSE": (104, 5, 202, 38),
    "-2s": (210, 5, 298, 38),
    "+2s": (306, 5, 394, 38),
}


def nothing(_: int) -> None:
    """Trackbar callback. Values are read in the render loop."""


def find_dict_block(source: str, variable: str) -> tuple[int, int] | None:
    """Return the contents of a top-level ``VARIABLE = { ... }`` dict."""
    match = re.search(r"\b" + re.escape(variable) + r"\s*=\s*\{", source)
    if match is None:
        return None

    start = match.end()
    depth = 1
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return start, index
    return None


def extract_array(block_text: str, key: str) -> list[int] | None:
    """Read ``'KEY': np.array([h, s, v])`` safely from HSV_RANGES."""
    pattern = r"[\"']" + re.escape(key) + r"[\"']\s*:\s*np\.array\(\[([^\]]+)\]"
    match = re.search(pattern, block_text)
    if match is None:
        return None
    try:
        values = [int(value.strip()) for value in match.group(1).split(",")]
    except ValueError:
        return None
    return values if len(values) == 3 else None


def load_last_calibration() -> dict[str, dict[str, list[int] | tuple[int, int, int]]]:
    """Start with the last saved HSV values, with defaults as a safe fallback."""
    colours = {
        name: {"lower": data["lower"][:], "upper": data["upper"][:], "display": data["display"]}
        for name, data in DEFAULT_COLOURS.items()
    }

    for colour, (lower_key, upper_key) in LOAD_KEY_MAP.items():
        for path in PIPELINE_PATHS:
            try:
                source = path.read_text()
            except FileNotFoundError:
                continue
            block = find_dict_block(source, "HSV_RANGES")
            if block is None:
                continue
            hsv_block = source[block[0]:block[1]]
            lower, upper = extract_array(hsv_block, lower_key), extract_array(hsv_block, upper_key)
            if lower is not None and upper is not None:
                colours[colour]["lower"] = lower
                colours[colour]["upper"] = upper
                print(f"[LOADED] {colour} from {path}")
                break
    return colours


def updated_pipeline_text(path: Path, colour: str, lower: list[int], upper: list[int]) -> str | None:
    """Build a safe HSV_RANGES-only edit. Return None without touching a file on error."""
    try:
        # newline="" preserves existing CRLF/LF line endings, as in the
        # original calibrator. It avoids creating a whole-file git diff.
        with path.open("r", newline="") as source_file:
            source = source_file.read()
    except FileNotFoundError:
        print(f"[ERROR] Could not find {path}")
        return None

    block = find_dict_block(source, "HSV_RANGES")
    if block is None:
        print(f"[ERROR] No HSV_RANGES block found in {path}; it was not changed.")
        return None

    start, end = block
    block_text = source[start:end]
    for dict_key, value_fn in PIPELINE_KEY_MAP[colour]:
        values = value_fn(lower, upper)
        pattern = r"([\"'])" + re.escape(dict_key) + r"\1(\s*:\s*np\.array\()\[[^\]]*\]"
        replacement = rf"\g<1>{dict_key}\g<1>\g<2>[{values[0]:3d}, {values[1]:3d}, {values[2]:3d}]"
        block_text, changed = re.subn(pattern, replacement, block_text)
        if changed != 1:
            problem = "missing" if changed == 0 else f"found {changed} times"
            print(f"[ERROR] {dict_key} is {problem} in HSV_RANGES in {path}; it was not changed.")
            return None
    return source[:start] + block_text + source[end:]


def write_to_pipeline(colour: str, lower: list[int], upper: list[int]) -> bool:
    """Save using the reference script's HSV_RANGES-only replacement logic."""
    edits: list[tuple[Path, str]] = []
    for path in PIPELINE_PATHS:
        edited = updated_pipeline_text(path, colour, lower, upper)
        if edited is None:
            print("[FAILED] Nothing was saved. Fix the error above and try again.")
            return False
        edits.append((path, edited))

    for path, edited in edits:
        with path.open("w", newline="") as output_file:
            output_file.write(edited)
        print(f"[SAVED] {colour} -> {path}")
    return True


def newest_video(video_dir: str) -> Path:
    """Return the newest supported video directly inside VIDEO_DIR."""
    if not video_dir.strip():
        raise ValueError("VIDEO_DIR is empty. Paste the folder path at the top of this script first.")
    folder = Path(video_dir).expanduser()
    if not folder.is_dir():
        raise ValueError(f"VIDEO_DIR is not a folder: {folder}")

    videos = [path for path in folder.iterdir() if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS]
    if not videos:
        extensions = ", ".join(sorted(VIDEO_EXTENSIONS))
        raise FileNotFoundError(f"No supported video ({extensions}) found directly in {folder}")
    return max(videos, key=lambda path: path.stat().st_mtime)


def make_mask_for(colour: str, hsv: np.ndarray, colours: dict) -> np.ndarray:
    data = colours[colour]
    lower = np.array(data["lower"], dtype=np.uint8)
    upper = np.array(data["upper"], dtype=np.uint8)
    if colour == "1_RED":
        wrap_lower = np.array([170, lower[1], lower[2]], dtype=np.uint8)
        wrap_upper = np.array([180, upper[1], upper[2]], dtype=np.uint8)
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower, upper), cv2.inRange(hsv, wrap_lower, wrap_upper))
    else:
        mask = cv2.inRange(hsv, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)


def set_trackbars(colour: str, colours: dict) -> None:
    lower, upper = colours[colour]["lower"], colours[colour]["upper"]
    for name, value in zip(NUDGE_CHANNELS, (lower[0], upper[0], lower[1], upper[1], lower[2], upper[2])):
        cv2.setTrackbarPos(name, WIN, int(value))


def read_trackbars(colour: str, colours: dict) -> tuple[list[int], list[int]]:
    lower = [cv2.getTrackbarPos(name, WIN) for name in ("H_low", "S_low", "V_low")]
    upper = [cv2.getTrackbarPos(name, WIN) for name in ("H_high", "S_high", "V_high")]
    colours[colour]["lower"], colours[colour]["upper"] = lower, upper
    return lower, upper


def draw_button(image: np.ndarray, name: str, active: bool = False) -> None:
    x0, y0, x1, y1 = BUTTONS[name]
    colour = (35, 150, 35) if active else (70, 70, 70)
    cv2.rectangle(image, (x0, y0), (x1, y1), colour, -1)
    cv2.rectangle(image, (x0, y0), (x1, y1), (220, 220, 220), 1)
    text_size, _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
    text_x = x0 + (x1 - x0 - text_size[0]) // 2
    text_y = y0 + (y1 - y0 + text_size[1]) // 2
    cv2.putText(image, name, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)


def main() -> None:
    try:
        video_path = newest_video(VIDEO_DIR)
    except (ValueError, FileNotFoundError) as error:
        print(f"[ERROR] {error}")
        return

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"[ERROR] OpenCV could not open: {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0
        print("[WARNING] Video FPS was unavailable; using 30 FPS for two-second skips.")
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[VIDEO] Using newest file: {video_path.name}")
    print("[CONTROLS] Click PLAY/PAUSE/-2s/+2s. Calibration clicks work only while paused.")

    ok, current_frame = cap.read()
    if not ok:
        print(f"[ERROR] Could not read the first frame of {video_path}")
        cap.release()
        return
    current_frame = cv2.resize(current_frame, (W, H))
    current_index = 0
    last_slider_pos = 0

    is_playing = False
    show_overview = False
    current_colour = "1_RED"
    active_channel_index = 0
    last_click_hsv: tuple[int, int, int] | None = None
    colours = load_last_calibration()
    mouse_state = {"sample_frame": current_frame.copy(), "playing": is_playing, "overview": show_overview}

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, W * 2, H + TOOLBAR_H + 150)

    cv2.createTrackbar(
    "Video_Pos",
    WIN,
    0,
    max(frame_count - 1, 1),
    nothing
    )

    cv2.createTrackbar("H_low", WIN, 0, 179, nothing)
    cv2.createTrackbar("H_high", WIN, 179, 179, nothing)
    cv2.createTrackbar("S_low", WIN, 0, 255, nothing)
    cv2.createTrackbar("S_high", WIN, 255, 255, nothing)
    cv2.createTrackbar("V_low", WIN, 0, 255, nothing)
    cv2.createTrackbar("V_high", WIN, 255, 255, nothing)
    set_trackbars(current_colour, colours)

    def seek(seconds: float) -> None:
        nonlocal current_frame, current_index, last_slider_pos
        last_index = max(0, frame_count - 1)
        target = max(0, min(last_index, round(current_index + seconds * fps)))
        cap.set(cv2.CAP_PROP_POS_FRAMES, target)
        read_ok, frame = cap.read()
        if read_ok:
            current_frame = cv2.resize(frame, (W, H))
            current_index = target

            cv2.setTrackbarPos("Video_Pos", WIN, current_index)
            last_slider_pos = current_index

            print(f"[SEEK] {current_index / fps:.2f}s")
        else:
            print("[WARNING] Could not read that seek position.")

    def sample_click(x: int, y: int) -> None:
        nonlocal last_click_hsv
        if mouse_state["playing"]:
            print("[INFO] Pause the video before clicking to sample HSV.")
            return
        if mouse_state["overview"] or x >= W or y >= H:
            return

        sample_frame = mouse_state["sample_frame"]
        hsv = cv2.cvtColor(sample_frame, cv2.COLOR_BGR2HSV)
        half = SAMPLE_BOX // 2
        y0, y1 = max(0, y - half), min(H, y + half + 1)
        x0, x1 = max(0, x - half), min(W, x + half + 1)
        h, s, v = [int(round(value)) for value in hsv[y0:y1, x0:x1].reshape(-1, 3).mean(axis=0)]
        last_click_hsv = (h, s, v)

        values = {
            "H_low": max(0, h - CLICK_MARGIN_H), "H_high": min(179, h + CLICK_MARGIN_H),
            "S_low": max(0, s - CLICK_MARGIN_S), "S_high": min(255, s + CLICK_MARGIN_S),
            "V_low": max(0, v - CLICK_MARGIN_V), "V_high": min(255, v + CLICK_MARGIN_V),
        }
        for name, value in values.items():
            cv2.setTrackbarPos(name, WIN, value)
        print(f"[SAMPLE] HSV=({h}, {s}, {v}); sliders set for {current_colour}.")

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        nonlocal is_playing
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for name, (x0, y0, x1, y1) in BUTTONS.items():
            if x0 <= x <= x1 and H + y0 <= y <= H + y1:
                if name == "PLAY":
                    is_playing = True
                elif name == "PAUSE":
                    is_playing = False
                elif name == "-2s":
                    seek(-SKIP_SECONDS)
                elif name == "+2s":
                    seek(SKIP_SECONDS)
                return
        sample_click(x, y)

    cv2.setMouseCallback(WIN, on_mouse)

    while True:
        if is_playing:
            ok, frame = cap.read()
            if not ok:
                is_playing = False
                print("[VIDEO] End reached; video paused.")
            else:
                current_frame = cv2.resize(frame, (W, H))
                current_index = min(current_index + 1, frame_count - 1)

                cv2.setTrackbarPos("Video_Pos", WIN, current_index)
                last_slider_pos = current_index

        slider_pos = cv2.getTrackbarPos("Video_Pos", WIN)

        if abs(slider_pos - last_slider_pos) > 1:

            cap.set(cv2.CAP_PROP_POS_FRAMES, slider_pos)

            ok, frame = cap.read()

            if ok:
                current_frame = cv2.resize(frame, (W, H))
                current_index = slider_pos

            last_slider_pos = slider_pos

        lower, upper = read_trackbars(current_colour, colours)
        # Match the reference click calibration: it samples a narrow vertical
        # blur, while display and mask detection use the original frame.
        mouse_state["sample_frame"] = cv2.GaussianBlur(current_frame, (1, 7), 0)
        mouse_state["playing"], mouse_state["overview"] = is_playing, show_overview
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        if show_overview:
            tile_w, tile_h = W // 3, H // 2
            tiles = []
            for colour in DEFAULT_COLOURS:
                mask = make_mask_for(colour, hsv, colours)
                tile = cv2.cvtColor(cv2.resize(mask, (tile_w, tile_h)), cv2.COLOR_GRAY2BGR)
                label = colour.split("_", 1)[1]
                cv2.putText(tile, label, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colours[colour]["display"], 1)
                if colour == current_colour:
                    cv2.rectangle(tile, (0, 0), (tile_w - 1, tile_h - 1), (0, 255, 255), 2)
                tiles.append(tile)
            image = np.vstack((np.hstack(tiles[:3]), np.hstack(tiles[3:])))
            image = cv2.resize(image, (W * 2, H))
        else:
            mask = make_mask_for(current_colour, hsv, colours)
            overlay = current_frame.copy()
            contour_colour = colours[current_colour]["display"]
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > 300:
                    cv2.drawContours(overlay, [contour], -1, contour_colour, 2)
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            image = np.hstack((overlay, mask_bgr))

        toolbar = np.zeros((TOOLBAR_H, W * 2, 3), dtype=np.uint8)
        draw_button(toolbar, "PLAY", is_playing)
        draw_button(toolbar, "PAUSE", not is_playing)
        draw_button(toolbar, "-2s")
        draw_button(toolbar, "+2s")
        status = "PLAYING" if is_playing else "PAUSED - click left pane to sample HSV"
        time_text = f"{current_index / fps:.2f}s / {frame_count / fps:.2f}s"
        cv2.putText(toolbar, f"{status}  |  {time_text}  |  arrows: +/-2s", (410, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (255, 255, 255), 1)
        cv2.putText(toolbar, "Space/P: play-pause  |  A: masks  |  S: save  |  Q: quit", (410, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (185, 185, 185), 1)
        combined = np.vstack((image, toolbar))

        label = current_colour.split("_", 1)[1]
        active = NUDGE_CHANNELS[active_channel_index]
        click_text = f"  last HSV: {last_click_hsv}" if last_click_hsv else ""
        cv2.rectangle(combined, (0, 0), (W * 2, 22), (25, 25, 25), -1)
        cv2.putText(combined, f"{label} | {lower} - {upper} | active: {active}{click_text}", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colours[current_colour]["display"], 1)
        cv2.imshow(WIN, combined)

        key = cv2.waitKeyEx(15)
        if key == -1:
            continue
        if key in LEFT_ARROW_KEYS:
            seek(-SKIP_SECONDS)
        elif key in RIGHT_ARROW_KEYS:
            seek(SKIP_SECONDS)
        elif key in KEY_MAP:
            current_colour = KEY_MAP[key]
            set_trackbars(current_colour, colours)
            last_click_hsv = None
            print(f"[COLOUR] {current_colour}")
        elif key == 9:
            active_channel_index = (active_channel_index + 1) % len(NUDGE_CHANNELS)
        elif key == ord("-"):
            name = NUDGE_CHANNELS[active_channel_index]
            cv2.setTrackbarPos(name, WIN, max(0, cv2.getTrackbarPos(name, WIN) - 1))
        elif key == ord("="):
            name = NUDGE_CHANNELS[active_channel_index]
            cv2.setTrackbarPos(name, WIN, min(NUDGE_MAX[name], cv2.getTrackbarPos(name, WIN) + 1))
        elif key == ord("["):
            name = NUDGE_CHANNELS[active_channel_index]
            cv2.setTrackbarPos(name, WIN, max(0, cv2.getTrackbarPos(name, WIN) - 5))
        elif key == ord("]"):
            name = NUDGE_CHANNELS[active_channel_index]
            cv2.setTrackbarPos(name, WIN, min(NUDGE_MAX[name], cv2.getTrackbarPos(name, WIN) + 5))
        elif key in {ord(" "), ord("p"), ord("P")}:
            is_playing = not is_playing
        elif key in {ord("a"), ord("A")}:
            show_overview = not show_overview
        elif key in {ord("s"), ord("S")}:
            read_trackbars(current_colour, colours)
            write_to_pipeline(current_colour, colours[current_colour]["lower"], colours[current_colour]["upper"])
        elif key in {ord("q"), ord("Q"), 27}:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
