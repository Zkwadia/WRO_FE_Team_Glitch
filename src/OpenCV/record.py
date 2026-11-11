import os
os.system("sudo pkill pigpiod")
os.system("sudo pigpiod -x -1")
import cv2
from datetime import datetime
from vision_pipeline_new import VisionPipeline, USE_LAB, FRAME_WIDTH, FRAME_HEIGHT
import threading

import pigpio

OUTPUT_DIR = "/home/pi/wro_logs/videos" #change this based on your file path, or use ours
os.makedirs(OUTPUT_DIR, exist_ok=True)
SHOW_WINDOW = True

# Recording FPS
RECORD_FPS = 30.0

# JPEG / video quality
FOURCC = cv2.VideoWriter_fourcc(*"mp4v")

pwm = pigpio.pi()

print("Starting camera...")

yellow_led = 19
red_led = 13
green_led = 26

pipeline = VisionPipeline(USE_LAB)

cap = pipeline.open_camera()

if cap is None:
    print("ERROR: Could not open camera.")
    exit(1)

for pin in [yellow_led, red_led, green_led]:
	pwm.set_mode(pin, pigpio.OUTPUT)
	pwm.write(pin, 0)

print("Camera opened successfully.")

WIN = "Recorder (R=Rec,S=Stop,Q=Quit)"
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, FRAME_WIDTH, FRAME_HEIGHT)

recording = False
video_writer = None
video_path = None
frame_count = 0

try:
    while True:

        ret, frame = cap.read()

        if not ret or frame is None:
            print("Camera frame read failed.")
            continue

        frame = cv2.resize(
            frame,
            (FRAME_WIDTH, FRAME_HEIGHT)
        )

        # ----------------------------------------------------
        # Display
        # ----------------------------------------------------

        display_frame = frame.copy()

        if recording:
            '''cv2.putText(
                display_frame,
                "RECORDING",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )
            cv2.putText(
                display_frame,
                f"Frames: {frame_count}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )'''
            pwm.write(red_led, 0)
            pwm.write(yellow_led, 0)
            pwm.write(green_led, 1)
        else:
            '''cv2.putText(
                display_frame,
                "Press R to start recording",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )'''
            pwm.write(red_led, 1)
            pwm.write(yellow_led, 0)
            pwm.write(green_led, 0)	
        cv2.imshow(WIN, display_frame)

        if recording and video_writer is not None:
            video_writer.write(frame)
            frame_count += 1

        key = cv2.waitKey(1) & 0xFF

        if key == ord("r"):
            if not recording:
                timestamp = datetime.now().strftime(
                    "%Y-%m-%d_%H-%M-%S"
                )
                video_path = os.path.join(
                    OUTPUT_DIR,
                    f"wro_recording_{timestamp}.mp4"
                )
                video_writer = cv2.VideoWriter(
                    video_path,
                    FOURCC,
                    RECORD_FPS,
                    (FRAME_WIDTH, FRAME_HEIGHT)
                )
                if not video_writer.isOpened():
                    print("ERROR: Could not create video file.")
                    video_writer = None
                else:
                    recording = True
                    frame_count = 0
                    print()
                    print("======================================")
                    print("RECORDING STARTED")
                    print(f"Saving to: {video_path}")
                    print("Press S to stop.")
                    print("======================================")
                    print()

        elif key == ord("s"):
            if recording:
                recording = False
                if video_writer is not None:
                    video_writer.release()
                    video_writer = None

                print()
                print("======================================")
                print("RECORDING STOPPED")
                print(f"Saved to: {video_path}")
                print(f"Frames recorded: {frame_count}")
                print("======================================")
                print()

                video_path = None
                frame_count = 0

        elif key == ord("q"):
            print("Quitting...")
            break

except KeyboardInterrupt:

    print("Keyboard interrupt received.")


finally:

    if video_writer is not None:
        video_writer.release()

        print("Video writer closed.")

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    print("Camera closed.")
