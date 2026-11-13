# MATLAB - HSV Video Calibrator

## Overview

This is our MATLAB-based HSV colour calibration tool, designed completely by us. It allows colour detection thresholds to be tuned using recorded camera footage and exported for use in the robot's Python OpenCV vision pipeline.

## Features

- Import and play recorded video footage.
- Navigate frames using playback controls and a timeline.
- Click on a colour to automatically get the HSV low/high values for a 5 x 5 pixel region.
- Calibrate all 6 colours used - red, green, blue, orange, black, pink
- Fine-tune HSV thresholds using sliders (or arrow keys)
- Export calibrated thresholds as a Python `HSV_RANGES` dictionary.

## Requirements

- MATLAB R2020b or later.
- Image Processing Toolbox.
- Recorded video footage (MP4, AVI, MOV and other supported formats).

## Running the Application

1. Open MATLAB and navigate to the project directory.
2. Run:

```matlab
app = HSVVideoCalibratorV1();
```

3. Click `Import Video` and select the desired footage.

## Calibration Workflow

1. **Import:** Load recorded footage from the robot's camera.
2. **Select:** Choose one of the six colours.
3. **Sample:** Click the desired colour in the video to generate initial HSV thresholds.
4. **Adjust:** Fine-tune the six HSV parameters:
5. **Inspect:** View all enabled colour detections or an individual binary mask.
6. **Export:** Save the calibration or export it as a Python file.

## Exporting to Python

Click **Export Python HSV_RANGES** to generate `hsv_ranges.py`.

The exported file contains:
- Lower and upper HSV thresholds for each colour.
- Separate ranges for wrapped Hue intervals.
- Minimum region pixel area.
- Mask-cleaning configuration.
- Enabled colour list.
