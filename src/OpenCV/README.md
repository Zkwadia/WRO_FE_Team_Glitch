# OpenCV Code
## Overview
This folder contains the OpenCV-heavy code related to calibration. These are scripts we use to calibrate the robot to the HSV values of game elements at different venues.

## Codes
- `HSVCalibrateNationals.py`: Our original calibration script that we used at nationals. You should place all colours - red/green blocks, both corner lines, pink walls, and black walls - within the camera frame. Then, type in a number (1-6) to choose which colour to calibrate, and click on a specific area. The sliders at the bottom will automatically adjust to that colour. Finally, tune the sliders to the perfect colour, and click 's' to save.
- `record.py`: The first part of our new calibration script. It records a video from the camera to be used to calibrate. You should take the robot around the full field while recording (press 'r' to start), making sure to show it all colours.
- `HSVCalibrateNew.py`: The main part of our new calibration script. It functions similarly to the original code, but this plays back the recorded video and you can adjust the sliders to calibrate. After calibrating for a few frames of the video, you should play the full video and make sure all colours are correctly detected. Finally, click 's' to save it to the vision pipeline.

There is also a more detailed explanation within each code on what it does and how to use it.