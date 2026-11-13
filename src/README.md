# /src Code Folder
## Overview
This folder contains all the codes and scripts required to run our robot. This includes calibration, obstacle, open, vision pipelines, and test codes.

## Codes
- `OBSTACLE_NATIONALS_FINAL.py`: Our original obstacle challenge code that we used at nationals. It contains 3 processes - drive, camera, and IMU/encoder (from ESP32). It is explained throughout the main README and there is also an explanation on what it does and how to use it on the code itself. Before running, make sure that the robot is completely within the parking zone and close to the back wall. Once you run the script (through either terminal or service), wait until the servo turns left and right (to confirm everything is working) before turning on the start switch. 
- `OPEN_NATIONALS_FINAL.py`: Our original open challenge code that we used at nationals. It contains 3 processes - drive, camera, and IMU/encoder (from ESP32). It is explained throughout the main README and there is also an explanation on what it does and how to use it on the code itself. Before running, make sure the robot's heading is as close to 0 as possible and wait for the servo to turn left/right before you start.
- `obstacle_nov.py`: Our new obstacle challenge code that we want to use at nationals. It contains 3 processes - drive, camera, and IMU/encoder (from ESP32). This code was made after we change dthe gear ratio, causing a huge increaese in speed. The obstacle avoidance and parking sequences have been changed, and we've removed speed PID completely. It is explained throughout the main README and there is also an explanation on what it does and how to use it on the code itself. Before running, make sure that the robot is completely within the parking zone and close to the back wall. Once you run the script (through either terminal or service), wait until the servo turns left and right (to confirm everything is working) before turning on the start switch. 
- `vision_pipeline_new.py` and `vision_pipeline_3.py`: Our vision pipeline codes, for obstacle and open challenge respectively. They process the camera data and format it to send to the main code. There is a full explanation on what it does within the code.
- `gains.json`: The file with all the PID/PD gains for the open and obstacle codes. You can edit it and it updates to the robot immediately. Both open and obstacle pull their values from this file.
- **Test Scripts** (BNO085x, Servo, Encoder, etc): All test scripts to check whether components are working.

## Subfolders
- `OpenCV`: Calibration scripts (there is a README within the folder with more detail)
- `ESP32`: Code for the ESP32's reading of IMU & encoder, and communication with the RPi 

There is also a more detailed explanation within each code on what it does and how to use it.