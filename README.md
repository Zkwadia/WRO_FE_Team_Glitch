# WRO_FE_Team_Glitch_2026 
## Table of Contents:
## Our Team:
We are Team Glitch, a 3 member team from Mumbai, India. We are a passionate group of 8th grade students who have prior experience in both FIRST Tech Challenge and WRO. We combine programming, engineering, and problem-solving skills and are mentored by The Innovation Story.

#### Our Members:  
- **Rehaan Dhandhia**: Programming, Electronics
- **Zeus Wadia**: Electronics, Mechanical, Programming
- **Shaurya Sule**: Electronics, Programming, Mechanical

## Electronic Systems

### Electronic Parts Used:
|Part Name|Image|Use in Our Robot|Quantity|Details|
|-|-|-|-|-|
|****Microcontrollers****|
|Raspberry Pi 4B|<img src="Bot-Photos/Parts Used/Raspberry Pi 4B.png" alt="Banner" height="150">|Primary processer - processes camera and LiDAR and controls steering servo and the drive motor|1|[Datasheet](https://pip-assets.raspberrypi.com/categories/545-raspberry-pi-4-model-b/documents/RP-008341-DS-1-raspberry-pi-4-datasheet.pdf?disposition=inline)
|Arduino Mega 2560|<img src="Bot-Photos/Parts Used/Arduino Mega 2560.png" alt="Banner" height="150">|Secondary microcontroller - processes IMU and encoder data and transmits it to the RasPi|1|[Datasheet](https://docs.arduino.cc/resources/datasheets/A000067-datasheet.pdf)
|****Sensors****|
|HikVision USB Webcam|<img src="Bot-Photos/Parts Used/HIKVISION Webcam.png" alt="Banner" height="150">|Camera that senses the game elements' position and colour to avoid them and choose the correct path|1|[Datasheet](https://assets.hikvision.com/prd/public/all/doc/m000043043/DS-U02_Datasheet_20240617.pdf)
|TFMini Plus LiDAR|<img src="Bot-Photos/Parts Used/TF Mini Plus LiDAR.png" alt="Banner" height="150">|Detects distance from sides of the robot to the field walls|2|[Datasheet](https://cdn.sparkfun.com/assets/2/b/0/3/8/TFmini_Plus-01-A02-Datasheet_EN.pdf)
|RPLiDAR C1|<img src="Bot-Photos/Parts Used/RPLiDAR C1.png" alt="Banner" height="150">|360° LiDAR mounted at front of the robot to detect distance from traffic signs|1|[Datasheet](https://d229kd5ey79jzj.cloudfront.net/3157/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf)
|BNO085x IMU|<img src="Bot-Photos/Parts Used/BNO085x IMU.png" alt="Banner" height="150">|9DOF IMU (XYZ, YPR) used for localization to find the robot's position on the field|1|[Datasheet](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-9-dof-orientation-imu-fusion-breakout-bno085.pdf)
|****Power****|
|11.1V Li-ion Battery|<img src="Bot-Photos/Parts Used/11.1V Li-ion Battery.png" alt="Banner" height="150">|Rechargable battery that powers the whole robot|1|[Datasheet](https://quartzcomponents.com/products/high-fly-11-1v-1000mah-3s-30c-lithium-polymer-rechargeable-battery?srsltid=AfmBOooGkjnd4aVrqf4FNReuglmn8xht3cvrOTYpgqrLoAxjA5Q6hU98)|
|DC-DC Buck Converter|<img src="Bot-Photos/Parts Used/DC DC Buck Converter.png" alt="Banner" height="150">|Reduces the 12V power to 5V for the Arduino, Raspberry Pi, and encoder|1|[Datasheet](https://somanytech.com/ic-lm2596-dc-to-dc-buck-converter-module-datasheet-schematic/)|
|MDD10A Motor Driver|<img src="Bot-Photos/Parts Used/MDD10A Motor Driver.png" alt="Banner" height="150">|Single channel motor driver to power and control the drive motors|1|[Datasheet](https://makermotor.com/content/cytron/pn00218-cyt4/MDD10A%20User%27s%20Manual.pdf?srsltid=AfmBOooV62io2gMUfWQ314DJQqHSyYWEV69z_PfxwK8i50yYT-ExlGiV)
|Arduino Mega Shield|<img src="Bot-Photos/Parts Used/Arduino Mega Shield.png" alt="Banner" height="150">|Circuit board attached to Arduino to fasten wires using screws instead of just plugging in|1| 
|****Actuators****|
|35KG Servo|<img src="Bot-Photos/Parts Used/35kg Servo.png" alt="Banner" height="150">|Controls angle of the front wheels to steer the robot|1|[Datasheet](https://hajim.rochester.edu/me/sites/kelley/me240/DS3235-270_datasheet.pdf)
|Johnsons Quad 600RPM DC Motor + Encoder|<img src="Bot-Photos/Parts Used/Johnson's Quad DC Motor.jpeg" alt="Banner" height="150">|Controls forward and backward movement of the robot. Uses the encoder for localisation|1|[Datasheet](https://download.robokits.co.in/downloads/RMCS-3072.pdf)

## Hardware Systems

### Drivetrain
  #### Drive
  We used a Johnsons Quad 600RPM DC Motor connected to the rear wheels to move the bot forward. The motor is connected to the rear wheels through a differential gearbox. This allows both wheels to move at different speeds, which is required while turning or if one rear wheel has more traction that the other. This system works by connecting a pinion gear the the motor directly moves to a large, ring gear. Mounted on the ring gear, is a spider gear. This spider gear is connected to both side gears, which are connected directly to both rear wheels. The spider gear is critical to allowing both rear wheels to move at different speeds. This works as when wheel needs to rotate more than another, it has more load, putting more tension from that side on the gearbox. This makes the spider gear rotate in the other direction, reducing the speed of the other wheel, while allowing that wheel to increase its speed. This allows our robot to turn. For example, while turning right, the wheel on the right will move slower than normal, while the left wheel will move faster while keeping motor speed constant.
  
  #### Steering

### Chassis


## Software Architecture

### Challenge-Specific Logic:
#### Open Challenge
In the Open Challenge, we use our 2 TFMini+ LiDAR sensors at the side of the robot to detect the distance from the inner field perimeter. These sensors detect the distance to the nearest object that is in front of them, which in our case is the walls (inner and outer) of the game field. When either of the sensors detects a large jump in the distance to the nearest wall, they recognise that a turn (clockwise or anticlockwise) is required. The servo then turns, until the bot completes a 90 degree turn. After this, the robot continues forward again, until that same sensor detects a large jump in distance. There, the bot turns again. After the robot completes 12 turns (3 rounds), it identifies that it has finished the course. 
We also use encoder data to track how much distance the robot has travelled, and the RPLiDAR for even more front and side obstacle information to get more reliable navigation. We also use multiprocessing is used so that we can run sensor reading, steering correction, and LiDAR processing simultaneously.

#### Obstacle Challenge

### Localization

#### Iteration 1: Encoder + Servo Angle + IMU
We used the encoder on the motor to aid in the localization of our robot. To calculate the amount of distance our robot has travelled, we checked the amount of encoder counts completed and used a basic formula to find the distance:
```
dist = (wheelCirc / 1560) * encPosition
```

ADD TRIGONOMETRY

#### Iteration 2: IMU

### Camera Integration

### LiDAR
