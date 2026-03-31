
# Overview
This is the repository of the GWUAV team of the GWU 2025 RTX Autonomous Compeition. We implemented a dual-drone solution, using a scout drone and package delivery drone. Our UAV code enable both drones to fly autonomously.

- Comms : Communications subteam, responsible for the transer of data between both drones. As well as the logging of data from drones to a laptop. It was vital that in order to maintain autonomy, the base station laptop did not transmit any information to the drones, and only prompted them to start, or recieve logs.
- Comp Scripts : Scripts used for the competition challenges, including calibrations for onboard cameras, and test scripts for pre-flight checks. Challenge 1 uses only the scout drone, while challenge 2 incorperates both a scout and package delivery (PD) drone.
- Visual Processing : THe visual processing system is more than just computer vision. It takes raw camera input and detects and identifies ArUco markers. It will then extract pose informaton from the correct marker and communicate it to the Navigation system
- Gazebo Simulaton : Simulations are an important part of testing the navigation scripts and visualizing mision plans. The simulator that was primarily used was Gazebo with models for ArUco markers. The Sim Scripts sub-directory shows different flight scripts that emulate real navigation in the real world. 

## Navigation Subteam
The Navigation subteam was an integral part of what allowed the drone to fly autonomously, but there is no dedicated directory. This is because many of the scripts that showcase navigation are more meaningfully represented in either competition or simulation scripts. For example, all Challenge1 and Challenge2 scripts in Comp Scripts were primarily developed and coded by the Navigation team. 

## Hardware
The hardware for the onboard computers for initial tests and prototyes was an NVIDIA Jetson Nano but was later swapped to a Raspberry Pi 5 for compatibility with other hardware systems.

## ArUco Markers
An ArUco is a system of ID represented by a black and white square. Similar to a QR code, it is comprised of a grid of smaller black and white squares arranged in an array and a one block black border around the marker. One configuration of black and white squares equates to a unique numerical ID. 

Most defauilt marker IDs used in this project are 6x6 format.

![An ArUco marker with ID 17](/Gazebo%20Simulation/models/ArUco_17/materials/textures/marker-17.png)

---

# Comp Scripts - Competition Scripts 
## C1 - Challenge 1:
Navigate the scout drone around a designated area. When correct ArUco marker is found, return to starting position. 

Navigation for the scout drone uses a Lissajous based path. This was determined to be faster than more simple searches like lawn-mower search. Example can be found in /Sim Scripts/lissajous.py


## C2 - Challenge 2:
Using a two drone method, a scout drone and delivery drone. Use the scout drone to identify the world coordinates of the target marker. Then communicate the location to the delivery drone that will drop a package at the marker. 

Communication is done using telemetry radio and a TCP like handshake. Examples can be found in /testcom

---

# VisProc - Visual Processing
The visual processing system is responsible for the computer vision detection of the landing zone markers and translation of raw sensor data to world coordinates that is used by the Navigation subsystem to direct the drone. Accurate pose data requires exceptional camera calibration. The cameras for our drones were calibrated using classic checkerboard pattern. ChArUco Board calibration was experimented with but was not as successful as normal checkerboard due to the physical quality difference of the two boards.

Machine Learning approaches to object detection were implemented on prototype builds, but not used on the final drone due to hardware limitations of the onboard computer.

## Imports
- pip install opencv-contrib-python / time / numpy / tkinter / PIL / json
- pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117

## Calibration - Calibrate Camera
This script calibrates a camera and is essential to remove distortion around camera edges and to calculate POSE and distance estimation. Calibration board and camera parameters must be adjusted based on individual hardware.

It uses a checkerboard for calibration by default, a ChArUco board can be used by commenting out checkerboard_read() and checkerboard_calibrate() and uncommenting charuco_read() and charuco_calibrate()

To calibrate a camera:
1. images.py 
2. calibration.py

Result is a calibration npz file that can be tested by aruco.py or moved to different directories to be used in any script that takes in camera output with a calibration folder

## Mapit
Mapit is takes the visual sensor pose information to center the drone above the marker. This increaes the percision of the drone when landing by making small adjustments as it descends


## ArUco Machine Learning
Aids the visual processing system using object detetion to improve performance and reliability in busy sensor information

### Data set
Here is the link to install the data set. use the YOLO8 configuration when downloading
[https://universe.roboflow.com/mqdrones/mq-aruco-2/dataset/3/images?split=valid]

### Train YOLO model

1. Run this command to train.

```
yolo detect train \
  data=dataset/data_config.yaml \
  model=yolov8n.pt \
  epochs=50 \
  imgsz=640 \
  name=my_experiment

```

2. strictly testing model detection with webcam

change <code>conf</code> to desired value.

```
yolo detect predict \
    model=runs/detect/my_experiment/weights/best.pt \
    source=0 \
    show=True \
    conf=0.5

```

---

# Simulation Scripts
<p>Navigation simulations with SITL, MAVProxy, Gazebo, MAVLINK, DroneKit, and pymavlink.</p>

## Shell script simstart.sh
<p> Running, just the shell script in the terminal should start up SITL with MAVProxy, Gazebo virtual world, and the autonomous mission scripts. You can specify different scripts from the Auto Scipts folder. </p>

## How to use capstone.world
<p>Capstone.world is a Gazebo simulation environment. It is supposed to emulate the real settings of the drone flight. You can make this as specific and realistic as you want by defining the gravity, time, wind conditions, etc. Gravity is default 9.8 m/s^2,
but think about simulations on the moon. Specifically for capstone.world the time is defined as early morning with clouds and natural lighting. Morning because the competition starts then and clouds because of a typical cloudy day at Northern Virginia. <br>

The .world file uses XML and SDF standards. Models were used to include drones and ArUco markers. <br> 

Also, the ArUco markers positions can be changed by editing the *<pose> </pose>* first two points which are x,y coordinates. This will give random locations for later tests with the drone. <br>

Now, other simulations and programs can be written to make the drone model fly within this .world (think of .launch scripts). Run this command in one terminal:</p>
```
gazebo --verbose ~/ardupilot_gazebo/worlds/capstone.world
```
<p>In the other terminal, you will run the .launch script.</p>

---

# Related Links
### GWU Article Overview
- ```https://cs.engineering.gwu.edu/multidisciplinary-team-wins-most-innovative-design-award-rtx-drone-competition```
- ```https://engineering.gwu.edu/rtx-uav-raytheon-autonomous-vehicle-competition```

### RTX Video of Compeition Day
- ```https://www.youtube.com/watch?v=TaiLjWskhFQ```

