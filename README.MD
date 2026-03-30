
# Overview
This is the repository of the GWUAV team of the GWU 2025 RTX Autonomous Compeition.

The code is divided into:
- Comms : Communications subteam, responsible for the transer of data between both drones. As well as the logging of data from drones to a laptop. It was vital that in order to maintain autonomy, the base station laptop did not transmit any information to the drones, and only prompted them to start, or recieve logs.


## ArUco Markers
An ArUco is a system of ID represented by a black and white square. Similar to a QR code, it is comprised of a grid of smaller black and white squares arranged in an array and a one block black border around the marker. One configuration of black and white squares equates to a unique numerical ID. 

Most defauilt marker IDs used in this project are ID 0-5 and 6x6 format.

---

# Comp Scripts - Competition Scripts 
## C1 - Challenge 1:
Navigate the scout drone around a designated area. When correct ArUco marker is found, return to starting position. 

Navigation for the scout drone uses a Lissajous based path. This was determined to be faster than more simple searches like lawn-mower search. Example can be found in /Sim Scripts/lissajous.py


## C2 - Challenge 2:
Using a two drone method, a scout drone and delivery drone. Use the scout drone to identify the world coordinates of the target marker. Then communicate the location to the delivery drone that will drop a package at the marker. 

Communication is done using telemetry radio and a TCP like handshake. Examples can be found in /testcom

# VisProc - Visual Processing
Responsible for the computer vision detection of the landing zone markers.

### Calibration - Calibrate Camera
This script calibrates a camera and is essential to remove distortion around camera edges and to calculate POSE and distance estimation

/t It uses a checkerboard for calibration by default, a ChArUco board can be used by commenting out checkerboard_read() and checkerboard_calibrate() and uncommenting charuco_read() and charuco_calibrate()

# Imports
- pip install opencv-contrib-python / time / numpy / tkinter / PIL / json
- pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117


/t To calibrate a camera:
/t /t 1. images.py
/t /t 2. calibration.py

Result is a calibration npz file that can be tested by aruco.py

### ArUco Machine Learning
/t Aids the visual processing system using object detetion to improve performance and reliability in busy sensor information

# Data set
Here is the link to install the data set. use the YOLO8 configuration when downloading
[https://universe.roboflow.com/mqdrones/mq-aruco-2/dataset/3/images?split=valid]

# Train YOLO model

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


#### Mapit
/t  Mapit is takes the visual sensor pose information to center the drone above the marker. This increaes the percision of the drone when landing by making small adjustments as it descends. 