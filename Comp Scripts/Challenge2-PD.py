########DEPENDENCIES###############
import json 
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import argparse
import logging
import serial
import os
from os import path, sys
import RPi.GPIO as GPIO

########CONSTANTS##################
PACKAGE_ALT = 1 # in meters
DROP_ALT = .3
 
############# SET UP LOGGING #############

# Ensure logs directory exists in the current working directory
log_dir = os.path.join(os.getcwd(), "Logs")
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
 
 # Generate a unique log file name based on the current timestamp
log_filename = os.path.join(log_dir, "mission_{}_P.log".format(time.strftime("%Y%m%d_%H%M%S")))
logging.basicConfig(
    filename=log_filename,
    level=logging.INFO,
    filemode='w',
    format='%(asctime)s - %(levelname)s - %(message)s'
)

########CONNECTION BETWEEN FLIGHT COMPUTER AND CONTROLLER#################
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
 
connection_string = args.connect
 
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
delivery_drone = connect(connection_string, wait_ready=True)
 
#######CONNECTION BETWEEN SCOUT AND DELIVERY DRONE#########
port = '/dev/ttyUSB0'
ser = serial.Serial(port=port, baudrate=57600, timeout=1)

########FUNCTIONS##################
# Package Delivery functions
GPIO.setmode(GPIO.BOARD)
GPIO.setup([8,10], GPIO.OUT)
 
GPIO.output(8, GPIO.LOW)
GPIO.output(10, GPIO.LOW)
 
def latch_open():
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.LOW)
 
def latch_close():
    GPIO.output(8, GPIO.HIGH)
    GPIO.output(10, GPIO.LOW)
    time.sleep(1)
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.LOW)

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not delivery_drone.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    delivery_drone.mode = VehicleMode("GUIDED")
    delivery_drone.armed = True
    while not delivery_drone.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    delivery_drone.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", delivery_drone.location.global_relative_frame.alt)
        if delivery_drone.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def main():

    #Order of messages
    #Recieve lat long       msg = "{:.7f}, {:.7f}".format(target_location.lat, target_location.lon)
    #Write ack              ack == msg
    #Recieve Mission go     mission_go_msg = "1"

    while True:

        logging.info("UXV Mission Start")

        if ser.in_waiting > 0:                                                                              #Message recieved, recieve lat long
            marker_latLong = ser.readline().decode('utf-8').strip()
            
            logging.info(f"Received coordinates: {marker_latLong}")
                        
                        
            marker_lat , marker_long = marker_latLong.split(',')                                            #Tokenize by comma
            marker_lat = marker_lat.strip()                                                                 #Strip of white space
            marker_long = marker_long.strip()

            #only print out bad location every few seconds
            if not marker_lat or not marker_long:                                                           #Marker lat or long is Null
                print(f"Bad coordinates Lat : {marker_lat} , Long : {marker_long}")
                
                logging.warning(f"Invalid coordinates received: Lat={marker_lat}, Long={marker_long}")
                #bad_marker = "bad marker"                                                                  #Let the scout time itself out, future implementation should have the scout recieve a bad marker
                #ser.write(bad_marker.encode('utf-8'))
            else:
                logging.info(f"Marker location: Lat={marker_lat}, Long={marker_long}")
                print(f"Marker coordinates Lat : {marker_lat} , Long : {marker_long}")
                ack = "{:.7f}, {:.7f}".format(marker_lat, marker_long)
                ser.write(ack.encode('utf-8'))
                logging.info(f"Sent message: {ack}")
                print(f"Sent message: {ack}")
                
                timeout = time.time() + 10  # Wait up to 10 seconds
                mission_go = False
                while time.time() < timeout:
                    if ser.in_waiting > 0:                                                                  #Message recieved, wait on mission go
                        msg = ser.readline().decode('utf-8').strip()
                        if msg == "1":
                           mission_go = True 
                           logging.info(f"Received message: {msg}")
                           
                           break
                        else:
                            print(f"Unexpected Message expected mission go , recieved {msg}")               #In case message recieved out of order  
                            logging.warning(f"Unexpected message: {msg}")
                     
                    time.sleep(0.1)
                
                if mission_go == False:
                    print(f"Mission go not recieved before timeout {timeout}")
                    logging.error("Mission go not received before timeout")

                else:
                    print(f"Mission go recieved")
                    logging.info("Mission go received")

                    arm_and_takeoff(PACKAGE_ALT)
                    logging.info("Drone armed and took off")
                    
                    #Add precision landing if needed
                    #From one meter , go to marker location
                    target_location = LocationGlobalRelative(marker_lat, marker_long, PACKAGE_ALT)
                    delivery_drone.simple_goto(target_location)                                                    #Non-blocking, must manually wait
                    #Block until target and drone believes it is one meter from target
                    while get_distance_metres(target_location, delivery_drone.location.global_relative_frame) > 1:         
                        time.sleep(0.1)
                    
                    print("Reached marker location , begin descend")
                    logging.info("Reached marker location, begin drop-off")
                    
                    #Descend down to .3 meter
                    drop_location = LocationGlobalRelative(marker_lat, marker_long, DROP_ALT)    #Assume alt is 1 meter
                    delivery_drone.simple_goto(drop_location)                                                       #Have the drone descend to drop package 
                    while delivery_drone.location.global_relative_frame.alt > DROP_ALT * 0.95:                                 #Block until within 0.3 meter
                        time.sleep(0.1)

                    print("Executing package drop")
                    logging.info("Executing package drop")                                                                        
                    latch_open()
                    time.sleep(1)
                    latch_close()
                    logging.info("Latch opened")
                  
                    delivery_drone.mode = VehicleMode("LAND")
               
                 
        logging.info("UXV Mission Complete Time: %s", time.ctime(time.time()))
        GPIO.cleanup()
        delivery_drone.close()

if __name__ == "__main__":
    main()
