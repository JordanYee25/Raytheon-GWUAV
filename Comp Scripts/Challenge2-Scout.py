"""
Search Mission with pose estimation of aruco marker for CHALLENGE 1

This script performs a search pattern mission. While the drone is
following the search waypoints, it monitors for an ArUco marker. Once a marker
is detected, the vehicle sends lattitude and longitude coordinates of the marker and lands.
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import numpy as np
import math
import argparse
import os
from os import path, sys
from aruco_library import ArucoSingleTracker, load_calibration_data
import logging
import serial

############# SET UP LOGGING #############

# Ensure logs directory exists in the current working directory
log_dir = os.path.join(os.getcwd(), "Logs")
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
    
# Generate a unique log file name based on the current timestamp
log_filename = os.path.join(log_dir, "mission_{}.log".format(time.strftime("%Y%m%d_%H%M%S")))
logging.basicConfig(
    filename=log_filename,
    level=logging.INFO,
    filemode='w',
    format='%(asctime)s - %(levelname)s - %(message)s'
)

############# TELEMETRY RADIO CONNECTION #############
port = '/dev/ttyUSB0'
baudrate=57600
timeout=1

############# CONSTANTS #############
# Search parameters
pi = math.pi
T_STEP = pi/8           # granularity of curve (time step)
T_MAX = 64              # number of time points for the mission
AMP_X = 8               # meters, half-amplitude in X-direction
AMP_Y = 8               # meters, half-amplitude in Y-direction
W_X = 1                 # angular frequency in X-direction
W_Y = math.sqrt(2)      # angular frequency in Y-direction
PHI_X = pi/2            # phase shift in X-direction
PHI_Y = 0               # phase shift in Y-direction
SCOUT_ALT = 4           # scouting altitude (meters)
MAX_AIRSPEED = 1
CENTER_LAT = 'CHANGEME'
CENTER_LONG = -'CHANGEME' 
EARTH_RADIUS = 6378.137   # km


############# CONNECTION #############
parser = argparse.ArgumentParser(description='Scout Challenge 1 Mission')
parser.add_argument('--connect', help="Vehicle connection target string.", default='')
args = parser.parse_args()
connection_string = args.connect

print("Connecting to vehicle on: %s" % connection_string)
scout = connect(connection_string, wait_ready=True)

############# LOAD CALIBRATION & INIT ARUCO TRACKER #############
calibration_filepath = 'calibration_minicam.npz'
camera_matrix, dist_coeffs = load_calibration_data(calibration_filepath)
aruco_tracker = ArucoSingleTracker(
    id_to_find=1,       # marker id to detect
    marker_size=0.254,      # marker size in m
    show_video=False,
    camera_matrix=camera_matrix,
    camera_distortion=dist_coeffs
)

############# HELPER FUNCTIONS #############
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # meters
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return newlat, newlon

def camera_to_uav(x_cam, y_cam):
    x_uav = -y_cam
    y_uav = x_cam
    return x_uav, y_uav

def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east  = x_uav * s + y_uav * c
    return north, east

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not scout.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    scout.mode = VehicleMode("GUIDED")
    scout.armed = True
    while not scout.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    scout.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", scout.location.global_relative_frame.alt)
        if scout.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def search():
    print("Uploading search pattern")
    cmds = scout.commands
    cmds.download()    # Download existing commands
    cmds.wait_ready()
    print("Clearing existing commands")
    cmds.clear()
    rows = 2 * AMP_Y + 1
    columns = 2 * AMP_X + 1
    met_to_deg = (1 / ((pi / 180) * EARTH_RADIUS)) / 1000  # conversion constant
    latitude_arr = np.zeros((rows, columns), dtype=np.float64)
    longitude_arr = np.zeros((rows, columns), dtype=np.float64)
    for row in range(rows):
        for col in range(columns):
            latitude_arr[row][col] = CENTER_LAT + (AMP_Y - row) * met_to_deg
            longitude_arr[row][col] = CENTER_LONG + ((AMP_X - col) * met_to_deg) / (math.cos(CENTER_LAT * (pi/180)))
    # Add MAV_CMD_NAV_TAKEOFF command (ignored if already airborne)
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, SCOUT_ALT))
    # Add waypoints
    for T_RANGE in range(0, T_MAX):
        X_point = math.ceil(AMP_X * math.sin(W_X * T_STEP * T_RANGE + PHI_X))
        Y_point = math.ceil(AMP_Y * math.sin(W_Y * T_STEP * T_RANGE + PHI_Y))
        lat = latitude_arr[-Y_point + AMP_Y][X_point + AMP_X]
        lon = longitude_arr[-Y_point + AMP_Y][X_point + AMP_X]
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                         lat, lon, SCOUT_ALT))
    print("Search waypoints uploaded.")
    cmds.upload()
    
############# MAIN METHOD #############
def main():
    # 1) upload mission & log start
    search()  
    mission_start_time = time.time()
    logging.info("UAV Start Time: %s", time.ctime(mission_start_time))
    
    # 2) arm and takeoff to scout altitude
    arm_and_takeoff(SCOUT_ALT)
    scout.airspeed = MAX_AIRSPEED
    print("Starting search mission")
    
    # 3) enter AUTO mode and reset waypoint counter
    scout.commands.next = 0
    scout.mode = VehicleMode("AUTO")
    while scout.mode.name != "AUTO":
        time.sleep(0.2)
    
    # 4) open serial port
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    
    # 5) main search loop
    aruco_discovery_logged = False

    while True:
        next_wp = scout.commands.next
        marker_found, x_cam, y_cam, _ = aruco_tracker.track(loop=False)

        # only act on the very first sighting of your target‐ID
        if marker_found and not aruco_discovery_logged:
            aruco_discovery_logged = True   #mark it done

            logging.info("Marker Discovery Time: %s", time.ctime(time.time()))
            # switch to GUIDED
            scout.mode = VehicleMode("GUIDED")
            while scout.mode.name != "GUIDED":
                time.sleep(0.2)
            logging.info("Switched to GUIDED mode, paused following waypoints")
            
            logging.info("Initiating Coordinate Calculation...")
            # compute offset -> geo coords
            x_uav, y_uav = camera_to_uav(x_cam, y_cam)
            uav_loc = scout.location.global_relative_frame
            north, east = uav_to_ne(x_uav, y_uav, scout.attitude.yaw)
            lat, lon = get_location_metres(uav_loc, north, east)
            logging.info("Marker Location: Lat = %.7f, Lon = %.7f", lat, lon)
            print("Marker Location: Lat = %.7f, Lon = %.7f", lat, lon)

            # send once
            msg = f"{lat:.7f}, {lon:.7f}"
            try:
                ser.write(msg.encode('utf-8'))
                logging.info("sent marker coords to ground station: %s", msg)
                print("sent marker coords to ground station: %s", msg)
            except Exception as e:
                logging.error("Failed to send marker coordinates over serial: %s", str(e))

            # land and exit
            scout.mode = VehicleMode("LAND")
            while scout.mode.name != "LAND":
                time.sleep(0.2)
            logging.info("Mission complete. Mode has been set to LAND")
            

            break  #out of the main loop

        # if flown all waypoints without  detecting
        if next_wp >= T_MAX:
            logging.info("No marker found; landing at waypoint %d", next_wp)
            print('no marker found during mission, setting to mode land')
            scout.mode = VehicleMode("LAND")
            while scout.mode.name != "LAND":
                time.sleep(0.2)
            break

        time.sleep(1)

    # cleanup
    logging.info("Mission complete at %s", time.ctime())
    print("Mission complete at %s", time.ctime())
    aruco_tracker.stop()
    scout.close()
    ser.close()

if __name__ == "__main__":
    main()
