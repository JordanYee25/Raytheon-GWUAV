"""
Scout in Challenge 2:
    Searches for marker
    Does pose estimation on found marker
    Sends pose estimate to PD drone
    RTLs after PD drone arrives to location
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
log_filename = os.path.join(log_dir, "mission_{}_S.log".format(time.strftime("%Y%m%d_%H%M%S")))
logging.basicConfig(
    filename=log_filename,
    level=logging.INFO,
    filemode='w',
    format='%(asctime)s - %(levelname)s - %(message)s'
)

############# CONSTANTS #############
# Search parameters
pi = math.pi
T_STEP = pi/8           # granularity of curve (time step)
T_MAX = 64              # number of time points for the mission
AMP_X = 10              # meters, half-amplitude in X-direction
AMP_Y = 10              # meters, half-amplitude in Y-direction
W_X = 1                 # angular frequency in X-direction
W_Y = 2                 # angular frequency in Y-direction
PHI_X = pi/2            # phase shift in X-direction
PHI_Y = 0               # phase shift in Y-direction
SCOUT_ALT = 7           # scouting altitude (meters)
CENTER_LAT = 38.6778467
CENTER_LONG = -77.2492611 # Default Gazebo/SITL params
EARTH_RADIUS = 6378.137   # km

############# TELEMETRY RADIO CONNECTION #############
port = '/dev/ttyUSB0'

############# CONNECTION #############
parser = argparse.ArgumentParser(description='Scout Challenge 1 Mission')
parser.add_argument('--connect', help="Vehicle connection target string.", default='')
args = parser.parse_args()
connection_string = args.connect

print("Connecting to vehicle on: %s" % connection_string)
scout = connect(connection_string, wait_ready=True)

############# LOAD CALIBRATION & INIT ARUCO TRACKER #############
calibration_filepath = 'calibration_logi.npz'
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
    """
    Returns new latitude and longitude by offsetting the original_location by dNorth and dEast (meters).
    """
    earth_radius = 6378137.0  # meters
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return newlat, newlon

def marker_position_to_angle(x, y, z):
    angle_x = math.atan2(x, z)
    angle_y = math.atan2(y, z)
    return angle_x, angle_y

def camera_to_uav(x_cam, y_cam):
    """
    Convert camera coordinate system to UAV coordinate system.
    """
    x_uav = -y_cam
    y_uav = x_cam
    return x_uav, y_uav

def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east  = x_uav * s + y_uav * c
    return north, east

def check_angle_descend(angle_x, angle_y, angle_desc):
    return math.sqrt(angle_x**2 + angle_y**2) <= angle_desc

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint():
    nextwaypoint = scout.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = scout.commands[nextwaypoint - 1]  # commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    return get_distance_metres(scout.location.global_frame, targetWaypointLocation)

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
    #scout.airspeed = 5 # adjust as needed

############# MISSION FUNCTIONS #############
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

def communicate(target_location, port, max_attempts=3):
    # Format the message to send
    msg = "{:.7f}, {:.7f}".format(target_location.lat, target_location.lon)
    
    # Open the serial port (ensure the appropriate exception handling)
    try:
        ser = serial.Serial(port=port, baudrate=57600, timeout=1)
    except serial.SerialException as e:
        logging.error("Failed to open serial port: %s", e)
        return
    
    attempt = 0
    ack_received = False

    while attempt < max_attempts and not ack_received:
        attempt += 1
        # Send the message to the drone
        ser.write(msg.encode('utf-8'))
        logging.info(
            "COMMS Transmitted the Marker Location: Lat = %.7f, Lon = %.7f (Attempt %d)",
            target_location.lat, target_location.lon, attempt
        )
        
        # Wait for acknowledgment from the second drone
        timeout = time.time() + 5  # Wait up to 5 seconds
        while time.time() < timeout:
            if ser.in_waiting > 0:
                ack = ser.readline().decode('utf-8').strip()
                if ack == msg:
                    ack_received = True
                    logging.info("Received ACK from second drone.")
                    print("Received ACK from second drone.")
                    break
                else:
                    # If an incorrect ACK is received, log a warning and break out to retry
                    logging.warning("Received incorrect ACK: '%s' (expected '%s')", ack, msg)
                    break
            time.sleep(0.1)
        
        if not ack_received:
            logging.warning("ACK not received or incorrect, resending message (Attempt %d)", attempt)
    
    if not ack_received:
        logging.error("Failed to receive correct ACK after %d attempts.", max_attempts)
    
    ser.close()

def main():
    search()  # Upload search pattern mission
    mission_start_time = time.time()

    arm_and_takeoff(SCOUT_ALT)
    logging.info("UAV Start Time: %s", time.ctime(mission_start_time))
    
    print("Starting search mission")
    scout.commands.next = 0
    scout.mode = VehicleMode("AUTO")
    while scout.mode != "AUTO":
        time.sleep(0.2)
        
    # Flag to log the first detection only once
    aruco_discovery_logged = False
    message_sent = False  # Flag to mark that message has been sent
    
    # Monitor mission progress and check for marker detection
    while True:
        next_waypoint = scout.commands.next
        # Check for marker detection in the search area
        marker_found, _, _, _ = aruco_tracker.track(loop=False)
        
        if marker_found:
            if not aruco_discovery_logged:
                logging.info("Marker Discovery Time: %s", time.ctime(time.time()))
                aruco_discovery_logged = True
                
            print("Marker detected! Initiating landing and location transmission.")
        
            print("Switching to GUIDED mode, pause following waypoints")
            scout.mode = VehicleMode("GUIDED")
            while scout.mode != "GUIDED":
                time.sleep(0.2)
                        
            while marker_found:
                marker_found, x_cam, y_cam, z_cam = aruco_tracker.track(loop=False)
                x_uav, y_uav = camera_to_uav(x_cam, y_cam)
                uav_location = scout.location.global_relative_frame
                    
                if uav_location.alt >= 5.0:
                    z_cam = uav_location.alt * 100.0
                    
                angle_x, angle_y = marker_position_to_angle(x_uav, y_uav, z_cam)
                    
                  
                north, east = uav_to_ne(x_uav, y_uav, scout.attitude.yaw)
                marker_lat, marker_lon = get_location_metres(uav_location, north, east)
                            
                target_location = LocationGlobalRelative(marker_lat, marker_lon, SCOUT_ALT)
                scout.simple_goto(target_location)
                    
                logging.info("SCOUT Location: Lat = %.7f, Lon = %.7f", uav_location.lat, uav_location.lon)
                logging.info("MARKER Location: Lat = %.7f, Lon = %.7f", target_location.lat, target_location.lon)
                          
                if communicate(target_location, port):
                    # On successful acknowledgment, send "mission go" message.
                    mission_go_msg = '1'
                    try:
                        ser = serial.Serial(port=port, baudrate=57600, timeout=1)
                        ser.write(mission_go_msg.encode('utf-8'))
                        logging.info("COMMS Transmitted 'mission go' message to pd drone")
                        print("Sent 'mission go' message to pd drone")
                        ser.close()
                    except serial.SerialException as e:
                        logging.error("Failed to open serial port for 'mission go' message: %s", e)

                    message_sent = True  # Mark that the message sequence was successfully sent
                    time.sleep(5)
                    
                    print("Mission accomplished. Returning to launch.")
                    scout.mode = VehicleMode("RTL")
                    while scout.mode != "RTL":
                        time.sleep(0.2)
                    break  # Break out of the inner marker loop
                else:
                    logging.warning("No ACK received from second drone. Retrying marker position update.")
                    print("No ACK received from second drone.")
                        
            # If message has been sent, break out of the outer loop entirely.
            if message_sent:
                break

        # If we have reached the maximum waypoint without marker detection, return to launch.
        if not message_sent and next_waypoint >= T_MAX:
            print("No marker found. Returning to launch.")
            scout.mode = VehicleMode("RTL")
            time.sleep(5)
            break

        time.sleep(1)

    print("Mission complete")
    logging.info("Mission End Time: %s", time.ctime(time.time()))
    scout.close()

if __name__ == "__main__":
    main()
