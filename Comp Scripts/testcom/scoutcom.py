import time
import numpy as np
import math
import os
from os import path, sys
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
PORT = 'com4'
BAUDRATE=57600
TIMEOUT=1

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
CENTER_LAT  = 38.8977
CENTER_LONG = -77.0365
EARTH_RADIUS = 6378.137   # km


############# HELPER FUNCTIONS #############
def get_current_gps(vehicle):
    location = vehicle.location.global_relative_frame
    return location.lat, location.lon, location.alt

def send_coords_and_wait_ack(ser, lat, lon,
                             max_retries=5,
                             retry_delay=0.5,
                             redo_msg="redo\n"):
    
    coord_msg = f"{lat:.7f},{lon:.7f}\n".encode('utf-8')
    expected_ack = coord_msg.decode('utf-8').strip()  # drop the newline
    redo_bytes = redo_msg.encode('utf-8')

    for attempt in range(1, max_retries + 1):
        try:
            # 1) send coords
            ser.write(coord_msg)
            logging.info(f"[Comm] Sent coords (attempt {attempt}): {coord_msg.decode().strip()}")

            # 2) wait for ack
            raw = ser.readline().decode('utf-8').strip()
            logging.info(f"[Comm] Received raw ack: {raw}")

            # 3a) good ack → done
            if raw == expected_ack:
                logging.info("[Comm] Ack matches, continuing.")
                return True

            # 3b) bad ack → tell remote to REDO
            logging.warning(f"[Comm] Unexpected ack (“{raw}”), sending REDO to reset.")
            ser.write(redo_bytes)
            logging.info(f"[Comm] Sent REDO message: {redo_msg.strip()}")

        except Exception as e:
            logging.error(f"[Comm] Serial error on attempt {attempt}: {e}")

        # wait before retrying
        time.sleep(retry_delay)

    logging.error(f"[Comm] Failed to get valid ack after {max_retries} attempts")
    return False


def send_go_command(ser):
    """
    Send the 'authorized' signal once we have a good ack.
    """
    go_msg = "authorized\n".encode('utf-8')
    ser.write(go_msg)
    logging.info("[Comm] Sent authorized command")
    return True
    
############# MAIN METHOD #############
def main():
    mission_start_time = time.time()
    logging.info("UAV Start Time: %s", time.ctime(mission_start_time))

    ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)


    while True:
        logging.info("Marker Discovery Time: %s", time.ctime())
        lat, lon = (000, 000)

        if send_coords_and_wait_ack(ser, lat, lon):
            if send_go_command(ser):
                break
        else:
            # bail out
            logging.error("Could not establish handshake—entering LAND mode anyway.")
            mode = "LAND"
            while mode != "LAND":
                time.sleep(0.2)
            break  # exit your mission loop


    # ——— 6) cleanup ———
    logging.info("Mission complete at %s", time.ctime())
    print("Mission complete at", time.ctime())
    ser.close()

if __name__ == "__main__":
    main()
