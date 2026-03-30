########DEPENDENCIES###############
import json 
import time
import math
import logging
import serial
import os
from os import path, sys

############# TELEMETRY RADIO CONNECTION #############
PORT = 'COM4'
BAUDRATE=57600
TIMEOUT=1

############# SET UP LOGGING #############
# Ensure logs directory exists in the current working directory
log_dir = os.path.join(os.getcwd(), "Logpd")
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

############# MAIN METHOD #############

def main():
    # set up logging
    log_dir = os.path.join(os.getcwd(), "Logs")
    os.makedirs(log_dir, exist_ok=True)
    log_filename = os.path.join(log_dir, f"mission_{time.strftime('%Y%m%d_%H%M%S')}_P.log")
    logging.basicConfig(filename=log_filename,
                        level=logging.INFO,
                        filemode='w',
                        format='%(asctime)s - %(levelname)s - %(message)s')

    # open serial port
    ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
    logging.info("UXV connected—waiting for mission authorization")

    while True:
        # — Receive “lat,lon” —
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        logging.info("Received coords request: %s", line)
        ("Received coords request: %s", line)
        try:
            lat_str, lon_str = line.split(',')
            target_lat = float(lat_str)
            target_lon = float(lon_str)
        except ValueError:
            logging.warning("Malformed coords, ignoring: %s", line)
            print("Malformed coords, ignoring: %s", line)
            continue

        # — Send ACK (echo back) —
        ack = f"{target_lat:.7f},{target_lon:.7f}\n".encode('utf-8')
        ser.write(ack)
        logging.info("Sent ACK: %s", ack.decode().strip())
        print("Sent ACK: %s", ack.decode().strip())

        # Wait for “authorized” or “redo”
        cmd = ser.readline().decode('utf-8').strip()
        if cmd == "authorized":
            logging.info("Authorized received—proceeding with mission")
        elif cmd.lower() == "redo":
            logging.info("Received 'redo'—restarting handshake")
            continue
        else:
            logging.warning("Unexpected '%s'—restarting handshake", cmd)
            continue

        # Drop package
        logging.info("Package released")
        time.sleep(1)

        logging.info("UXV Mission Complete Time: %s", time.ctime())
        break

    ser.close()

if __name__ == "__main__":
    main()