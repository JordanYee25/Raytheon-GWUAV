"""
Test at drone lab the communications message passing and package drop.
A computer will send an okay signal to drone while in manual flight to release package.
"""

########DEPENDENCIES###############
import json 
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import math
import argparse
import logging
import serial
import os
from os import path, sys
from gpiozero import Motor
from drop import open as drop_open
from drop import close as drop_close


############# TELEMETRY RADIO CONNECTION #############
PORT = '/dev/ttyUSB0'
BAUDRATE=57600
TIMEOUT=1


def main():
   
    # open serial port
    ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
    logging.info("UXV connected—waiting for mission authorization")

    while True:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        try:
            if line == "DROP":
                time.sleep(2)          # let the drone settle on the ground
                open()            # actuate your drop mechanism
                time.sleep(1)
                close()
        except ValueError:
            print("Error: %s:, line)
            continue
        """
        # — Send ACK (echo back) —
        ack = f"{target_lat:.7f},{target_lon:.7f}\n".encode('utf-8')
        ser.write(ack)
        logging.info("Sent ACK: %s", ack.decode().strip())

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

        #Add correct height for package drop, or LAND
        
        time.sleep(2)          # let the drone settle on the ground
        open()            # actuate your drop mechanism
        time.sleep(1)
        close()

        #Add behavior of what to do after package drop, like RTL or LAND

        logging.info("UXV Mission Complete Time: %s", time.ctime())
        break

    # — Cleanup —
    #delivery_drone.close()
    ser.close()
"""
if __name__ == "__main__":
    main()
