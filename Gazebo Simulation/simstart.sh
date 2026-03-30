#!/bin/bash

# Open a tab to run MAVProxy and SITL instance
# A second tab to run Gazebo world simulation
# A third for the drone simulation
# Add a fourth to get a second drone

gnome-terminal --tab --title="MAVProxy & SITL" -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out udp:127.0.0.1:14551"
# Port 14551 for Scout Drone, Port xxx for Ground Control, Port xxx for Delivery Drone (e.d. 14550)

gnome-terminal --tab --title="Gazebo Capstone World" -- bash -c "gazebo --verbose ~/ardupilot_gazebo/worlds/capstone.world"
gnome-terminal --tab --title="Scout Drone" -- bash -c "python3.8 speed_test.py --connect udpin:localhost:14551; exec bash"
