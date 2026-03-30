#############DEPENDENCIES#############
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import numpy as np
import math
import argparse

########CONSTANTS#############
pi = math.pi
T_STEP = pi/8 # defines granularity of curve with more (x,y) coordinates
T_MAX = 64 # how many time points we want from 0 - T_MAX (though it doesn't have to be 0) & should be enough for the challenge duration
AMP_X = 10 # meters
AMP_Y = 10 # field dimensions are 90ft x 90ft ~ approx. 27.4 m. The curve will be slightly less than boundary to not trigger geofence
W_X = 1
W_Y = 2
PHI_X = pi/2
PHI_Y = 0
SCOUT_ALT = 10 # meters ~ approx. 33 ft
CENTER_LAT = -35.363262
CENTER_LONG = 149.165237 # Default Gazebo/SITL params
EARTH_RADIUS = 6378.137 # km

#############CONNECTION#############
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
scout = connect(connection_string, wait_ready=True)

#####SETTING UP THE LISSAJOUS CURVE#######
def lissajous_search():
    cmds = scout.commands
    cmds.download() # Download current list of commands FROM drone
    cmds.wait_ready() # wait until download is complete
    
    cmds.clear() # Clear list before adding new ones

    rows = 2*AMP_Y + 1
    columns = 2*AMP_X + 1 # store longitude and latitude coordinates every 1 meter apart

    met_to_deg = (1 / ((pi/180) * EARTH_RADIUS)) / 1000 # converting between degrees to meters, constant

    # latitude is approx. constant at all points of the Earth
    # new latitude = original latitude + translation_meters * meters_to_degrees
    # positive translation -> move up
    # negative translation -> move down
    np.set_printoptions(precision=10)
    latitude_arr = np.arange(rows*columns, dtype=np.float64).reshape(rows, columns)
    for row in range(0, rows):
        for col in range(0, columns):
            latitude_arr[row][col] = CENTER_LAT +  (AMP_Y - row) * met_to_deg
            #print(latitude_arr[row][col])

    # longitude varies with latitude degrees
    # new longitude = original longitude + (translation_mters * meters_to_degrees / cos(original long. * pi/180))
    # positive translation -> move left
    # negative translation -> move down
    longitude_arr = np.arange(rows*columns, dtype=np.float64).reshape(rows, columns)
    for row in range(0, rows):
        for col in range(0, columns):
            longitude_arr[row][col] = CENTER_LONG + ((AMP_X - col) * met_to_deg)/(math.cos(CENTER_LAT * (pi/180)))
            #print(longitude_arr[row][col])
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, SCOUT_ALT))

    for T_RANGE in range(0, T_MAX):
        X_point = math.ceil( AMP_X * math.sin(W_X * T_STEP * T_RANGE + PHI_X) )
        Y_point = math.ceil( AMP_Y * math.sin(W_Y * T_STEP * T_RANGE + PHI_Y) )
        print("(x,y) = (%s, %s). lat: %s. long: %s" %(X_point, Y_point, format(latitude_arr[-Y_point + AMP_Y][X_point + AMP_X], ".10f"), format(longitude_arr[-Y_point + AMP_Y][X_point + AMP_X],".10f")))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                                   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                                   latitude_arr[-Y_point + AMP_Y][X_point + AMP_X], 
                                   longitude_arr[-Y_point + AMP_Y][X_point + AMP_X], SCOUT_ALT))

    print(" Upload search pattern to vehicle")
    cmds.upload()

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat)+(dlong*dlong))*1.113195e5 # 1.113195e5 is the number of metres per degree of lat/long
    
def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = scout.commands.next
    if nextwaypoint==0:
        return None
    missionitem=scout.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(scout.location.global_frame, targetWaypointLocation)
    return distancetopoint

def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not scout.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    scout.mode = VehicleMode("GUIDED")
    scout.armed = True

    while not scout.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    scout.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", scout.location.global_relative_frame.alt)      
        if scout.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def main():
    lissajous_search()

    arm_and_takeoff(10)
    print("Starting mission")
    
    scout.commands.next = 0 # Reset mission set to first (0) waypoint

    scout.mode = VehicleMode("AUTO")
    while scout.mode !="AUTO":
        time.sleep(.2)
        
    while True:
        next_waypoint = scout.commands.next
        print('Distance to waypoint (%s): %s' %(next_waypoint, distance_to_current_waypoint()))
        
        if next_waypoint is T_MAX: #T_MAX is the last waypoint, make it an extra waypoint than is needed for lissajous
            print("Exit 'standard' mission when last waypoint reached")
            break;
        
        time.sleep(1)

    print('Return to launch')
    scout.mode = VehicleMode("RTL")

    #Close vehicle object before exiting script
    print("Close vehicle object")
    scout.close()
 
if __name__ == "__main__":

    main()
