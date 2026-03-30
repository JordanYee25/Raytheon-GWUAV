"""
Lissajous search mission. The drone's starting point is random. The only information we need is the center of the field.
 
Lissajous curves are defined by two parametric equations:
[x,y] = < Ax*sin(wx * t + phix), Ay*sin(wy * t + phiy) >
    Ax = Amplitude in X-direction
    Ay = Amplitude in Y-direction
 
    wx = Angular frequency in X-dir
    wy = Angular frequency in Y-dir
 
    phix = Phase shift in X-dir
    phiy = Phase shift in Y-dir
 
 
Lissajous automatically bounds the field dimensions with Ax and Ay. A geofence will still be created
for extra caution and fail-safe behavior. Adjusting the six parameters leads to infinitely many patterns.
If ratio rw = wx/wy is rational, the curve is cyclical (repeating). Irrational -> the pattern doesn't ever
repeat.
 
Latitude, longitude, altitude, heading
"""
 
#############DEPENDENCIES#############
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import numpy as np
import math
import argparse
 
import cv2
import numpy as np
import time
import threading
import queue
 
#Used to track execution time throughout programa for logging, do:       time.time() - start_time
start_time = time.time()
print("Mission start")
  
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
    print("Lissajous curve search pattern") #let's make a repeating figure 8 for starters...
 
    """
 
    The array represents the Cartesian plane with the origin being in the middle of the array.
 
    +-------^-------+
    |       |       |
    |       |       |
    <-------+------->
    |       |       |
    |       |       |
    +---------------+
 
    The array will need to be twice the amplitudes in both directions + 1 to include the four quadrants.
    The amplitudes will only count half, and are therefore known as half-amplitudes.
 
    y-coordinates corrspond to rows. x-coordinates correspond to the columns.
 
    To access the cell that corresponds to the appropriate (x,y) points, follow this mapping:
 
    cell array[row] = -Y_point + AMP_Y
    cell array[column] = X_point + AMP_X
 
    So, an (x,y) pair maps to array[-Y_point + AMP_Y][X_point + AMP_X].
    and the origin (0,0), center of field, is array[AMP_Y][AMP_X]
    """
 
    cmds = scout.commands
    cmds.download() # Download current list of commands FROM drone
    cmds.wait_ready() # wait until download is complete
 
    print(" Clear any existing commands")
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
 
    print("Add lissajous waypoints.")
     
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
 
    print("YAY Lissajous curve made ^_^")
 
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

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)
    
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
 
    #Multithread the aruco detection here using tracker class
    tracker = aruco_tracker()
    detection_thread = threading.Thread(target=tracker.search)
    detection_thread.start()
 
    lissajous_search()
    
    arm_and_takeoff(SCOUT_ALT)
    print("Starting mission")
    
    scout.commands.next = 0 # Reset mission set to first (0) waypoint
 
    scout.mode = VehicleMode("AUTO")
    while scout.mode !="AUTO":
        time.sleep(.2)
    
    #To keep the print message for distance to waypoint consistently at 1 message per second (Instead of an unreadable number)
    #Get the current time, then check that if one second has passed since taking that time. If so print message, and update time
    #This way the message will be printed at least 1 second apart while the rest of the loop can run full speed, which is needed for
    #displaying openCV frames at a reasonable rate
    curr_time = time.time()
 
    while True:
        next_waypoint = scout.commands.next
        
        #Pulls a frame from the frame queue (which is really just a single frame buffer) and displays it
        if tracker.frame_queue:
            frame = tracker.frame_queue.get()            
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
            
        #Force shutdown for opencv frame, in other words manual override
        #cv2.waitKey(1) means wait one millisecond, it is more consistent to hold down the shutdown key than giving it a short tap
        if cv2.waitKey(1) == ord('q'):
            print("SHUTDOWN DETECTED")
            tracker.shutdown = True
            # detection_thread.join() #End thread
            break;
 
        #Checks if a previously valued variable of None has changed values. In other words, it has found a correct marker
        #In current testing phase it will break the loop and RTL, in future iterations it will center itself on aruco marker
        if tracker.found_marker is not None:
            print("Marker found")
            scout.mode = VehicleMode("GUIDED") # switch to guided mode
            while scout.mode !="GUIDED":
                time.sleep(.2)
            
            time.sleep(10)
            break;        
            
        if next_waypoint is T_MAX: #T_MAX is the last waypoint, make it an extra waypoint than is needed for lissajous
            print('End of search mission, Return to launch')
            scout.mode = VehicleMode("RTL")
            break;
        
        time.sleep(1)
      
    print("Close vehicle object")
    scout.close()
    tracker.cap.release()
    cv2.destroyAllWindows()
    detection_thread.join()
 
class aruco_tracker():
    #Aruco Detector class:
    #Competition day ID is unknown until competition day, assuming '2' for testing
    #We are using aruco 6x6 per competition spec
    #found marker will be checked by main script in main() for any value that isnt None
    def __init__(self):
        self.TARGET_ID = 0
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)                #Opencv detection parameters for detecting aruco markers
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
 
        self.found_marker = None
        self.frame_queue = queue.Queue(maxsize=1)                                                   #A queue of frames to show, max size one because you dont want a line of frames to pile up
        self.cap = cv2.VideoCapture(0)                                                              #The video capture for the tracker should not chnage
        self.shutdown = False                                                                       #For manual shut down of tracker class, set to True

        self.is_detected    = False
        self._kill          = False # I'm thinking shutdown is the same

        self._t_read      = time.time()
        self._t_detect    = self._t_read
        self.fps_read    = 0.0
        self.fps_detect  = 0.0 

        # Precision landing:
        #--- 180 deg rotation matrix around the x axis
        self._R_flip      = np.zeros((3,3), dtype=np.float32)
        self._R_flip[0,0] = 1.0
        self._R_flip[1,1] =-1.0
        self._R_flip[2,2] =-1.0

    def _rotationMatrixToEulerAngles(self,R):
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6        
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def _update_fps_read(self):
        t           = time.time()
        self.fps_read    = 1.0/(t - self._t_read)
        self._t_read      = t
        
    def _update_fps_detect(self):
        t           = time.time()
        self.fps_detect  = 1.0/(t - self._t_detect)
        self._t_detect      = t    

    def stop(self):
        self._kill = True
        self.shutdown = True
 
    #Simple search, only looks for the marker and does not do any POSE estimation or ML estimation    
    def search(self):
        #marker_found = False
        x = y = z = 0
 
        #This will condinue searching for a marker until either the correct marker is found or some shutdown signal is sent (self.shutdown)
        while self.found_marker is  None and self.shutdown is False:
            self._update_fps_detect()
            ret, frame = self.cap.read()
 
            if not ret:
                exit("Failed to grab frame")
 
            #detect markers in the frame
            corners, ids, rejected = self.detector.detectMarkers(frame)
                
            #Currently detected markers including non-target ones for logging purposes
            detected_markers = []
 
            #if any markers are detected...
            if ids is not None:
                for i in range(len(ids)):
                    #draw the id associated with the detected marker
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[ids[i][0]]], dtype=np.int32))
 
                    #Get the center of the markere based on the average coordinates of the corner
                    #This is the center in pixels and not any physical dimension
                    x = (corners[i-1][0][0][0] + corners[i-1][0][1][0] + corners[i-1][0][2][0] + corners[i-1][0][3][0]) / 4
                    y = (corners[i-1][0][0][1] + corners[i-1][0][1][1] + corners[i-1][0][2][1] + corners[i-1][0][3][1]) / 4
                    center = (x,y)
 
                    if ids[i][0] == self.TARGET_ID:
                        #convert marker corners to integer coordinates
                        int_corners = np.int32(corners[i])
                        #use integer coordinates to draw a red box around marker
                        cv2.polylines(frame, [int_corners], isClosed=True, color=(0, 0, 255), thickness=5)
                                        
                        #print(f"Distance to marker {TARGET_ID}: {distance:.2f} meters")
                        found_target = {
                            "real_time" : time.time(),
                            "program_time" : time.time() - start_time,                    
                            "target_id" : ids[i][0],
                            "center" : center,
                            "is_target" : True
                        }
                      
                        detected_markers.append(found_target) #Print to STDOUT
                        
                        #Set found marker to target which will be send to main script to block
                        self.found_marker = found_target
 
                        #draw the coordinate frame axes and print distance onto the frame
                        cv2.putText(frame, f"DropZone Found {self.TARGET_ID}", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        break
                    else:
                        found_target = {
                            "real_time" : time.time(),
                            "program_time" : time.time() - start_time,                    
                            "target_id" : ids[i][0],
                            "center" : center,
                            "is_target" : False
                        }
                        cv2.putText(frame, "Non-DropZone", (10, 110),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)  
                        
                        detected_markers.append(found_target)

            #We can show the frame on a threa due to opencv not being threat safe and python not allowing it
            #Frames are put onto a queue to be displayed on the main thread, where "frames" are treated as
            #immutable values
            self.frame_queue.put(frame)
     
if __name__ == "__main__":
 
    main()
