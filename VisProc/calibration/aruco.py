import cv2
import numpy as np
import time

#Used to track execution time throughout programa for logging, do:       time.time() - start_time
start_time = time.time()
print("Program start")

calibration_data = np.load('./calibration.npz')
camera_matrix =  calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

dict =  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

MARKER_SIZE = .252 #old marker    0.254 new marker    0.305 Size of marker (not including white border)
TARGET_ID = 2

#get video feed
cap = cv2.VideoCapture(0)

#Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print("Width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print("Height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

arucoType = None

#Change parameters for higher threshhold
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dict, parameters)


while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        continue
            
            
    corners, ids, rejected = detector.detectMarkers(frame)


    #Currently detected markers including non-target ones for logging purposes
    detected_markers = []

    #if any markers are detected...
    if ids is not None:
        for i in range(len(ids)):

            x = (corners[i-1][0][0][0] + corners[i-1][0][1][0] + corners[i-1][0][2][0] + corners[i-1][0][3][0]) / 4
            y = (corners[i-1][0][0][1] + corners[i-1][0][1][1] + corners[i-1][0][2][1] + corners[i-1][0][3][1]) / 4
            center = (x,y)


            #draw the id associated with the detected marker
            cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[ids[i][0]]], dtype=np.int32))

            #for the marker we want
            if ids[i][0] == TARGET_ID:
                #convert marker corners to integer coordinates
                int_corners = np.int32(corners[i])
                #use integer coordinates to draw a red box around marker
                cv2.polylines(frame, [int_corners], isClosed=True, color=(0, 0, 255), thickness=3)
                
                #estimate the pose of target marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([corners[i]], MARKER_SIZE, camera_matrix, dist_coeffs)

                #calculate and print the distance to console
                distance = np.linalg.norm(tvec)
                
                #print(f"Distance to marker {TARGET_ID}: {distance:.2f} meters")
                #Updated log information for more dynamic logs
                found_target = {
                    "real_time" : time.time(),
                    "program_time" : time.time() - start_time,                    
                    "target_id" : ids[i][0],
                    "target_distance" : distance,
                    "Center" : center,
                    "is_target" : True
                } 
                detected_markers.append(found_target)
                
                #draw the coordinate frame axes and print distance onto the frame
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
                cv2.putText(frame, f"Distance: {distance:.2f}m", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f"DropZone {TARGET_ID}", (10, 70), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                break
            else: 
                #Do not highlight the marker but still find its distance for logs
                int_corners = np.int32(corners[i])
                
                #estimate the pose of target marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, camera_matrix, dist_coeffs)

                #calculate and print the distance to console
                distance = np.linalg.norm(tvec)

                found_target = {
                    "real_time" : time.time(),
                    "program_time" : time.time() - start_time,                    
                    "target_id" : ids[i][0],
                    "target_distance" : distance,
                    "is_target" : False
                } 

                detected_markers.append(found_target)

                cv2.putText(frame, "Non-DropZone", (10, 110), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)  
        print(detected_markers)
                          

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        print("SHUTDOWN KEY DETECTED - Shutdown in 3 seconds")
        break

cap.release()
cv2.destroyAllWindows()
time.sleep(3)  #Make sure the camera is released
