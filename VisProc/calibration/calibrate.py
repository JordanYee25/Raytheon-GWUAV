import cv2
from cv2 import aruco
import numpy as np
import os

CHECKERBOARD_ROWS = 7       #Calibration checkerboard dimensions
CHECKERBOARD_COLS = 6

CHARUCO_BOARD_COLS = 15     #ChArUco board dimensions
CHARUCO_BOARD_ROWS = 9

                            # Dimensions of target marker are in meters
SQUARE_SIZE = 0.108         # Square Size is the length of one edge of the inner squares
MARKER_SIZE = 0.01884       # Marker Size is the length of one edge of the entire marker

image_size = (1920,1200)    


objp = np.zeros((CHECKERBOARD_ROWS * CHECKERBOARD_COLS, 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_COLS, 0:CHECKERBOARD_ROWS].T.reshape(-1, 2)
objp *= SQUARE_SIZE


object_points = []
image_points = []

def checkerboard_read(images):
    for i, image in enumerate(images):
        print(f"Processing image {i+1}/{len(images)}: {image}")
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (6, 7), None)

        if ret:
            #refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-5)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

            image_points.append(corners2)
            object_points.append(objp)

            #Save
            cv2.drawChessboardCorners(img, (CHECKERBOARD_COLS, CHECKERBOARD_ROWS), corners2, ret)
            processed_path = f"./processed/{os.path.basename(image)}"
            cv2.imwrite(processed_path, img)
        else:
            print("Checkerboard not detected")

    
def checkerboard_calibrate():

        if len(object_points) == 0 or len(image_points) == 0:
            print("No object points")
            exit()
        
        flags = cv2.CALIB_RATIONAL_MODEL  # Enables 8-coefficient model
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
        criteria=criteria,
        flags=flags
        )
     
        #Reprojection images Debugging only
        #This shows the calibration array of image points ontop of detected image points
        #Pretty much the finished camera calibration's interpretation of the edges vs the actual detected edges
        #Comes with some nuance, because how do we know the detected edges are completely accurate
        #The more centered the blue is on the green dot, the closer the calbiratio is to aruco detection levels
        for i, (image_path, obj_pts, img_pts) in enumerate(zip(images, object_points, image_points)):
            print(f"Reprojection image {i + 1}/{len(images)}: {image_path}")
            image_data = cv2.imread(image_path)
            if image_data is None:
                print(f"Failed to load image {image_path}. Skipping...")
                continue

            #Reproject the 3D object points into the 2D image plane
            reprojected_points, _ = cv2.projectPoints(obj_pts, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)

            #detected points (green)
            for point in img_pts:
                x, y = point.ravel()
                cv2.circle(image_data, (int(x), int(y)), 5, (0, 255, 0), -1)  

            for detected_point, reprojected_point in zip(img_pts, reprojected_points):
                x1, y1 = detected_point.ravel()  # Detected point
                x2, y2 = reprojected_point.ravel()  # Reprojected point

                # Apply scaling factor to the line vector
                x2_scaled = x1 + (x2 - x1) * 10
                y2_scaled = y1 + (y2 - y1) * 10

                # Draw the reprojected point
                cv2.circle(image_data, (int(x2), int(y2)), 3, (255, 0, 0), -1)  # Blue for reprojected points

                # Draw a vector line indicating the scaled offset
                cv2.line(image_data, (int(x1), int(y1)), (int(x2_scaled), int(y2_scaled)), (0, 0, 255), 1)  # Red line for vector

            os.makedirs("./processed", exist_ok=True)
            processed_path = f"./processed/reprojection_{os.path.basename(image_path)}"
            cv2.imwrite(processed_path, image_data)

        # Output results
        print("---------------------------------------")
        print("Calibration Successful! \nRMSE: ", ret)
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", dist_coeffs)
        np.savez('calibration.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)



datadir = "./output/"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png") ])
order = np.argsort([int(p.split(".")[-2].split("_")[-1]) for p in images])
images = images[order]

#charuco_read(images)
#charuco_calibrate()

checkerboard_read(images)
checkerboard_calibrate()