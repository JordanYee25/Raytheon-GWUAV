import cv2
import numpy as np
import json
import time
import tkinter as tk
from PIL import Image, ImageTk

#constants
MARKER_SIZE = 0.3556 
TARGET_ID = 2
PIXEL_TOLERANCE = 30   #pixels
ANGLE_TOLERANCE = 6    #degrees
LOG_INTERVAL = 0.5     #seconds

#aruco dictionary mapping
ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_6X6_100
}

last_log_time = time.time()
log_text_widget = None  #global text widget for logs
stage = "searching"     #global stage variable

def load_calibration_data(filepath='calibration_data_live.npz'):
    """
    load camera calibration data from a numpy .npz file.
    returns:
        camera_matrix (np.ndarray): camera intrinsic parameters.
        dist_coeffs (np.ndarray): distortion coefficients.
    """
    calibration_data = np.load(filepath)
    camera_matrix = calibration_data['camera_matrix']
    dist_coeffs = calibration_data['dist_coeffs']
    return camera_matrix, dist_coeffs

def init_video_capture(camera_index=1):
    """
    initialize video capture.
    returns:
        cap (cv2.VideoCapture): the video capture object.
    """
    return cv2.VideoCapture(camera_index)

def log_json(event, **kwargs):
    """
    log events in json format if the log_interval has passed.
    """
    global last_log_time, log_text_widget
    current_time = time.time()
    if current_time - last_log_time >= LOG_INTERVAL:
        log_entry = {"event": event, **kwargs}
        log_msg = json.dumps(log_entry)
        print(log_msg)
        if log_text_widget is not None:
            log_text_widget.insert(tk.END, log_msg + "\n")
            log_text_widget.see(tk.END)
        last_log_time = current_time

def calculate_angle(rvec):
    """
    calculate the angle between the camera's z-axis and the marker's z-axis.
    """
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    z_axis = rotation_matrix[:, 2]
    camera_z_axis = np.array([0, 0, 1])
    #normalize both vectors
    z_axis_norm = z_axis / np.linalg.norm(z_axis)
    camera_z_axis_norm = camera_z_axis / np.linalg.norm(camera_z_axis)
    #compute the angle via the dot product
    cos_theta = np.dot(z_axis_norm, camera_z_axis_norm)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    angle_rad = np.arccos(cos_theta)
    return np.degrees(angle_rad)

def search_aruco(frame, aruco_type, parameters, camera_matrix, dist_coeffs):
    """
    search for aruco markers in the frame and estimate pose for each detected marker.
    
    returns:
        corners, ids, rvecs, tvecs if markers are detected;
        otherwise (None, None, None, None).
    """
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_type, parameters=parameters)
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, markerLength=MARKER_SIZE, cameraMatrix=camera_matrix, distCoeffs=dist_coeffs
        )
        return corners, ids, rvecs, tvecs
    else:
        return None, None, None, None

def get_move_directions(center_x, center_y, marker_center_x, marker_center_y):
    """
    compute movement directions based solely on the offset between the marker center and frame center.
    Since the drone is flying in a 2D plane, only left/right and forward/backward instructions are provided.
    
    returns:
        a list of instruction strings.
    """
    directions = []
    horizontal_offset = marker_center_x - center_x
    vertical_offset = marker_center_y - center_y

    if horizontal_offset < -PIXEL_TOLERANCE:
        directions.append("move left")
    elif horizontal_offset > PIXEL_TOLERANCE:
        directions.append("move right")
    
    if vertical_offset < -PIXEL_TOLERANCE:
        directions.append("move forward")
    elif vertical_offset > PIXEL_TOLERANCE:
        directions.append("move backward")
    
    if not directions:
        directions.append("hold position")
    
    return directions

def center(frame, corners, ids, rvecs, tvecs, camera_matrix, dist_coeffs):
    """
    determine whether the target marker is centered based on horizontal, vertical, and angle tolerances.
    Draw bounding boxes on the frame:
      - For the target marker: green if centered, blue if misaligned.
      - For non-target markers: red.
    Additionally, draw the z-axis bar (using drawFrameAxes) on the target marker.
    
    returns:
        frame (with bounding boxes and z-axis bar), marker_status (True if centered, False if misaligned),
        and a list of movement directions.
    """
    marker_status = None
    move_directions = []
    frame_height, frame_width = frame.shape[:2]
    center_x, center_y = frame_width // 2, frame_height // 2

    for i in range(len(ids)):
        if ids[i][0] == TARGET_ID:
            int_corners = np.int32(corners[i])
            marker_center_x = int(np.mean(int_corners[0][:, 0]))
            marker_center_y = int(np.mean(int_corners[0][:, 1]))
            rvec = rvecs[i][0]
            roll_angle = calculate_angle(rvec)
            # marker is considered centered if pixel offsets and roll angle are within tolerance
            marker_status = (
                abs(marker_center_x - center_x) <= PIXEL_TOLERANCE and
                abs(marker_center_y - center_y) <= PIXEL_TOLERANCE and
                abs(roll_angle - 180) <= ANGLE_TOLERANCE
            )
            move_directions = get_move_directions(center_x, center_y, marker_center_x, marker_center_y)
            if marker_status:
                log_json("centered", marker_center_x=marker_center_x,
                         marker_center_y=marker_center_y, roll_angle=round(roll_angle, 2),
                         directions=", ".join(move_directions))
                color = (0, 255, 0)  # green for centered target marker
            else:
                log_json("misaligned", marker_center_x=marker_center_x,
                         marker_center_y=marker_center_y, center_x=center_x,
                         center_y=center_y, roll_angle=round(roll_angle, 2),
                         directions=", ".join(move_directions))
                color = (255, 0, 0)  # blue for misaligned target marker
            # Draw the z-axis bar on the target marker.
            tvec = tvecs[i][0]
            axis_length = MARKER_SIZE / 2
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_length, 2)
        else:
            color = (0, 0, 255)  # red for non-target markers
        cv2.polylines(frame, [np.int32(corners[i])], True, color, thickness=5)
    return frame, marker_status, move_directions

def process_frame(frame, camera_matrix, dist_coeffs, aruco_type, parameters):
    """
    wrapper that calls search_aruco and center sequentially.
    
    returns:
        annotated frame, marker_status, and movement directions.
    """
    corners, ids, rvecs, tvecs = search_aruco(frame, aruco_type, parameters, camera_matrix, dist_coeffs)
    if ids is not None:
        frame, marker_status, directions = center(frame, corners, ids, rvecs, tvecs, camera_matrix, dist_coeffs)
    else:
        marker_status = None
        directions = []
    return frame, marker_status, directions

def gui_main():
    """
    create a tkinter gui that displays the video feed with the processed frame,
    stage info (searching, centering, hold) and movement directions on a side panel,
    and a fixed-size text box that shows console logs.
    """
    global log_text_widget, stage
    camera_matrix, dist_coeffs = load_calibration_data()
    cap = init_video_capture()
    parameters = cv2.aruco.DetectorParameters()
    aruco_type = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_ARUCO_ORIGINAL"])

    root = tk.Tk()
    root.title("AVC GWUAV Visual Processing")

    #use pack to keep the video frame fixed on the left.
    video_frame = tk.Frame(root)
    video_frame.pack(side=tk.LEFT, padx=10, pady=10)

    video_label = tk.Label(video_frame)
    video_label.pack()

    #status panel on the right
    status_frame = tk.Frame(root)
    status_frame.pack(side=tk.LEFT, padx=10, pady=10, anchor="n")

    status_label = tk.Label(status_frame, text="stage: initializing...", font=("helvetica", 16), justify="left")
    status_label.pack()

    #fixed-size log frame at the bottom
    log_frame = tk.Frame(root, width=800, height=300)
    log_frame.pack(side=tk.BOTTOM, padx=10, pady=10)
    log_frame.pack_propagate(False)

    log_text_widget = tk.Text(log_frame, height=20, width=200)
    log_text_widget.pack(side="left", expand=False)
    
    scrollbar = tk.Scrollbar(log_frame, command=log_text_widget.yview)
    scrollbar.pack(side="right", fill="y")
    log_text_widget.config(yscrollcommand=scrollbar.set)


    def update_frame():
        global stage
        ret, frame = cap.read()
        if ret:
            processed_frame, marker_status, directions = process_frame(frame, camera_matrix, dist_coeffs, aruco_type, parameters)
            #enlarge the video frame by resizing (fixed scale factor 1.1)
            scale_factor = 1.1
            processed_frame = cv2.resize(processed_frame, None, fx=scale_factor, fy=scale_factor)
            frame_rgb = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)
            if marker_status is None:
                stage = "searching"
                status_label.config(text="stage: searching", fg="red")
            elif marker_status:
                stage = "hold"
                status_label.config(text="stage: hold\ninstructions: " + ", ".join(directions), fg="green")
            else:
                stage = "centering"
                status_label.config(text="stage: centering\ninstructions: " + ", ".join(directions), fg="blue")
        root.after(30, update_frame)

    update_frame()
    root.mainloop()
    cap.release()

if __name__ == '__main__':
    gui_main()