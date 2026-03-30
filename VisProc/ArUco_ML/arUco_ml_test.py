import cv2
from ultralytics import YOLO
import csv
import time

def main():
    # 1. Load YOLO model (Make sure to import this to drone)
    model = YOLO("runs/detect/my_experiment/weights/best.pt")
    
    # 2. Set up a video capture. Should be 0 of not just check other numbers
    cap = cv2.VideoCapture(0)
    
    # 3. ArUco dictionary setup (choose the correct one)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters_create()
    
    # 4. Create a unique run identifier using a timestamp for file names
    run_id = time.strftime("%Y%m%d-%H%M%S")
    video_filename = f"run_{run_id}.avi"
    csv_filename = f"detection_log_{run_id}.csv"
    
    # 5. Set up VideoWriter to record the annotated video
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_filename, fourcc, 20.0, (frame_width, frame_height))
    
    # 6. Open a CSV file to log detection data
    csv_file = open(csv_filename, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['timestamp', 'class', 'score', 'bbox', 'aruco_ids', 'aruco_corners'])
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 7. YOLO Inference on the current frame
        results = model.predict(frame, conf=0.5)
        # Create an annotated copy of the frame
        annotated_frame = results[0].plot()

        # 8. Process each detected box from YOLO
        boxes = results[0].boxes
        for box in boxes:
            cls = int(box.cls[0])  # class index
            score = float(box.conf[0])
            if score < 0.5:
                continue

            # Suppose "aruco" is class 0
            if cls == 0:
                # Extract the bounding box coordinates (x1,y1,x2,y2)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                roi = frame[y1:y2, x1:x2]

                # 9. Run OpenCV ArUco detection on the ROI
                corners, ids, _ = cv2.aruco.detectMarkers(roi, aruco_dict, parameters=aruco_params)
                aruco_ids = []
                aruco_corners = []
                if ids is not None and len(ids) > 0:
                    aruco_ids = ids.flatten().tolist()
                    for corner in corners:
                        # Flatten the corner coordinates for logging
                        aruco_corners.append(corner.reshape(-1).tolist())
                    
                    # draws detected marker corners on ROI
                    cv2.aruco.drawDetectedMarkers(roi, corners, ids)
                    # Replace the ROI in the annotated frame with the annotated ROI
                    annotated_frame[y1:y2, x1:x2] = roi

                # 10. Log detection data (timestamp, class, confidence, bounding box, ArUco IDs, and corners)
                detection_time = time.time()
                bbox = [x1, y1, x2, y2]
                csv_writer.writerow([detection_time, cls, score, bbox, aruco_ids, aruco_corners])
                print(f"Logged detection at {detection_time}: class {cls}, score {score}, bbox {bbox}, ArUco IDs {aruco_ids}")

        # 11. Write the annotated frame to the video file
        out.write(annotated_frame)

        # 12. Display the annotated frame (i don't this worked last time so im going to comment this section out for now)
        
        #cv2.imshow("YOLO+ArUco", annotated_frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break

    # 13. Release resources and close files
    cap.release()
    out.release()
    csv_file.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
