import cv2
from ultralytics import YOLO

def main():
    # 1. Load YOLO model (trained to detect "aruco" class, for instance)
    model = YOLO("runs/detect/my_experiment/weights/best.pt")

    # 2. Set up a video capture (0 for webcam, or a file path)
    cap = cv2.VideoCapture(0)

    # 3. ArUco dictionary setup (choose correct one)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters_create()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 4. YOLO Inference
        results = model.predict(frame, conf=0.5)

        # 5. Annotate YOLO results
        annotated_frame = results[0].plot()

        # 6. For each detection, if the class == "aruco", run ArUco detect on ROI
        boxes = results[0].boxes
        for box in boxes:
            cls = int(box.cls[0])  # class index
            score = float(box.conf[0])
            if score < 0.5:
                continue

            # Suppose "aruco" is class 0
            if cls == 0:
                # Extract bounding box in x1,y1,x2,y2 format
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # The predicted bounding box
                roi = frame[y1:y2, x1:x2]

                # Run OpenCV ArUco detection on the ROI
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    roi, 
                    aruco_dict, 
                    parameters=aruco_params
                )

                if ids is not None and len(ids) > 0:
                    # We found the ArUco ID(s)
                    print(f"Detected ArUco IDs: {ids.flatten().tolist()}")

                    # Optionally draw detected marker corners on ROI
                    cv2.aruco.drawDetectedMarkers(roi, corners, ids)

                    # Put that annotated ROI back into the main frame
                    annotated_frame[y1:y2, x1:x2] = roi

        # 7. Show results
        cv2.imshow("YOLO+ArUco", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
