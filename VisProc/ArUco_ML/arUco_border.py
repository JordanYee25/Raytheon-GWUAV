import cv2
import numpy as np

def is_square(approx, tolerance=0.2):
    # Check that the approximated contour has 4 vertices and is convex.
    if len(approx) != 4 or not cv2.isContourConvex(approx):
        return False

    pts = approx.reshape(4, 2)
    # Compute the Euclidean distances between consecutive vertices.
    sides = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
    mean_side = np.mean(sides)

    # Check that all sides are roughly equal in length.
    if any(abs(side - mean_side) > tolerance * mean_side for side in sides):
        return False

    # Optionally, check that each angle is approximately 90 degrees.
    def angle(pt1, pt2, pt3):
        # Returns the angle (in degrees) between the vectors (pt1-pt2) and (pt3-pt2)
        v1 = pt1 - pt2
        v2 = pt3 - pt2
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
    
    angles = [angle(pts[i - 1], pts[i], pts[(i + 1) % 4]) for i in range(4)]
    if any(abs(a - 90) > 10 for a in angles):  # Allow some tolerance
        return False

    return True

def detect_square_marker_in_frame(frame):
    # Convert to grayscale and blur to reduce noise
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Perform Canny edge detection
    edged = cv2.Canny(blurred, 50, 150)

    # Find contours in the edged image
    contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for cnt in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # Filter out small contours and check if it's a square.
        if cv2.contourArea(approx) > 100 and is_square(approx):
            squares.append(approx)
    
    return squares

def main():
    # Capture video from the default camera (change the index or file path as needed)
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Error: Could not open video capture")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect squares in the current frame
        squares = detect_square_marker_in_frame(frame)

        # Draw detected squares on the frame
        for square in squares:
            cv2.drawContours(frame, [square], -1, (0, 255, 0), 2)

        # Display the frame
        cv2.imshow("Detected Squares", frame)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
