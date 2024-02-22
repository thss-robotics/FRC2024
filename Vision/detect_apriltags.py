import cv2
import numpy as np
from pupil_apriltags import Detector

# Initialize camera
cap = cv2.VideoCapture(0)

# Configure AprilTag detector
at_detector = Detector(families='tag36h11',
                       nthreads=4,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

def draw_cube(img, corners, tag_center):
    
    # Define a simple cube with 8 points
    cube = np.float32([[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0],
                       [0, 0, -1], [0, 1, -1], [1, 1, -1], [1, 0, -1]]).reshape(-1, 3)

    # Adjust these values for cam
    focal_length = 1
    cube_size = 1
    camera_matrix = np.array([[focal_length, 0, tag_center[0]],
                              [0, focal_length, tag_center[1]],
                              [0, 0, 1]], dtype=float)

    # Assuming no lens distortion
    dist_coeffs = np.zeros((4,1))

    # Project cube points onto the image plane
    imgpts, _ = cv2.projectPoints(cube * cube_size, (0,0,0), (0,0,0), camera_matrix, dist_coeffs)

    imgpts = np.int32(imgpts).reshape(-1,2)

    # Draw ground floor in green
    cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)

    # Draw pillars in blue
    for i,j in zip(range(4), range(4, 8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255), 3)

    # Draw top layer in red
    cv2.drawContours(img, [imgpts[4:]], -1, (0,0,255), 3)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

    for tag in tags:
        corners = np.int32(tag.corners)
        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
        tag_center = np.mean(corners, axis=0)
        cv2.putText(frame, str(tag.tag_id), (int(tag_center[0]), int(tag_center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        draw_cube(frame, corners, tag_center)

    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
