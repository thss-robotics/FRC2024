import cv2
import numpy as np
import time

def main():
    # Instead of reading camera config from '/boot/frc.json', set manual configuration
    width, height = 640, 480

    # Simulate camera capture using your laptop's webcam
    input_stream = cv2.VideoCapture(0)
    if not input_stream.isOpened():
        print("Error: Could not open video device.")
        return

    # Set the width and height if needed (depends on your camera)
    input_stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    input_stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Replace NetworkTables with a simple dictionary (for demonstration)
    vision_nt = {}

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    while True:
        start_time = time.time()

        # Read frame
        ret, input_img = input_stream.read()
        if not ret:
            print("Error: No frame captured.")
            continue

        output_img = np.copy(input_img)

        # Convert to HSV and threshold image
        hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        binary_img = cv2.inRange(hsv_img, (0, 0, 100), (85, 255, 255))

        _, contour_list, _ = cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

        x_list = []
        y_list = []

        for contour in contour_list:

            # Ignore small contours that could be because of noise/bad thresholding
            if cv2.contourArea(contour) < 15:
                continue

            cv2.drawContours(output_img, [contour], -1, color=(255, 255, 255), thickness=-1)

            rect = cv2.minAreaRect(contour)
            center, size, angle = rect
            center = tuple([int(dim) for dim in center])  # Convert to int so we can draw

            # Draw rectangle and circle
            cv2.drawContours(output_img, [cv2.boxPoints(rect).astype(int)], -1, color=(0, 0, 255), thickness=2)
            cv2.circle(output_img, center=center, radius=3, color=(0, 0, 255), thickness=-1)

            x_list.append((center[0] - width / 2) / (width / 2))
            y_list.append((center[1] - height / 2) / (height / 2))

        vision_nt['target_x'] = x_list
        vision_nt['target_y'] = y_list

        processing_time = time.time() - start_time
        fps = 1 / processing_time
        cv2.putText(output_img, f"FPS: {round(fps, 1)}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        cv2.imshow('Processed', output_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    input_stream.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
