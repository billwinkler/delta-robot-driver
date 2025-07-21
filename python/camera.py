# python/camera.py
import os
import time
import cv2

# Constants
DATA_DIR = 'pecan_data'
os.makedirs(DATA_DIR, exist_ok=True)

def capture_image():
    """
    Captures an image from the camera, resizes it, and saves it to the data directory.
    """
    camera = None
    for i in range(5):
        camera = cv2.VideoCapture(i)
        if camera.isOpened():
            print(f"Camera found at index {i}")
            break
        camera.release()

    if camera is None or not camera.isOpened():
        raise IOError("Cannot open camera")

    try:
        time.sleep(0.5) # wait for camera to initialize
        ret, frame = camera.read()
        if ret:
            resized_frame = cv2.resize(frame, (224, 224))  # Resize for model input
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(DATA_DIR, f"capture_{timestamp}.jpg")
            cv2.imwrite(filename, resized_frame)
            print(f"Image saved to {filename}")
            return filename
        else:
            raise ValueError("Failed to capture frame from camera")
    finally:
        if camera and camera.isOpened():
            camera.release()

if __name__ == "__main__":
    capture_image()

