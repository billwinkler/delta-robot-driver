from picamera2 import Picamera2
import cv2
import time
import argparse

def capture_image(output_path):
    """
    Captures a single image and saves it to the specified path.

    Args:
        output_path (str): The full path where the image will be saved.
    """
    picam = Picamera2()
    try:
        # Configure for a single, high-quality capture
        config = picam.create_still_configuration(main={"size": (640, 480)})
        picam.configure(config)
        picam.start()
        
        # Allow time for the camera to adjust to lighting
        time.sleep(1)
        
        # Capture the image data
        frame = picam.capture_array()
        
        # Convert from RGB (picamera2) to BGR (OpenCV)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Save the image to the specified file
        cv2.imwrite(output_path, frame_bgr)
        print(f"Image saved to {output_path}")

    finally:
        picam.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture an image from the camera.")
    parser.add_argument("output_path", help="The full path to save the captured image.")
    args = parser.parse_args()
    
    capture_image(args.output_path)
