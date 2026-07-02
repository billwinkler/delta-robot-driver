import numpy as np
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
        # Configure preview for initial AE/AWB settling
        preview_config = picam.create_preview_configuration(main={"size": (1296, 972), "format": "YUV420"})
        picam.configure(preview_config)
        picam.start()
        
        # Enable controls and settle in preview mode
        picam.set_controls({"AeEnable": True, "AwbEnable": True})
        time.sleep(5)  # Initial settling
        
        # Switch to still mode
        still_config = picam.create_still_configuration(main={"size": (2592, 1944), "format": "YUV420"})
        picam.switch_mode(still_config)
        
        # Wait for exposure to stabilize using metadata (loop until ExposureTime is reasonable and stable)
        prev_exposure = 0
        stable_count = 0
        start_time = time.time()  # Define start_time here
        while stable_count < 3:  # Require 3 consecutive stable frames
            metadata = picam.capture_metadata()
            current_exposure = metadata.get("ExposureTime", 0)
            if abs(current_exposure - prev_exposure) < 0.1 * prev_exposure and current_exposure > 40000:  # Stable if <10% change, >40ms from logs
                stable_count += 1
            else:
                stable_count = 0
            prev_exposure = current_exposure
            picam.capture_array()  # Discard frame
            time.sleep(0.1)  # ~100ms per check
            if time.time() - start_time > 60:
                break  # Timeout if not stabilizing
        
        # Capture the stabilized frame
        frame_yuv = picam.capture_array()
        
        # Trim to exact dimensions to remove padding artifact (green line)
        height, width = 1944, 2592
        frame_yuv = frame_yuv[:int(height * 1.5), :width]
        
        # Convert YUV420 (I420) to BGR
        frame_bgr = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV2BGR_I420)
        
        # Save
        cv2.imwrite(output_path, frame_bgr)
        print(f"Image saved to {output_path}")

    finally:
        picam.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture an image from the camera.")
    parser.add_argument("output_path", help="The full path to save the captured image.")
    args = parser.parse_args()
    
    capture_image(args.output_path)
