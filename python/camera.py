from picamera2 import Picamera2
import time
import argparse
import cv2
import numpy as np

picam2 = Picamera2()

parser = argparse.ArgumentParser(description="Capture an image for CNN training data.")
parser.add_argument("output_path", help="The full path to save the captured image.")
parser.add_argument("--width", type=int, default=320, help="Image width for resizing (default: 320 for efficiency).")
parser.add_argument("--height", type=int, default=240, help="Image height for resizing (default: 240 for efficiency).")
parser.add_argument("--grayscale", action="store_true", default=True, help="Capture in grayscale for training efficiency (default: False).")
args = parser.parse_args()

# Use still configuration for high-quality full-resolution capture initially
config = picam2.create_still_configuration(main={"size": (2592, 1944)})  # Full sensor for max FOV
picam2.configure(config)

picam2.start()

# Set controls for consistent, high-quality images suitable for CNN (balanced, not over-processed)
picam2.set_controls({
    "AeEnable": True,         # Auto Exposure for varying lighting
    "AwbEnable": True,        # Auto White Balance for color consistency (even if grayscale, helps initial capture)
    "NoiseReductionMode": 1,  # Minimal noise reduction to preserve details for learning
    "Sharpness": 0.5,         # Reduced sharpness to avoid artifacts in training
    "Contrast": 1.0,          # Standard contrast
    "Saturation": 1.0,        # Neutral saturation (no boost for accurate representation)
    "ScalerCrop": (0, 0, 2592, 1944),  # Full frame for complete context/FOV
    "ColourGains": (1.0, 1.0) # Neutral color gains (avoid bias in dataset)
})

time.sleep(5)  # Settling time for stable exposure/WB

# Capture raw array for post-processing (resize/grayscale)
array = picam2.capture_array("main")  # RGB array from full capture

# Resize to small dimensions for efficient training
resized = cv2.resize(array, (args.width, args.height))

if args.grayscale:
    # Convert to grayscale (1 channel, reduces data size/compute)
    gray = cv2.cvtColor(resized, cv2.COLOR_RGB2GRAY)
    cv2.imwrite(args.output_path, gray)  # Save as grayscale JPEG
else:
    # Save as RGB JPEG
    bgr = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)
    cv2.imwrite(args.output_path, bgr)

picam2.stop()
print(f"Image saved to {args.output_path} (size: {args.width}x{args.height}, grayscale: {args.grayscale})")
