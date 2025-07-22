from picamera2 import Picamera2
import time

picam2 = Picamera2()

config = picam2.create_still_configuration()
picam2.configure(config)

picam2.start()

picam2.set_controls({
    "AeEnable": True,         # Enable Auto Exposure (ensures proper brightness)
    "AwbEnable": True,        # Enable Auto White Balance (ensures proper colors)
    "NoiseReductionMode": 3,  # High-quality noise reduction (matches ISP processing)
    "Sharpness": 1.0,         # Match rpicam-still sharpness
    "Contrast": 1.0,          # Prevent washed-out image
    "Saturation": 1.2,        # Slight color boost (adjust if needed)
    "ScalerCrop": (0, 0, 2592, 1944),  # Ensure full frame is used
    "ColourGains": (1.5, 1.5) # Try matching CFE color correction
})

time.sleep(5)

picam2.capture_file("out_test.jpg")
picam2.stop()
