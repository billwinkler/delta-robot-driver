from picamera2 import Picamera2
import cv2
import numpy as np
import time

# Initialize the camera
picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={"size": (640, 480)}))
# Set manual exposure and white balance to prevent auto-adjustments
# picam.set_controls({"ExposureTime": 50000, "AnalogueGain": 1.0, "AwbEnable": True, "ColourGains": (1.0, 1.0)})
picam.start()

# Add delay to allow camera stabilization
time.sleep(1)  # 1-second delay

# Get the full sensor resolution
full_res = picam.camera_properties['PixelArraySize']  # e.g., (3280, 2464) for Camera Module v2
print(f"Full sensor resolution: {full_res}")

# Explicitly set ScalerCrop to full (though it's the default)
picam.set_controls({"ScalerCrop": (0, 0, 2592, 1944)})

# Capture a single frame
frame = picam.capture_array()
picam.stop()

# Convert RGB to BGR for OpenCV
frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

# Save original image for debugging
cv2.imwrite("original_frame.jpg", frame)

# Blur to reduce noise
frame = cv2.GaussianBlur(frame, (3, 3), 0)

# Convert to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
cv2.imwrite("hsv_image.jpg", hsv)

# Debug: Print pecan's HSV
print(f"Pecan HSV at (311, 234): {hsv[234, 311]}") #light brown
print(f"Pecan HSV at (255, 225): {hsv[225, 255]}") #dark brown

# It found a pecan with these values
# Pecan HSV at (311, 234): [ 20  49 220]
# Pecan HSV at (255, 225): [ 81  30 171]

# Mask for white areas (table)
lower_white = np.array([0, 0, 40])
upper_white = np.array([180, 200, 255])
white_mask = cv2.inRange(hsv, lower_white, upper_white)

# Brown detection
lower_brown = np.array([10, 30, 120])
upper_brown = np.array([80, 255, 255])
brown_mask = cv2.inRange(hsv, lower_brown, upper_brown)

# Combine masks
mask = cv2.bitwise_and(brown_mask, white_mask)

# Noise reduction
mask = cv2.erode(mask, None, iterations=6)
mask = cv2.dilate(mask, None, iterations=6)

# Save masks for debugging
cv2.imwrite("white_mask.jpg", white_mask)
cv2.imwrite("brown_mask.jpg", brown_mask)
cv2.imwrite("mask.jpg", mask)

result = cv2.bitwise_and(frame, frame, mask=brown_mask)
cv2.imwrite("brown_result.jpg", result)

# Find contours
# Find contours
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f"Found {len(contours)} contours")
for i, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / h
    center_x, center_y = x + w // 2, y + h // 2
    print(f"Contour {i}: area={area}, aspect_ratio={aspect_ratio}, center=({center_x}, {center_y}), w={w}, h={h}")

if contours:
    best_contour = None
    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        center_x = x + w // 2
        center_y = y + h // 2
        aspect_ratio = float(w) / h
        if 200 < area < 6000 and 0.8 < aspect_ratio < 2.5 and white_mask[center_y, center_x] > 0:
            valid_contours.append(contour)
            print(f"Possible pecan at ({center_x}, {center_y}) with area {area}, aspect_ratio {aspect_ratio}")
    if valid_contours:
        best_contour = max(valid_contours, key=cv2.contourArea)  # Select largest valid contour
    if best_contour is not None:
        M = cv2.moments(best_contour)
        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])
        x, y, w, h = cv2.boundingRect(best_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        area = cv2.contourArea(best_contour)
        aspect_ratio = float(w) / h
        print(f"Selected pecan: area={area}, aspect_ratio={aspect_ratio}, center=({center_x}, {center_y})")
        print(f"Pecan found at coordinates: ({center_x}, {center_y}) for robot effector repositioning")
    else:
        print("No pecan detected (no suitable contours)")
else:
    print("No contours detected")

# Save and display
cv2.imwrite("output.jpg", frame)
#cv2.imshow("Result", frame)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
