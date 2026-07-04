"""Scene-camera vision for the grip-search harness.

Captures a locked-exposure frame from the Arducam OV9281 (UVC) and
locates the laser-cross intersection and the pecan on the platform.

Usage:  .venv/bin/python3 scene_vision.py [--frame PATH] [--save PATH]
Prints one JSON line: {"cross": [x, y] | null, "pecan": [x, y] | null}

Calibration notes (2026-07-04):
- Exposure must be locked (auto_exposure=1, exposure_time_absolute=25);
  auto-exposure starts from black and needs >>15 frames to converge.
- The platform ROI keeps clipped laser lines and off-platform clutter
  from skewing the line fit.
- The pecan is a compact dark blob on the bright paper; the gripper
  jaws are also dark but form a large region touching the ROI top.
"""
import glob
import json
import subprocess
import sys
import tempfile

import cv2
import numpy as np

# platform region in scene pixels [y0, y1, x0, x1]
ROI = (360, 740, 450, 1240)
EXPOSURE = 25
PECAN_AREA = (500, 5000)
PECAN_MAX_DIM = 110
EDGE_MARGIN = 55


def device():
    paths = glob.glob("/dev/v4l/by-id/usb-*UC762*-video-index0")
    if not paths:
        raise RuntimeError("OV9281 not found under /dev/v4l/by-id")
    return paths[0]


def capture(save_path):
    dev = device()
    subprocess.run(
        ["v4l2-ctl", "-d", dev, "--set-ctrl=auto_exposure=1",
         f"--set-ctrl=exposure_time_absolute={EXPOSURE}"],
        check=True, capture_output=True)
    subprocess.run(
        ["v4l2-ctl", "-d", dev,
         "--set-fmt-video=width=1280,height=800,pixelformat=MJPG",
         "--stream-mmap", "--stream-count=1", "--stream-skip=5",
         f"--stream-to={save_path}"],
        check=True, capture_output=True)
    return save_path


def _line_from_points(pts):
    pts = np.array(pts, dtype=float)
    mean = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - mean)
    return mean, vt[0]


def _intersect(p1, d1, p2, d2):
    a = np.array([d1, -d2]).T
    b = p2 - p1
    t, _ = np.linalg.lstsq(a, b, rcond=None)[0]
    return p1 + t * d1


def find_cross(gray):
    y0, y1, x0, x1 = ROI
    mask = np.zeros_like(gray)
    mask[y0:y1, x0:x1] = 255
    roi = cv2.bitwise_and(gray, mask)
    _, thr = cv2.threshold(roi, 215, 255, cv2.THRESH_BINARY)
    thr = cv2.dilate(thr, np.ones((3, 3), np.uint8))
    segs = cv2.HoughLinesP(thr, 1, np.pi / 180, threshold=60,
                           minLineLength=80, maxLineGap=25)
    if segs is None:
        return None
    angles = []
    for s in segs[:, 0]:
        sx1, sy1, sx2, sy2 = s
        angles.append(np.arctan2(sy2 - sy1, sx2 - sx1) % np.pi)
    angles = np.array(angles)
    a0 = angles[0]
    diff = np.abs(angles - a0)
    c0 = np.minimum(diff, np.pi - diff) < np.pi / 8
    if c0.all() or (~c0).sum() < 1:
        return None
    pts0, pts1 = [], []
    for s, in0 in zip(segs[:, 0], c0):
        sx1, sy1, sx2, sy2 = s
        (pts0 if in0 else pts1).extend([(sx1, sy1), (sx2, sy2)])
    p1, d1 = _line_from_points(pts0)
    p2, d2 = _line_from_points(pts1)
    xy = _intersect(p1, d1, p2, d2)
    if not (x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1):
        return None
    return [round(float(xy[0]), 1), round(float(xy[1]), 1)]


def find_pecan(gray):
    y0, y1, x0, x1 = ROI
    region = gray[y0:y1, x0:x1]
    _, dark = cv2.threshold(region, 100, 255, cv2.THRESH_BINARY_INV)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0
    for c in cnts:
        area = cv2.contourArea(c)
        if not (PECAN_AREA[0] <= area <= PECAN_AREA[1]):
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        if bw > PECAN_MAX_DIM or bh > PECAN_MAX_DIM:
            continue
        if by <= 2:  # touches ROI top edge -> jaw/arm shadow, not pecan
            continue
        m = cv2.moments(c)
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        # platform corner tape triangles hug the ROI edges; real pecans
        # (and all deposit spots) are interior
        if (cx < EDGE_MARGIN or cy < EDGE_MARGIN
                or (x1 - x0) - cx < EDGE_MARGIN
                or (y1 - y0) - cy < EDGE_MARGIN):
            continue
        if area > best_area:
            best = [round(cx + x0, 1), round(cy + y0, 1)]
            best_area = area
    return best


def main():
    args = sys.argv[1:]
    frame = None
    save = None
    if "--frame" in args:
        frame = args[args.index("--frame") + 1]
    if "--save" in args:
        save = args[args.index("--save") + 1]
    if frame is None:
        frame = save or tempfile.mktemp(suffix=".jpg", prefix="scene-")
        capture(frame)
    gray = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        print(json.dumps({"error": f"unreadable frame {frame}"}))
        return 1
    print(json.dumps({"cross": find_cross(gray),
                      "pecan": find_pecan(gray),
                      "frame": frame}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
