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


MIN_JAW_HEIGHT = 60   # px: jaw fingers are tall; pecans are not
TOP_ATTACH = 12       # jaw component must start this close to ROI top
MIN_FINGER_WIDTH = 8  # px: ignore skinny noise columns
JAW_SIDE_MARGIN = 40  # px: the perforated strut enters from the sides
JAW_PAIR_DX = (20, 150)   # tip separation range, px
JAW_PAIR_DY = 60          # max tip height difference, px


def find_jaw_tips(gray):
    """Locate the two gripper jaw tips and their gap center.

    The jaws are the dark components that hang from the ROI top edge
    (exactly the property find_pecan uses to reject them). Their tips
    are the two deepest 'fingers' of those components' bottom profile.
    At grip depth the tips sit on the platform plane — the same plane
    as the pecan — so gap-center vs pecan is a parallax-free error.

    Validated 2026-07-04 on the grip-attempt archive: 126/132 open-jaw
    servo frames detected; gap center within ~1 px of the hand-measured
    calibration ((941.5,515.5) vs (942,500) manual). Closed-jaw frames
    often return None (tips merge with each other/the pecan) — callers
    must treat None as 'jaws not measurable', not as an error.

    Returns (tips, gap_center) or (None, None).
    """
    y0, y1, x0, x1 = ROI
    region = gray[y0:y1, x0:x1]
    _, dark = cv2.threshold(region, 100, 255, cv2.THRESH_BINARY_INV)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    n, labels, stats, _ = cv2.connectedComponentsWithStats(dark)
    jaw_mask = np.zeros_like(dark)
    for i in range(1, n):
        if (stats[i, cv2.CC_STAT_TOP] <= TOP_ATTACH
                and stats[i, cv2.CC_STAT_HEIGHT] >= MIN_JAW_HEIGHT):
            jaw_mask[labels == i] = 255
    if not jaw_mask.any():
        return None, None

    # bottom profile: per column, the lowest jaw pixel (-1 = none)
    hgt, wid = jaw_mask.shape
    ys = np.arange(hgt).reshape(-1, 1)
    prof = np.where(jaw_mask > 0, ys, -1).max(axis=0)

    # contiguous runs of jaw presence
    runs, start = [], None
    for x in range(wid):
        if prof[x] >= 0 and start is None:
            start = x
        elif prof[x] < 0 and start is not None:
            runs.append((start, x))
            start = None
    if start is not None:
        runs.append((start, wid))
    runs = [r for r in runs if r[1] - r[0] >= MIN_FINGER_WIDTH]
    if not runs:
        return None, None

    # finger tips = local maxima of the bottom profile
    tips = []
    for (a, b) in runs:
        seg = prof[a:b].astype(float)
        smooth = np.convolve(seg, np.ones(9) / 9, mode="same")
        w = 15
        for x in range(len(seg)):
            lo, hi = max(0, x - w), min(len(seg), x + w + 1)
            if (smooth[x] >= smooth[lo:hi].max() - 0.5
                    and seg[x] == seg[lo:hi].max()):
                tips.append((a + x, int(seg[x])))
    tips.sort()
    merged = []
    for t in tips:
        if merged and t[0] - merged[-1][0] < 30:
            if t[1] > merged[-1][1]:
                merged[-1] = t
        else:
            merged.append(t)
    # a "tip" at the ROI bottom is a shadow column; one hugging a side
    # edge is the perforated strut — neither is a jaw
    merged = [t for t in merged
              if t[1] < (y1 - y0) - 5
              and JAW_SIDE_MARGIN <= t[0] <= (x1 - x0) - JAW_SIDE_MARGIN]
    if len(merged) < 2:
        return None, None
    # deepest valid PAIR: plausibly separated, similar height
    merged.sort(key=lambda t: -t[1])
    best = None
    for i in range(len(merged)):
        for j in range(i + 1, len(merged)):
            a, b = sorted([merged[i], merged[j]])
            if (JAW_PAIR_DX[0] <= b[0] - a[0] <= JAW_PAIR_DX[1]
                    and abs(a[1] - b[1]) <= JAW_PAIR_DY):
                best = [a, b]
                break
        if best:
            break
    if not best:
        return None, None

    tips_abs = [[round(float(tx + x0), 1), round(float(ty + y0), 1)]
                for tx, ty in best]
    gap = [round((tips_abs[0][0] + tips_abs[1][0]) / 2, 1),
           round((tips_abs[0][1] + tips_abs[1][1]) / 2, 1)]
    return tips_abs, gap


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
    jaws, gap = find_jaw_tips(gray)
    print(json.dumps({"cross": find_cross(gray),
                      "pecan": find_pecan(gray),
                      "jaws": jaws,
                      "gap-center": gap,
                      "frame": frame}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
