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

# Platform mask: a QUADRILATERAL, not a rect — the re-aimed camera
# (2026-07-05) sees the platform rotated in frame, and any bounding
# rect swallows the strut/pen/clutter beside it. Corners measured
# from the banked reference frame; POSE-BOUND like every pixel
# constant here (see the tripwire check).
PLATFORM_QUAD = np.array([(699, 263), (928, 481), (673, 626), (472, 350)],
                         dtype=np.int32)   # elevated-oblique mount, 2026-07-07
                                           # (tape centroids, thresh 100 on the
                                           # post-home frame; top corner only
                                           # visible with the arm at home)
EXPOSURE = 25        # starting guess; capture() adapts to ambient light
LASER_EXPOSURE = 2   # laser-isolation frame (exp=1 starves the lines:
                     # in-quad max 148 vs saturated at exp=2)
TARGET_MEAN = 150    # adaptive-exposure target for the normal frame
MEAN_TOL = 30        # tight band: at mean ~200 the pecan's pixels rise
                     # above the dark threshold and detection fails
PECAN_AREA = (350, 5000)  # floor lowered 2026-07-05: a laser stripe
                          # across the pecan + bright exposure can
                          # shrink the dark blob (nominal ~900 at the
                          # rigid-mount scale, measured 540 split)
PECAN_MAX_DIM = 110
EDGE_MARGIN = 25     # min distance (px) from the quad boundary for a
                     # pecan centroid — rejects the corner tapes, which
                     # sit ON the boundary
# Static dark blobs INTERIOR to the ROI that pass the pecan filters
# (scene px, measured). POSE-BOUND: re-measure whenever the camera
# moves. History (old pose, 2026-07-04): the tape corner at
# [552.6, 456.1] stole the first genuine :lifted verdict; the
# mounting hole at [541.4, 599.3] caused n6's false :assume-contact.
# Cleared 2026-07-05 after the camera re-aim; re-derived empirically
# from empty-platform frames at the new pose.
STATIC_BLOBS = []
STATIC_BLOB_R = 18


def device():
    paths = glob.glob("/dev/v4l/by-id/usb-*UC762*-video-index0")
    if not paths:
        raise RuntimeError("OV9281 not found under /dev/v4l/by-id")
    return paths[0]


def _grab(dev, exposure, save_path):
    subprocess.run(
        ["v4l2-ctl", "-d", dev, "--set-ctrl=auto_exposure=1",
         f"--set-ctrl=exposure_time_absolute={exposure}"],
        check=True, capture_output=True)
    subprocess.run(
        ["v4l2-ctl", "-d", dev,
         "--set-fmt-video=width=1280,height=800,pixelformat=MJPG",
         "--stream-mmap", "--stream-count=1", "--stream-skip=5",
         f"--stream-to={save_path}"],
        check=True, capture_output=True)
    return save_path


def capture(save_path):
    """Adaptive-exposure capture: the fixed exp=25 silently assumed
    the evening-lit hangar; daylight saturates it (2026-07-05, mean
    244/255). The sensor is linear in exposure_time_absolute, so
    scale toward TARGET_MEAN, re-grabbing up to 3 times (saturation
    clips the measured mean, so one scaling step can undershoot)."""
    dev = device()
    exp = EXPOSURE
    qmask = None
    for _ in range(6):
        _grab(dev, max(1, int(round(exp))), save_path)
        g = cv2.imread(save_path, cv2.IMREAD_GRAYSCALE)
        if g is None:
            break
        if qmask is None:
            qmask = _quad_mask(g.shape)
        # meter on the QUAD INTERIOR: the dark bench around the
        # platform dilutes the full-frame mean and leaves the paper
        # blown out (2026-07-05: frame mean 185 but in-quad 209 —
        # pecan pixels rose above the dark threshold)
        m = float(g[qmask > 0].mean())
        if abs(m - TARGET_MEAN) <= MEAN_TOL:
            break
        if m > 235:
            # saturated: the measured mean under-reports true
            # brightness, so proportional stepping stalls — cut hard
            exp = max(1.0, exp * 0.25)
        else:
            exp = min(5000.0, max(1.0, exp * TARGET_MEAN / max(m, 1.0)))
    return save_path


def capture_pair(save_path):
    """Two-exposure capture (2026-07-05 scheme): a short-exposure
    frame where only the self-luminous laser cross survives, then the
    adaptive normal frame for pecan/jaw detection. Cross detection on
    the laser frame sees a dark background — no paper reflections,
    no lighting banding, no ambient-light assumption. The laser frame
    lands next to the normal frame as <save>-laser.jpg."""
    dev = device()
    base, ext = (save_path.rsplit(".", 1) + ["jpg"])[:2]
    laser_path = f"{base}-laser.{ext}"
    _grab(dev, LASER_EXPOSURE, laser_path)
    capture(save_path)
    return save_path, laser_path


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


def _quad_mask(shape):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [PLATFORM_QUAD], 255)
    return mask


def _in_quad(x, y, margin=0.0):
    """Signed distance test: True if (x,y) is at least margin px
    inside the platform quad."""
    d = cv2.pointPolygonTest(PLATFORM_QUAD.astype(np.float32),
                             (float(x), float(y)), True)
    return d >= margin


def _cross_from_binary(thr, hough_threshold, min_line_length, max_line_gap):
    thr = cv2.dilate(thr, np.ones((3, 3), np.uint8))
    segs = cv2.HoughLinesP(thr, 1, np.pi / 180, threshold=hough_threshold,
                           minLineLength=min_line_length,
                           maxLineGap=max_line_gap)
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
    if not _in_quad(xy[0], xy[1]):
        return None
    return [round(float(xy[0]), 1), round(float(xy[1]), 1)]


def _cross_at(gray, hough_threshold, min_line_length, max_line_gap,
              bright_thr=215):
    roi = cv2.bitwise_and(gray, _quad_mask(gray.shape))
    _, thr = cv2.threshold(roi, bright_thr, 255, cv2.THRESH_BINARY)
    return _cross_from_binary(thr, hough_threshold, min_line_length,
                              max_line_gap)


def find_cross_laser(laser_gray):
    """Cross detection on the laser-isolation frame via RIDGE
    isolation, not absolute brightness.

    Exposure cannot separate the laser from sunlight — both scale
    linearly, so their ratio is fixed (2026-07-05: sunlit paper ~130,
    laser-painted line ~148 at exp=2; no threshold works). But the
    lines are narrow RIDGES: subtracting a median-blurred background
    removes slow sun gradients entirely and leaves the +20-unit
    ridge. Works in any ambient light (at night the background is
    already black and the residual is the laser unchanged).
    Validated on the failing daylight frame: intersection [525.2
    341.3] vs visible ~(510-525, 335)."""
    bg = cv2.medianBlur(laser_gray, 31)
    resid = cv2.bitwise_and(cv2.subtract(laser_gray, bg),
                            _quad_mask(laser_gray.shape))
    _, thr = cv2.threshold(resid, 12, 255, cv2.THRESH_BINARY)
    return (_cross_from_binary(thr, 60, 80, 25)
            or _cross_from_binary(thr, 40, 50, 40))


def find_cross(gray):
    """Locate the laser-cross intersection: strict first, then a
    relaxed fallback for jaw-occluded crosses.

    The strict Hough parameters are exact and reliable when both laser
    legs are unbroken. When the gripper hangs over one leg it gets
    chopped below minLineLength and the cross vanishes (live-verify
    n3/n4, 2026-07-04: one angle family found, cross plainly visible).
    The relaxed pass accepts shorter/gappier segments — noisier (its
    intersection can drift when extra segments join a cluster; archive
    sweep showed it must NOT replace the strict pass wholesale), but
    good enough for a closed loop that re-measures every iteration.
    """
    xy = _cross_at(gray, 60, 80, 25)
    if xy is not None:
        return xy
    return _cross_at(gray, 40, 50, 40)


def find_pecan(gray):
    qmask = _quad_mask(gray.shape)
    masked = cv2.bitwise_and(gray, qmask)
    # outside-quad pixels are 0 after masking, which reads as "dark";
    # paint them bright so the inverse threshold ignores them
    masked[qmask == 0] = 255
    # MEDIAN-RELATIVE dark threshold (the 2025 detector's trick): a
    # fixed 100 breaks when residual over-exposure lifts the pecan's
    # pixels — 0.62x the in-quad median tracks the paper's actual
    # brightness in any ambient light, clamped to sane bounds
    thr_val = max(60, min(115, int(0.62 * float(np.median(gray[qmask > 0])))))
    _, dark = cv2.threshold(masked, thr_val, 255, cv2.THRESH_BINARY_INV)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    # bridge the laser stripe: a pecan sitting ON the cross gets its
    # dark blob bisected by the bright line (n12 :no-target)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    best, best_area, best_contour = None, 0, None
    for c in cnts:
        area = cv2.contourArea(c)
        if not (PECAN_AREA[0] <= area <= PECAN_AREA[1]):
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        if bw > PECAN_MAX_DIM or bh > PECAN_MAX_DIM:
            continue
        m = cv2.moments(c)
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        # real pecans (and all deposit spots) are interior; the corner
        # tapes sit ON the quad boundary and fail the margin test
        if not _in_quad(cx, cy, EDGE_MARGIN):
            continue
        # a pecan's contour is FULLY interior; jaws and arm shadows
        # pierce the boundary and get clipped against it (the quad
        # replacement for the old "touches ROI top edge" jaw test)
        if any(not _in_quad(float(p[0][0]), float(p[0][1]), 3)
               for p in c[::4]):
            continue
        # known static dark blobs are never the pecan
        if any((cx - sx) ** 2 + (cy - sy) ** 2 <= STATIC_BLOB_R ** 2
               for sx, sy in STATIC_BLOBS):
            continue
        if area > best_area:
            best = [round(cx, 1), round(cy, 1)]
            best_area = area
            best_contour = c
    if best is None:
        return None, None
    # orientation: the pecan is an ellipsoid and the wristless delta
    # arm closes its jaws along a FIXED axis — the major-axis angle
    # decides graspability (2026-07-05, Bill's observation: a bump
    # that rolls the long axis into the closing direction makes the
    # grab impossible until a human repositions it)
    # major-axis angle from central moments (unambiguous, unlike
    # fitEllipse's rotation convention): theta maximizes the second
    # moment; degrees in [0, 180), image coordinates
    mm = cv2.moments(best_contour)
    angle = None
    if mm["m00"] > 0 and (mm["mu20"] != mm["mu02"] or mm["mu11"] != 0):
        theta = 0.5 * np.arctan2(2 * mm["mu11"], mm["mu20"] - mm["mu02"])
        angle = round(float(np.degrees(theta)) % 180.0, 1)
    return best, angle


MIN_JAW_HEIGHT = 60   # px: jaw fingers are tall; pecans are not
MIN_FINGER_WIDTH = 8  # px: ignore skinny noise columns
JAW_SIDE_MARGIN = 25  # px: strut rejection; 40 made the far band unconvergeable (2026-07-07)
JAW_PAIR_DX = (20, 150)   # tip separation range, px
JAW_PAIR_DY = 60          # max tip height difference, px
TIP_PAIR_VEC = (56, 15)   # left-tip -> right-tip image vector at open
                          # hover, POSE-BOUND (2026-07-07 mount: (58,13)
                          # (55,15) (57,15) (52,16) across the session's
                          # frames). Used only to reconstruct a tip the
                          # pecan occludes; verify-at-depth remains the
                          # precision stage.


def find_jaw_tips(gray, exclude=None):
    """Locate the two gripper jaw tips and their gap center.

    exclude: optional (x, y, r) disc to remove from the dark mask,
    FALLBACK-ONLY: the plain detection runs first and short-circuits,
    so every frame that detects today is byte-identical. The disc is
    for the KNOWN pecan position (unmoved since precheck): near servo
    convergence on the elevated-oblique mount the hover jaws visually
    merge with the pecan into one blob whose bottom profile is a
    single pecan-shaped finger — the two real tips flank the disc and
    reappear once it is removed (2026-07-07 reliability trial 1:
    servo died :no-gap at 18.9 px error, one iteration from tol).

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
    tips, gap = _jaw_tips_impl(gray, None)
    if exclude is None:
        return tips, gap
    ex, ey, er = exclude
    # contaminated = a "tip" is actually the pecan (trial 3: the
    # pecan paired with one real jaw tip at 85 px separation and
    # the biased gap steered the servo into an overshoot)
    trusted = ([] if tips is None else
               [t for t in tips
                if (t[0] - ex) ** 2 + (t[1] - ey) ** 2 >= er ** 2])
    contaminated = tips is not None and len(trusted) < len(tips)
    if not (tips is None or contaminated):
        return tips, gap
    tips2, gap2 = _jaw_tips_impl(gray, exclude)
    if tips2 is not None:
        return tips2, gap2
    if len(trusted) == 1:
        # the other tip is occluded BY the pecan (exclusion found
        # nothing to rescue) — reconstruct it from the stable
        # tip-to-tip image vector (trial 5: real left tip directly
        # behind the pecan at mid-approach)
        (rx, ry) = trusted[0]
        if rx >= ex:                       # trusted the RIGHT tip
            other = [rx - TIP_PAIR_VEC[0], ry - TIP_PAIR_VEC[1]]
        else:
            other = [rx + TIP_PAIR_VEC[0], ry + TIP_PAIR_VEC[1]]
        pair = sorted([[float(rx), float(ry)], other])
        gap = [round((pair[0][0] + pair[1][0]) / 2, 1),
               round((pair[0][1] + pair[1][1]) / 2, 1)]
        return [[round(p[0], 1), round(p[1], 1)] for p in pair], gap
    return None, None


def _jaw_tips_impl(gray, exclude):
    # full-frame dark components; a JAW is one that overlaps the
    # platform quad but PIERCES its boundary from outside (the quad
    # replacement for the old "hangs from the ROI top edge" test —
    # exactly the property find_pecan uses to reject them)
    _, dark = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    if exclude is not None:
        ex, ey, er = exclude
        cv2.circle(dark, (int(round(ex)), int(round(ey))),
                   int(round(er)), 0, -1)
    qmask = _quad_mask(gray.shape)

    n, labels, stats, _ = cv2.connectedComponentsWithStats(dark)
    jaw_mask = np.zeros_like(dark)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_HEIGHT] < MIN_JAW_HEIGHT:
            continue
        comp = labels == i
        inside = int(np.count_nonzero(comp & (qmask > 0)))
        outside = int(np.count_nonzero(comp & (qmask == 0)))
        if inside >= 50 and outside >= 50:      # pierces the boundary
            jaw_mask[comp] = 255
    # only the in-quad part of the jaws matters for tip finding
    jaw_mask = cv2.bitwise_and(jaw_mask, qmask)
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
    # jaw tips hang INTO the quad: a "tip" hugging the boundary is a
    # clipped shadow column or the strut, not a jaw
    merged = [t for t in merged if _in_quad(t[0], t[1], JAW_SIDE_MARGIN)]
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

    tips_abs = [[round(float(tx), 1), round(float(ty), 1)]
                for tx, ty in best]
    gap = [round((tips_abs[0][0] + tips_abs[1][0]) / 2, 1),
           round((tips_abs[0][1] + tips_abs[1][1]) / 2, 1)]
    return tips_abs, gap


def main():
    args = sys.argv[1:]
    frame = None
    save = None
    laser_frame = None
    if "--frame" in args:
        frame = args[args.index("--frame") + 1]
    if "--laser-frame" in args:
        laser_frame = args[args.index("--laser-frame") + 1]
    if "--save" in args:
        save = args[args.index("--save") + 1]
    exclude = None
    if "--exclude" in args:
        exclude = tuple(float(v) for v in
                        args[args.index("--exclude") + 1].split(","))
    if frame is None:
        frame = save or tempfile.mktemp(suffix=".jpg", prefix="scene-")
        frame, laser_frame = capture_pair(frame)
    gray = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        print(json.dumps({"error": f"unreadable frame {frame}"}))
        return 1

    # cross: prefer the laser frame (dark background, laser only).
    # The laser is the brightest thing in that frame BY CONSTRUCTION,
    # so threshold relative to its own max instead of a fixed 215 —
    # at exp=1 the lines peak ~180 and a fixed threshold finds
    # nothing (2026-07-05). Fall back to the normal frame so archive
    # frames and a failed short capture still resolve.
    cross = None
    if laser_frame is not None:
        laser_gray = cv2.imread(laser_frame, cv2.IMREAD_GRAYSCALE)
        if laser_gray is not None:
            cross = find_cross_laser(laser_gray)
    if cross is None:
        cross = find_cross(gray)

    jaws, gap = find_jaw_tips(gray, exclude)
    pecan, pecan_angle = find_pecan(gray)
    print(json.dumps({"cross": cross,
                      "pecan": pecan,
                      "pecan-angle": pecan_angle,
                      "jaws": jaws,
                      "gap-center": gap,
                      "frame": frame,
                      "laser-frame": laser_frame}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
