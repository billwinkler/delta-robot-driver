"""Offline pecan detector for the eye-in-hand grayscale training frames.

Finds the pecan's pixel centroid in a 320x240 grayscale frame: the pecan
is a compact dark blob on a bright platform. Robust to the dataset's
lighting variation (window-blind banding) and soft shadows by using
percentile-relative thresholding plus shape filtering; the gripper jaws
(dark triangles at the top edge) are masked out.

Usage:
  python3 pecan_detector.py <image> [...]        # detect, print centroids
  python3 pecan_detector.py --eval <data-dir>    # run over labels.edn set,
                                                 # write detections.csv +
                                                 # annotated failure frames
"""

import csv
import glob
import json
import os
import re
import sys

import cv2
import numpy as np

# Regions occupied by the gripper jaws (x0, y0, x1, y1), measured
# empirically as the per-pixel max over the dataset (pixels dark in
# EVERY frame are the jaws — the camera is rigidly mounted, so they
# never move). Only these boxes are masked; a pecan at the top center
# or corners remains visible.
JAW_BOXES = [(24, 0, 126, 25), (196, 0, 300, 25)]
JAW_MASK_ROWS = 25  # used for edge-touch penalty only

# Pecan size bounds at 320x240 (long axis ~40-60 px).
MIN_AREA = 250
MAX_AREA = 6000


def detect_pecan(gray):
    """Returns {'centroid': (x, y), 'area': ..., 'score': ...} or None.

    gray: single-channel uint8 image.
    """
    h, w = gray.shape
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # The platform is bright; the pecan is far darker than any soft
    # shadow. Threshold relative to the background level so the
    # blind-banding doesn't matter.
    bg = np.percentile(blur, 75)
    thresh_val = bg * 0.55
    _, mask = cv2.threshold(blur, thresh_val, 255, cv2.THRESH_BINARY_INV)

    # Mask out the gripper jaws (boxes only, not the whole top band).
    for (x0, y0, x1, y1) in JAW_BOXES:
        mask[y0:y1, x0:x1] = 0

    # Clean up speckle and close small gaps.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if not (MIN_AREA <= area <= MAX_AREA):
            continue
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0
        if solidity < 0.65:  # the attached shadow deforms the contour
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        aspect = max(bw, bh) / max(1, min(bw, bh))
        if aspect > 3.0:
            continue
        # Blobs touching the frame edge are usually the jaws' shadow or a
        # half-visible object; keep them but penalize.
        edge_touch = x == 0 or y <= JAW_MASK_ROWS or x + bw >= w or y + bh >= h

        m = cv2.moments(c)
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]

        # Darkness inside the contour: the pecan is darker than tape or
        # shadow edges.
        blob_mask = np.zeros_like(gray)
        cv2.drawContours(blob_mask, [c], -1, 255, -1)
        mean_inside = cv2.mean(gray, mask=blob_mask)[0]
        darkness = bg - mean_inside

        score = darkness * solidity * (0.5 if edge_touch else 1.0)
        if best is None or score > best["score"]:
            best = {
                "centroid": (float(cx), float(cy)),
                "area": float(area),
                "solidity": float(solidity),
                "mean_inside": float(mean_inside),
                "score": float(score),
            }
    return best


def load_labels(data_dir):
    """Parses labels.edn (simple, flat EDN) into a list of dicts."""
    text = open(os.path.join(data_dir, "labels.edn")).read()
    entries = []
    for m in re.finditer(
        r'\{:image\s+"([^"]+)",?\s+:offset\s+\[([-\d.]+)\s+([-\d.]+)\],?\s+:home\s+\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]\}',
        text,
    ):
        entries.append(
            {
                "image": m.group(1),
                "offset": (float(m.group(2)), float(m.group(3))),
                "home": tuple(float(m.group(i)) for i in (4, 5, 6)),
            }
        )
    return entries


def evaluate(data_dir, out_dir="detector_eval"):
    labels = load_labels(data_dir)
    os.makedirs(out_dir, exist_ok=True)
    rows, misses = [], []
    for entry in labels:
        path = os.path.join(data_dir, "images", entry["image"])
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"unreadable: {path}")
            continue
        det = detect_pecan(gray)
        row = {
            "image": entry["image"],
            "offset_x": entry["offset"][0],
            "offset_y": entry["offset"][1],
        }
        if det:
            row.update(
                {
                    "px": round(det["centroid"][0], 2),
                    "py": round(det["centroid"][1], 2),
                    "area": det["area"],
                    "score": round(det["score"], 1),
                }
            )
        else:
            misses.append(entry["image"])
            # save annotated miss for review
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.imwrite(os.path.join(out_dir, "miss_" + entry["image"]), vis)
        rows.append(row)

    csv_path = os.path.join(out_dir, "detections.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["image", "offset_x", "offset_y", "px", "py", "area", "score"]
        )
        writer.writeheader()
        writer.writerows(rows)

    n = len(rows)
    hits = sum(1 for r in rows if "px" in r)
    print(json.dumps({"total": n, "hits": hits, "hit_rate": round(hits / n, 4),
                      "misses": misses}, indent=2))


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--eval":
        evaluate(args[1] if len(args) > 1 else "../pecan_training_data")
    else:
        for path in args:
            gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            det = detect_pecan(gray)
            print(path, det["centroid"] if det else None)
