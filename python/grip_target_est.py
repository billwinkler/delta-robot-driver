"""Estimate the grip-point pixel g from the latest 2025 sessions.

In the reversed-demonstration dataset the pecan sits at the grip point
while the arm is at offset o and the pecan appears at pixel p. Within a
session the fit convention is offset = M @ px + b, so the pixel where
the pecan appears at o = 0 is g = p - M^-1 @ o, per frame.
"""
import json
import re

import cv2
import numpy as np
from pecan_detector import detect_pecan

M = np.array([[0.515, -0.226], [0.232, 0.514]])
Minv = np.linalg.inv(M)

edn = open("../pecan_training_data/labels.edn").read()
entries = re.findall(r'\{:image "([^"]+)",\s*:offset \[(-?\d+) (-?\d+)\]', edn)
sessions = sorted({e[0].rsplit("_sample", 1)[0] for e in entries})
last = sessions[-3:]
gs = []
for name, ox, oy in entries:
    if name.rsplit("_sample", 1)[0] not in last:
        continue
    img = cv2.imread(f"../pecan_training_data/images/{name}", cv2.IMREAD_GRAYSCALE)
    det = detect_pecan(img)
    if not det:
        continue
    p = np.array(det["centroid"], dtype=float)
    o = np.array([float(ox), float(oy)])
    gs.append(p - Minv @ o)
gs = np.array(gs)
print(json.dumps({"sessions": last, "n": len(gs),
                  "g_median": np.median(gs, axis=0).round(1).tolist(),
                  "g_std": gs.std(axis=0).round(1).tolist()}))
