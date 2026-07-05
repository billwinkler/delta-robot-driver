"""Scene-camera pose tripwire.

Every pixel constant in scene_vision.py (PLATFORM_QUAD, the banked
Jacobian, static blobs) is bound to the camera's pose. A mount bump
silently mis-aims the grip — on 2026-07-05 the camera was found
pointing at a wall, and the fix cost a full recalibration session.
This check turns that failure mode into a 30-second session-start
test.

Bank a reference:   scene_vision capture + copy to data/pose-ref.jpg
Check (each session): .venv/bin/python3 python/check_pose.py
  -> {"moved": false, "corners": [{"dx": .., "dy": ..}, ...]}

Method: a patch around each PLATFORM_QUAD corner (the tape corners —
fixed features) is template-matched against the current frame within
a search window. Any corner displaced more than TOL px, or matching
poorly, means the camera (or the platform) moved: STOP and
recalibrate per the recovery recipe in the notes before servoing.
"""
import json
import sys

import cv2
import numpy as np

from scene_vision import PLATFORM_QUAD, capture

REF = "data/pose-ref.jpg"
PATCH = 56        # template size around each corner
SEARCH = 48       # +- search window
TOL_PX = 8        # displacement beyond this = moved
MIN_SCORE = 0.55  # normalized correlation below this = unrecognizable


def corner_shifts(ref, cur):
    out = []
    for (x, y) in PLATFORM_QUAD:
        h = PATCH // 2
        t = ref[y - h:y + h, x - h:x + h]
        y0, y1 = max(0, y - h - SEARCH), y + h + SEARCH
        x0, x1 = max(0, x - h - SEARCH), x + h + SEARCH
        win = cur[y0:y1, x0:x1]
        if t.size == 0 or win.shape[0] < PATCH or win.shape[1] < PATCH:
            out.append({"score": 0.0, "dx": None, "dy": None})
            continue
        res = cv2.matchTemplate(win, t, cv2.TM_CCOEFF_NORMED)
        _, score, _, loc = cv2.minMaxLoc(res)
        out.append({"score": round(float(score), 3),
                    "dx": int(loc[0] + x0 - (x - h)),
                    "dy": int(loc[1] + y0 - (y - h))})
    return out


def main():
    frame = None
    if "--frame" in sys.argv:
        frame = sys.argv[sys.argv.index("--frame") + 1]
    ref = cv2.imread(REF, cv2.IMREAD_GRAYSCALE)
    if ref is None:
        print(json.dumps({"error": f"no reference at {REF} — bank one "
                          "with: scene_vision capture, cp to " + REF}))
        return 1
    if frame is None:
        frame = capture("/tmp/pose-check.jpg")
    cur = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
    if cur is None:
        print(json.dumps({"error": f"unreadable frame {frame}"}))
        return 1
    corners = corner_shifts(ref, cur)
    # a low-texture corner patch (plain plywood) correlates poorly at
    # ANY position — treat it as "no signal", not as movement. Three
    # well-matched corners fully determine a pose change.
    valid = [c for c in corners
             if c["dx"] is not None and c["score"] >= MIN_SCORE]
    moved = (len(valid) < 3
             or any(abs(c["dx"]) > TOL_PX or abs(c["dy"]) > TOL_PX
                    for c in valid))
    print(json.dumps({"moved": moved, "valid-corners": len(valid),
                      "corners": corners}))
    return 2 if moved else 0


if __name__ == "__main__":
    sys.exit(main())
