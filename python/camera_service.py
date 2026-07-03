"""Persistent camera service for the servo loop.

The one-shot capture path (bb camera -> camera-save.py) re-initializes
the camera and re-settles auto-exposure on every call (~9s/frame). The
servo loop needs sub-second act-and-sense, so this service keeps
picamera2 alive: settle AE once at startup, lock exposure/gain, then
serve frames on demand over a tiny TCP line protocol.

Protocol (one line per request, newline-terminated):
  capture <path>\n   -> grabs a frame, saves 320x240 grayscale JPEG
                        to <path>, replies "ok <path> <ms>\n"
  ping\n             -> "pong\n"
  quit\n             -> shuts the service down

Run:  python3 camera_service.py [port]     (default 8765, localhost only)

Frames match the 2025 training dataset's geometry: 320x240 grayscale
from the full sensor field of view.
"""

import socket
import sys
import time

import cv2
import numpy as np
from picamera2 import Picamera2

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8765


def start_camera():
    picam = Picamera2()
    # Video configuration: continuous, fast frames (vs still mode's
    # per-shot conversion). 1296x972 is a full-FOV binned mode.
    config = picam.create_video_configuration(
        main={"size": (1296, 972), "format": "YUV420"}
    )
    picam.configure(config)
    picam.start()

    # This stack defaults ScalerCrop to a small center region (digital
    # zoom) — the 2025 scripts all set it to the full sensor explicitly.
    try:
        picam.set_controls({"ScalerCrop": (0, 0, 2592, 1944)})
    except Exception as e:
        print(f"ScalerCrop failed: {e}", flush=True)

    # Exposure lock is NOT possible with this venv's picamera2 against
    # the system libcamera (any exposure control — AeEnable or a direct
    # ExposureTime — trips a missing ExposureTimeMode symbol inside the
    # camera thread, uncatchable here). Run with auto-exposure, which is
    # what the 2025 dataset used; scene lighting is constant between
    # servo frames. Revisit if AE flicker shows up in detection.
    time.sleep(3)  # let AE settle before first serve
    md = picam.capture_metadata()
    print(f"camera ready (AE auto): exposure={md.get('ExposureTime')}us "
          f"gain={md.get('AnalogueGain', 0):.2f}", flush=True)
    return picam


def grab_frame(picam):
    """Returns a 320x240 grayscale frame (full field of view)."""
    yuv = picam.capture_array()
    # Y plane = grayscale, matching the 2025 dataset's processing.
    y = yuv[:972, :1296]
    return cv2.resize(y, (320, 240), interpolation=cv2.INTER_AREA)


def main():
    picam = start_camera()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", PORT))
    srv.listen(4)
    print(f"camera service listening on 127.0.0.1:{PORT}", flush=True)

    while True:
        conn, _ = srv.accept()
        with conn:
            f = conn.makefile("rw")
            for line in f:
                parts = line.strip().split(None, 1)
                if not parts:
                    continue
                cmd = parts[0]
                if cmd == "ping":
                    f.write("pong\n")
                    f.flush()
                elif cmd == "capture" and len(parts) == 2:
                    t0 = time.monotonic()
                    frame = grab_frame(picam)
                    path = parts[1]
                    cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
                    ms = int((time.monotonic() - t0) * 1000)
                    f.write(f"ok {path} {ms}\n")
                    f.flush()
                elif cmd == "quit":
                    f.write("bye\n")
                    f.flush()
                    picam.stop()
                    return
                else:
                    f.write(f"err unknown: {line.strip()}\n")
                    f.flush()


if __name__ == "__main__":
    main()
