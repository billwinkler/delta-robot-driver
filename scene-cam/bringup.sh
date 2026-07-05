#!/bin/bash
# Arducam OV9281 UVC bring-up: enumerate, characterize, test frame.
set -u
OUT=~/dev/delta-robot-driver/scene-cam
DEV=$(ls /dev/v4l/by-id/usb-*-video-index0 2>/dev/null | head -1)
if [ -z "$DEV" ]; then echo "NO_USB_CAMERA"; exit 1; fi
REAL=$(readlink -f "$DEV")
echo "=== device: $DEV -> $REAL"
echo "=== driver info ==="
v4l2-ctl -d "$REAL" --info
echo "=== formats ==="
v4l2-ctl -d "$REAL" --list-formats-ext
echo "=== controls ==="
v4l2-ctl -d "$REAL" --list-ctrls
echo "=== test frame (ffmpeg) ==="
ffmpeg -hide_banner -loglevel warning -f v4l2 -i "$REAL" -frames:v 1 -y "$OUT/test-frame.png" && echo "FRAME_OK $OUT/test-frame.png"
