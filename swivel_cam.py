#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10,<3.13"
# dependencies = ["mediapipe>=0.10.14", "opencv-python-headless>=4.9", "numpy"]
# ///
"""Swivel webcam helper: head pose from a webcam via MediaPipe Face Landmarker.

Streams UDP packets to Swivel (127.0.0.1:4243 by default): 7 little-endian
doubles = x, y, z (cm, head position), yaw, pitch, roll (degrees), valid (1/0).
Video never leaves this process; only the angles are sent.
"""
import argparse
import math
import socket
import struct
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import mediapipe as mp

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/face_landmarker/"
             "face_landmarker/float16/1/face_landmarker.task")


def model_path():
    cache = Path.home() / ".cache/swivel"
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / "face_landmarker.task"
    if not path.exists():
        print("downloading face model…", file=sys.stderr, flush=True)
        tmp = path.with_suffix(".part")
        urllib.request.urlretrieve(MODEL_URL, tmp)
        tmp.rename(path)
    return str(path)


class OneEuro:
    """Adaptive low-pass: steady when still, responsive when moving (units: degrees)."""

    def __init__(self, min_cutoff=0.8, beta=0.02, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x = self.dx = None

    @staticmethod
    def _a(cutoff, dt):
        return 1.0 / (1.0 + 1.0 / (2 * math.pi * cutoff * dt))

    def __call__(self, x, dt):
        if self.x is None or dt <= 0:
            self.x, self.dx = x, 0.0
            return x
        dx = (x - self.x) / dt
        self.dx += self._a(self.d_cutoff, dt) * (dx - self.dx)
        self.x += self._a(self.min_cutoff + self.beta * abs(self.dx), dt) * (x - self.x)
        return self.x


def angles(m):
    """Yaw, pitch, roll (degrees) from MediaPipe's 4x4 facial transformation matrix."""
    yaw = math.degrees(math.atan2(m[0][2], m[2][2]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -m[1][2]))))
    roll = math.degrees(math.atan2(m[1][0], m[1][1]))
    return yaw, pitch, roll


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--port", type=int, default=4243)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--raw", action="store_true", help="disable smoothing")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # 720p at 30 fps over USB
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        sys.exit(f"cannot open {a.device}")

    vision = mp.tasks.vision
    opts = vision.FaceLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=model_path()),
        running_mode=vision.RunningMode.VIDEO,
        num_faces=1,
        output_facial_transformation_matrixes=True,
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", a.port)
    print(f"webcam {a.device} streaming to 127.0.0.1:{a.port}", file=sys.stderr, flush=True)
    t0 = time.monotonic()
    last_ts = -1
    filters = [OneEuro() for _ in range(3)]
    last_t = None
    with vision.FaceLandmarker.create_from_options(opts) as lm:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            ts = int((time.monotonic() - t0) * 1000)
            if ts <= last_ts:
                ts = last_ts + 1
            last_ts = ts
            img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            res = lm.detect_for_video(img, ts)
            if res.facial_transformation_matrixes:
                m = res.facial_transformation_matrixes[0]
                ang = angles(m)
                now = time.monotonic()
                dt = now - last_t if last_t else 1 / 30
                last_t = now
                if not a.raw:
                    ang = [f(v, dt) for f, v in zip(filters, ang)]
                pkt = (m[0][3], m[1][3], m[2][3], *ang, 1.0)
            else:
                pkt = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                filters = [OneEuro() for _ in range(3)]
                last_t = None
            sock.sendto(struct.pack("<7d", *pkt), dest)
            if a.verbose:
                print("yaw %+6.1f pitch %+6.1f roll %+6.1f face %d" % (*pkt[3:6], pkt[6]), flush=True)


if __name__ == "__main__":
    main()
