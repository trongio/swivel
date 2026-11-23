#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10,<3.13"
# dependencies = ["mediapipe>=0.10.14", "opencv-python-headless>=4.9", "numpy", "onnxruntime>=1.17"]
# ///
"""Swivel webcam helper: head pose from a webcam.

Backends: "opentrack" (default) uses OpenTrack's neural-net head pose models
(head localizer + head-pose-0.5-big, trained for head tracking, robust at large
angles); "mediapipe" uses MediaPipe Face Landmarker.

Streams UDP packets to Swivel (127.0.0.1:4243 by default): 7 little-endian
doubles = x, y, z (cm, head position), yaw, pitch, roll (degrees), valid (1/0).
With --preview PORT, a small JPEG of the view with the tracked head box is also sent to
127.0.0.1:PORT about 10 times a second for Swivel's preview. Nothing leaves this computer.
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
import numpy as np

OT_BASE = "https://raw.githubusercontent.com/opentrack/opentrack/master/tracker-neuralnet/models/"
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/face_landmarker/"
             "face_landmarker/float16/1/face_landmarker.task")


def fetch(name, url):
    cache = Path.home() / ".cache/swivel"
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / name
    if not path.exists():
        print(f"downloading {name}…", file=sys.stderr, flush=True)
        tmp = path.with_suffix(".part")
        urllib.request.urlretrieve(url, tmp)
        tmp.rename(path)
    return str(path)


def model_path():
    return fetch("face_landmarker.task", MODEL_URL)


class OpenTrackNN:
    """OpenTrack's neural-net head tracker (localizer + pose estimator), ported from
    tracker-neuralnet/model_adapters.cpp."""

    def __init__(self):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 4
        prov = ["CPUExecutionProvider"]
        self.loc = ort.InferenceSession(fetch("head-localizer.onnx", OT_BASE + "head-localizer.onnx"),
                                        opts, providers=prov)
        self.pose = ort.InferenceSession(fetch("head-pose-0.5-big.onnx", OT_BASE + "head-pose-0.5-big.onnx"),
                                         opts, providers=prov)
        self.out_names = [o.name for o in self.pose.get_outputs()]
        self.box = None
        self.fov = 95.0  # diagonal field of view at zoom 1 (Anker PowerConf C200 widest)
        self.zoom = 1.0

    def localize(self, gray):
        h, w = gray.shape
        x = cv2.resize(gray, (288, 224), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0 - 0.5
        r = self.loc.run(["logit_box"], {"x": x[None, None]})[0][0]
        if 1 / (1 + math.exp(-r[0])) < 0.5:
            return None
        un = lambda v: 0.5 * (v + 1)  # noqa: E731
        x0, y0, x1, y1 = un(r[1]) * w, un(r[2]) * h, un(r[3]) * w, un(r[4]) * h
        return (x0, y0, x1 - x0, y1 - y0)

    @staticmethod
    def normalize_brightness(img):
        b = int(np.percentile(img, 90))
        alpha = (0.9 * 0.5 / max(5, b)) if b < 127 else 1.0 / 255
        return img.astype(np.float32) * alpha - 0.5

    def __call__(self, frame):
        """Returns (yaw, pitch, roll) in degrees, or None. Image axes: x right, y down."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.box is None:
            self.box = self.localize(gray)
            if self.box is None:
                return None
        bx, by, bw, bh = self.box
        size = int(max(bw, bh))
        if size < 24:
            self.box = None
            return None
        cx = min(max(bx + 0.5 * bw, 0.0), gray.shape[1])
        cy = min(max(by + 0.5 * bh, 0.0), gray.shape[0])
        patch = cv2.getRectSubPix(gray, (size, size), (cx, cy))
        x = self.normalize_brightness(cv2.resize(patch, (129, 129), interpolation=cv2.INTER_AREA))
        # run the original and the mirrored crop in one batch, average the two estimates
        batch = np.stack([x, x[:, ::-1]])[:, None].copy()
        try:
            res = self.pose.run(self.out_names, {"x": batch})
            outs = [dict(zip(self.out_names, [r[i:i + 1] for r in res])) for i in (0, 1)]
        except Exception:  # model without a dynamic batch axis: run twice
            outs = [dict(zip(self.out_names, self.pose.run(self.out_names, {"x": v[None, None].copy()})))
                    for v in (x, x[:, ::-1])]
        out = outs[0]
        q = out["quat"][0]  # x, y, z, w
        ob = out["box"][0]
        half = 0.5 * size
        nb = (cx + half * ob[0], cy + half * ob[1], half * (ob[2] - ob[0]), half * (ob[3] - ob[1]))
        # rotation uncertainty: large -> lost the head, relocalize next frame
        unc = float(np.abs(out["rotaxis_scales_tril"][0]).max()) if "rotaxis_scales_tril" in out else 0.0
        if nb[2] > 10 and nb[3] > 10 and unc < 0.5:
            # smooth the crop box: a box that jumps a few pixels per frame shows up as pose jitter
            k = 0.35
            self.box = tuple(o + k * (n - o) for o, n in zip(self.box, nb)) if self.box else nb
        else:
            self.box = None
        w_, x_, y_, z_ = float(q[3]), float(q[0]), float(q[1]), float(q[2])
        if w_ < 0:
            w_, x_, y_, z_ = -w_, -x_, -y_, -z_
        n = math.sqrt(x_ * x_ + y_ * y_ + z_ * z_)
        if n < 1e-9:
            return 0.0, 0.0, 0.0
        ang = math.degrees(2 * math.atan2(n, w_))
        rx, ry, rz = x_ / n * ang, y_ / n * ang, z_ / n * ang
        mx, my, mz = self.rotvec(outs[1]["quat"][0])
        # mirroring negates rotation about the vertical (y) and viewing (z) axes
        rx, ry, rz = (rx + mx) / 2, (ry - my) / 2, (rz - mz) / 2
        return self.line_of_sight(rx, ry, rz, cx, cy, gray.shape[1], gray.shape[0])

    def line_of_sight(self, rx, ry, rz, x, y, w, h):
        """The net reads the head's rotation relative to the line from the camera to the head,
        so leaning or sliding sideways looks like turning. Rotate by that line's angle off the
        camera axis (as OpenTrack does) so only real head turns remain."""
        a = w / h
        t = math.tan(math.radians(self.fov) / 2) / self.zoom
        tw, th = t / math.sqrt(1 + 1 / (a * a)), t / math.sqrt(1 + a * a)
        ray = ((x / w * 2 - 1) * tw, (y / h * 2 - 1) * th, 1.0)
        ax, ay = -ray[1], ray[0]  # (0, 0, 1) x ray
        s_ = math.hypot(ax, ay)
        ang = math.atan2(s_, 1.0)
        qc = (math.cos(ang / 2), *(v / s_ * math.sin(ang / 2) if s_ > 1e-9 else 0.0 for v in (ax, ay, 0.0)))
        n = math.sqrt(rx * rx + ry * ry + rz * rz)
        th_ = math.radians(n)
        qm = (math.cos(th_ / 2), *((v / n * math.sin(th_ / 2)) if n > 1e-9 else 0.0 for v in (rx, ry, rz)))
        w1, x1, y1, z1 = qc
        w2, x2, y2, z2 = qm
        q = (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
             w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
        return nose_angles(q)


    @staticmethod
    def rotvec(q):
        w_, x_, y_, z_ = float(q[3]), float(q[0]), float(q[1]), float(q[2])
        if w_ < 0:
            w_, x_, y_, z_ = -w_, -x_, -y_, -z_
        n = math.sqrt(x_ * x_ + y_ * y_ + z_ * z_)
        if n < 1e-9:
            return 0.0, 0.0, 0.0
        ang = math.degrees(2 * math.atan2(n, w_))
        return x_ / n * ang, y_ / n * ang, z_ / n * ang


def nose_angles(q):
    """(yaw, pitch, roll) in degrees from where the nose points. Unlike rotation-vector parts,
    yaw and pitch stay put when the head tilts sideways."""
    w, x, y, z = q
    fx, fy, fz = -2 * (x * z + w * y), -2 * (y * z - w * x), -(1 - 2 * (x * x + y * y))
    yaw = math.degrees(math.atan2(-fx, -fz))
    pitch = math.degrees(math.atan2(fy, math.hypot(fx, fz)))
    # roll: the head's own sideways tilt, only shown, never used
    roll = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return yaw, pitch, roll


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
    ap.add_argument("--backend", choices=["opentrack", "mediapipe"], default="opentrack")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--fov", type=float, default=95.0, help="diagonal field of view at zoom 1 (degrees)")
    ap.add_argument("--preview", type=int, default=0, help="send a JPEG preview to this local UDP port")
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # 720p at 30 fps over USB
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        sys.exit(f"cannot open {a.device}")

    if a.backend == "opentrack":
        return run_opentrack(cap, a)

    import mediapipe as mp
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


def run_opentrack(cap, a):
    nn = OpenTrackNN()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", a.port)
    print(f"webcam {a.device} (opentrack nn) streaming to 127.0.0.1:{a.port}", file=sys.stderr, flush=True)
    filters = [OneEuro() for _ in range(3)]
    last_t = None
    prev_t = 0.0
    nn.fov = a.fov
    zoom_t = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        if time.monotonic() - zoom_t > 1.0:  # Swivel can change the zoom while we run
            zoom_t = time.monotonic()
            z = cap.get(cv2.CAP_PROP_ZOOM)
            nn.zoom = z / 100 if z and z >= 100 else 1.0
        ang = nn(frame)
        if ang is not None:
            now = time.monotonic()
            dt = now - last_t if last_t else 1 / 30
            last_t = now
            if not a.raw:
                ang = [f(v, dt) for f, v in zip(filters, ang)]
            pkt = (0.0, 0.0, 0.0, *ang, 1.0)
        else:
            pkt = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            filters = [OneEuro() for _ in range(3)]
            last_t = None
        sock.sendto(struct.pack("<7d", *pkt), dest)
        if a.preview and time.monotonic() - prev_t > 0.1:
            prev_t = time.monotonic()
            send_preview(sock, a.preview, frame, nn.box, ang)
        if a.verbose:
            print("yaw %+6.1f pitch %+6.1f roll %+6.1f face %d" % (*pkt[3:6], pkt[6]), flush=True)


def send_preview(sock, port, frame, box, ang):
    """Mirrored thumbnail with the head box, as one JPEG datagram."""
    h, w = frame.shape[:2]
    k = 360 / w
    img = cv2.resize(frame, (360, int(h * k)), interpolation=cv2.INTER_AREA)
    if box is not None:
        bx, by, bw, bh = (v * k for v in box)
        color = (137, 227, 87) if ang is not None else (45, 210, 246)
        cv2.rectangle(img, (int(bx), int(by)), (int(bx + bw), int(by + bh)), color, 2)
    img = cv2.flip(img, 1)  # like a mirror, so moving left moves left
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if ok and len(jpg) < 60000:
        try:
            sock.sendto(jpg.tobytes(), ("127.0.0.1", port))
        except OSError:
            pass


if __name__ == "__main__":
    main()
