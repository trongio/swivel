#!/usr/bin/python3
"""Swivel: Sony headphone head tracking as a virtual gamepad's right stick.

Linux, GTK4 + libadwaita. Starts `sony-tracker` (https://github.com/kdani3/SonyTrackerLinux),
which reads the headset's orientation over Bluetooth HID and streams OpenTrack
UDP poses to 127.0.0.1:4242. Head yaw/pitch become right-stick deflection on a
virtual controller created through /dev/uinput, so any game that lets you look
around with the right stick gets head tracking, including Proton games with no
TrackIR/OpenTrack support.

Recenter from a keyboard shortcut: python3 swivel.py --recenter
"""
import collections
import fcntl
import glob
import json
import math
import os
import re
import shutil
import signal
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

import gi

gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
gi.require_version("Gsk", "4.0")
gi.require_version("Graphene", "1.0")
gi.require_version("Pango", "1.0")
from gi.repository import Adw, Gdk, GLib, Graphene, Gsk, Gtk, Pango  # noqa: E402

APP_ID = "io.github.trongio.Swivel"
PORT = 4242
HERE = Path(__file__).resolve().parent
CONF = Path(GLib.get_user_config_dir()) / "swivel/settings.json"
PIDFILE = Path(GLib.get_user_runtime_dir()) / "swivel.pid"
HID_UUID = "00001124-0000-1000-8000-00805f9b34fb"
DEFAULTS = {"yaw_max": 35.0, "pitch_max": 20.0, "deadzone": 0.5, "curve": 1.0,
            "anti_deadzone": 0.2, "smooth": 0.35, "invert_yaw": False,
            "invert_pitch": False, "pad": "playstation", "tracker": "", "drift": 0.5,
            "yaw_axis": None, "pitch_axis": None, "bias": None,
            "webcam": "off", "cam_device": "/dev/video0", "cam_sign": [1, 1],
            "ignore_roll": True}
HISTORY_SECONDS = 10.0
# Headsets send ~25 reports/s. If they go quiet this long, restart the tracker (it
# re-enables the sensor). Restarts talk to the headset over Bluetooth, so keep them
# rare: back off up to STALL_MAX_SECONDS, or a busy link can drop entirely.
STALL_SECONDS = 2.0
# Drift correction: yaw comes from a gyro with no compass, so "straight ahead" slowly
# slides. While the head is near center and fairly still, ease the center toward it.
DRIFT_WINDOW_DEG = 15.0
DRIFT_MAX_SPEED_DPS = 25.0
STALL_MAX_SECONDS = 30.0
DEMO = bool(os.environ.get("SWIVEL_DEMO"))
if DEMO:
    APP_ID += ".Demo"  # run alongside a real instance

# ---- minimal uinput (no python-evdev needed) ----------------------------------
EV_SYN, EV_KEY, EV_ABS = 0, 1, 3
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_HAT0X, ABS_HAT0Y = 0, 1, 2, 3, 4, 5, 0x10, 0x11
BUTTONS = [0x130, 0x131, 0x133, 0x134, 0x136, 0x137, 0x13A, 0x13B, 0x13C, 0x13D, 0x13E]
UI_SET_EVBIT, UI_SET_KEYBIT, UI_SET_ABSBIT = 0x40045564, 0x40045565, 0x40045567
UI_DEV_SETUP, UI_ABS_SETUP, UI_DEV_CREATE, UI_DEV_DESTROY = 0x405C5503, 0x401C5504, 0x5501, 0x5502

PADS = {
    # name, vendor, product, version, stick range, extra buttons (L2/R2 digital)
    "playstation": (b"Sony Interactive Entertainment Wireless Controller", 0x054C, 0x09CC, 0x8111,
                    (0, 255), [0x138, 0x139]),
    "xbox": (b"Microsoft X-Box 360 pad (Swivel)", 0x045E, 0x028E, 0x110, (-32768, 32767), []),
}


class VirtualPad:
    """A gamepad that only ever moves its right stick."""

    def __init__(self, kind="playstation"):
        name, vid, pid, ver, (lo, hi), extra = PADS.get(kind, PADS["playstation"])
        self.lo, self.hi = lo, hi
        self.last = None
        self.fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)
        for ev in (EV_KEY, EV_ABS):
            fcntl.ioctl(self.fd, UI_SET_EVBIT, ev)
        for b in BUTTONS + extra:
            fcntl.ioctl(self.fd, UI_SET_KEYBIT, b)
        fuzz, flat = (0, 0) if hi == 255 else (16, 128)
        center = (lo + hi + 1) // 2
        axes = {ABS_X: (lo, hi, fuzz, flat), ABS_Y: (lo, hi, fuzz, flat),
                ABS_RX: (lo, hi, fuzz, flat), ABS_RY: (lo, hi, fuzz, flat),
                ABS_Z: (0, 255, 0, 0), ABS_RZ: (0, 255, 0, 0),
                ABS_HAT0X: (-1, 1, 0, 0), ABS_HAT0Y: (-1, 1, 0, 0)}
        for code, (a_lo, a_hi, a_fuzz, a_flat) in axes.items():
            fcntl.ioctl(self.fd, UI_SET_ABSBIT, code)
            val = center if code in (ABS_X, ABS_Y, ABS_RX, ABS_RY) else 0
            fcntl.ioctl(self.fd, UI_ABS_SETUP,
                        struct.pack("<HxxiiiiII", code, val, a_lo, a_hi, a_fuzz, a_flat, 0))
        fcntl.ioctl(self.fd, UI_DEV_SETUP, struct.pack("<HHHH80sI", 3, vid, pid, ver, name, 0))
        fcntl.ioctl(self.fd, UI_DEV_CREATE)

    def _ev(self, typ, code, val):
        os.write(self.fd, struct.pack("<qqHHi", 0, 0, typ, code, val))

    def _axis(self, v, last):
        v = max(-1.0, min(1.0, v))
        f = (v + 1) / 2 * (self.hi - self.lo) + self.lo
        # Hysteresis of 0.8 step: a value hovering between two stick positions
        # (very visible on the 8-bit PlayStation stick) doesn't toggle between them.
        if last is not None and abs(f - last) < 0.8 and not (v == 0 and last != round(f)):
            return last
        return int(round(f))

    def right_stick(self, x, y):
        # Only send changes, so the game doesn't keep treating this pad as the
        # active controller (and switching button prompts) while your head is still.
        lx, ly = self.last if self.last else (None, None)
        val = (self._axis(x, lx), self._axis(-y, ly))  # head up = stick up
        if val == self.last:
            return
        self.last = val
        self._ev(EV_ABS, ABS_RX, val[0])
        self._ev(EV_ABS, ABS_RY, val[1])
        self._ev(EV_SYN, 0, 0)

    def close(self):
        try:
            fcntl.ioctl(self.fd, UI_DEV_DESTROY)
        finally:
            os.close(self.fd)


# ---- headset discovery ----------------------------------------------------------
def sony_hidraw_present():
    for f in glob.glob("/sys/class/hidraw/*/device/uevent"):
        try:
            if "HID_ID=0005:0000054C" in Path(f).read_text():
                return True
        except OSError:
            pass
    return False


def connect_hid_profiles():
    """Sony headsets often connect for audio only; ask BlueZ for the HID profile too."""
    try:
        out = subprocess.run(["bluetoothctl", "devices", "Connected"], capture_output=True,
                             text=True, timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return
    for mac in re.findall(r"Device ([0-9A-F:]{17})", out):
        try:
            info = subprocess.run(["bluetoothctl", "info", mac], capture_output=True,
                                  text=True, timeout=5).stdout
        except (OSError, subprocess.TimeoutExpired):
            continue
        if "v054C" not in info or "Human Interface Device" not in info:
            continue
        path = "/org/bluez/hci0/dev_" + mac.replace(":", "_")
        subprocess.run(["busctl", "call", "org.bluez", path, "org.bluez.Device1",
                        "ConnectProfile", "s", HID_UUID], capture_output=True, timeout=10)


def find_tracker(configured):
    for cand in (configured, os.environ.get("SONY_TRACKER"), shutil.which("sony-tracker"),
                 HERE / "SonyTrackerLinux/sony-tracker", Path.home() / "SonyTrackerLinux/sony-tracker"):
        if cand and os.access(str(cand), os.X_OK):
            return str(cand)
    return None


# ---- orientation math ------------------------------------------------------------
def q_from_euler(yaw, pitch, roll):
    """sony-tracker's ZYX euler (degrees) back to a quaternion (w, x, y, z)."""
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    cp, sp = math.cos(math.radians(pitch) / 2), math.sin(math.radians(pitch) / 2)
    cr, sr = math.cos(math.radians(roll) / 2), math.sin(math.radians(roll) / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_axis_angle(axis, deg):
    h = math.radians(deg) / 2
    return (math.cos(h), axis[0] * math.sin(h), axis[1] * math.sin(h), axis[2] * math.sin(h))


def q_rotvec(q):
    """Rotation vector in degrees (axis * angle)."""
    w, x, y, z = q if q[0] >= 0 else (-q[0], -q[1], -q[2], -q[3])
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    ang = math.degrees(2 * math.atan2(n, w))
    return (x / n * ang, y / n * ang, z / n * ang)


def v_norm(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return tuple(c / n for c in v)


def v_dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def v_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class OneEuro:
    """One Euro filter: strong smoothing when moving slowly, little lag when moving fast."""

    def __init__(self, min_cutoff=1.0, beta=1.0, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x = self.dx = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, dt):
        if self.x is None or dt <= 0:
            self.x, self.dx = x, 0.0
            return x
        dx = (x - self.x) / dt
        self.dx += self._alpha(self.d_cutoff, dt) * (dx - self.dx)
        cutoff = self.min_cutoff + self.beta * abs(self.dx)
        self.x += self._alpha(cutoff, dt) * (x - self.x)
        return self.x


CAM_PORT = 4243
CAM_STALE = 0.4  # seconds; older webcam poses are ignored
CAM_TAU = 1.5  # seconds; how fast the webcam pulls the headphone center back
CAM_MAX_SPEED_DPS = 40.0  # only correct while the head moves slower than this
CAM_MAX_ANGLE = 25.0  # only trust face tracking this close to center


def find_uv():
    for cand in (shutil.which("uv"), Path.home() / ".local/bin/uv", Path.home() / ".cargo/bin/uv"):
        if cand and os.access(str(cand), os.X_OK):
            return str(cand)
    return None


HYST = 0.6  # degrees past the deadzone before the stick engages
RAMP = 0.8  # degrees over which the game-deadzone compensation fades in
OUTPUT_HZ = 120  # stick updates per second (the headset only reports ~25/s)
GLIDE_SECONDS = 0.03  # how fast the stick glides to each new filtered target


# ---- tracking engine --------------------------------------------------------------
class Engine:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.running = False
        self.yaw = self.pitch = self.roll = 0.0
        self.sx = self.sy = 0.0
        self.center = (0.0, 0.0, 0.0)
        self.want_recenter = True
        self.last_packet = 0.0
        self.history = collections.deque(maxlen=1200)
        self.status = "Stopped"
        self.proc = self.pad = self.thread = self.sock = None
        self.errlog = None
        self.restarts = 0
        self.fail_streak = 0
        self.reanchor = False
        self.prev_pkt = None  # (time, yaw) for head speed
        self.qc = (1.0, 0.0, 0.0, 0.0)  # center orientation
        self.qr = (1.0, 0.0, 0.0, 0.0)  # current orientation relative to center
        self.calib_sink = None  # list collecting relative rotations during calibration
        self.calibrating = False
        self.target = (0.0, 0.0)
        self.filters = (OneEuro(), OneEuro())
        self.out_thread = None
        self.engaged = [False, False]
        self.cam_proc = self.cam_sock = self.cam_thread = self.cam_log = None
        self.cam = None  # (time, yaw, pitch) relative to the webcam center, signs applied
        self.cam_raw = None  # (yaw, pitch) as the helper reports them
        self.cam_center = None
        self.cam_want_recenter = True
        self.cam_status = ""

    def shape(self, deg, max_deg, axis=0):
        """Head degrees -> stick deflection.

        The deadzone has hysteresis (engage at deadzone + HYST, release at deadzone)
        and the game-deadzone compensation ramps in over RAMP degrees, so sensor
        noise near the edge can't flick the stick on and off.
        """
        dz = self.cfg["deadzone"]
        engaged = self.engaged[axis]
        if not engaged and abs(deg) >= dz + HYST:
            engaged = True
        elif engaged and abs(deg) <= dz:
            engaged = False
        self.engaged[axis] = engaged
        if not engaged:
            return 0.0
        mag = abs(deg) - dz
        x = min(mag / max(max_deg - dz, 1e-3), 1.0) ** self.cfg["curve"]
        ad = self.cfg["anti_deadzone"] * min(1.0, mag / RAMP)  # jump past the game's own stick deadzone
        return math.copysign(ad + (1 - ad) * x, deg)

    def axes(self):
        """(yaw axis, pitch axis) in the tracker's frame: calibrated, or its defaults."""
        y = self.cfg.get("yaw_axis") or (0.0, 0.0, 1.0)
        p = self.cfg.get("pitch_axis") or (0.0, 1.0, 0.0)
        return tuple(y), tuple(p)

    def without_roll(self, q):
        """Swing-twist split: drop the twist about the head's forward axis (head tilt)."""
        if not self.cfg.get("ignore_roll", True):
            return q
        ya, pa = self.axes()
        ra = v_norm(v_cross(pa, ya))
        proj = v_dot(q[1:], ra)
        tw = (q[0], ra[0] * proj, ra[1] * proj, ra[2] * proj)
        n = math.sqrt(sum(c * c for c in tw))
        if n < 1e-9:
            return q
        tw = tuple(c / n for c in tw)
        return q_mul(q, q_conj(tw))

    def decompose(self, r):
        """Split a rotation vector into (yaw, pitch, roll) along the head axes.

        Solves r = yaw*Y + pitch*P + roll*R jointly, so axes measured during
        calibration don't have to be exactly perpendicular.
        """
        ya, pa = self.axes()
        ra = v_norm(v_cross(pa, ya))
        m = (ya, pa, ra)  # columns
        det = v_dot(ya, v_cross(pa, ra))
        if abs(det) < 1e-6:
            return v_dot(r, ya), v_dot(r, pa), v_dot(r, ra)
        # Cramer's rule
        return (v_dot(r, v_cross(m[1], m[2])) / det,
                v_dot(ya, v_cross(r, m[2])) / det,
                v_dot(ya, v_cross(pa, r)) / det)

    def _correct_drift(self, now):
        strength = 0 if self.calibrating else self.cfg.get("drift", 0.5)
        prev, self.prev_pkt = self.prev_pkt, (now, self.yaw)
        if strength <= 0 or prev is None:
            return
        dt = now - prev[0]
        if not 0 < dt < 0.5:
            return
        speed = abs((self.yaw - prev[1] + 180) % 360 - 180) / dt
        if abs(self.yaw) < DRIFT_WINDOW_DEG and speed < DRIFT_MAX_SPEED_DPS:
            # time constant: 8 s at strength 0.5, 4 s at 1.0
            k = min(1.0, dt * strength / 4.0)
            shift = self.yaw * k
            self.qc = q_mul(self.qc, q_axis_angle(self.axes()[0], shift))
            self.yaw -= shift

    def mode(self):
        return self.cfg.get("webcam", "off")

    def start(self):
        control = self.mode() == "control"
        tracker = None if control else find_tracker(self.cfg.get("tracker"))
        if not control and not tracker:
            raise FileNotFoundError("sony-tracker not found (install it or set its path)")
        self.pad = VirtualPad(self.cfg.get("pad", "playstation"))
        if not control:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("127.0.0.1", PORT))
            self.sock.settimeout(0.05)
        self.running = True
        self.cam = self.cam_raw = None
        self.cam_want_recenter = True
        if self.mode() != "off":
            self._start_cam()
        self.want_recenter = True
        self.restarts = 0
        self.last_packet = self.spawned = time.monotonic()
        self.status = "Connecting to headphones…"
        self.target = (0.0, 0.0)
        self.filters = (OneEuro(), OneEuro())
        if control:
            self.status = "Starting webcam…"
            self.thread = None
        else:
            self.thread = threading.Thread(target=self._loop, args=(tracker,), daemon=True)
            self.thread.start()
        self.out_thread = threading.Thread(target=self._output_loop, daemon=True)
        self.out_thread.start()

    def _spawn(self, tracker):
        # stderr goes to a file: a pipe nobody reads fills up and blocks the tracker
        if self.errlog is None:
            self.errlog = open(Path(GLib.get_user_runtime_dir()) / "swivel-tracker.log", "w+")
        self.proc = subprocess.Popen([tracker], stdout=subprocess.DEVNULL, stderr=self.errlog)
        self.spawned = time.monotonic()

    def _last_error(self, default):
        try:
            self.errlog.seek(0)
            lines = [ln.strip() for ln in self.errlog.read().splitlines() if ln.strip()]
            return lines[-1] if lines else default
        except (OSError, AttributeError):
            return default

    def _kill_proc(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.proc = None

    def _start_cam(self):
        uv = find_uv()
        helper = HERE / "swivel_cam.py"
        if not uv or not helper.exists():
            self.cam_status = "webcam helper unavailable (needs uv)"
            return
        self.cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cam_sock.bind(("127.0.0.1", CAM_PORT))
        self.cam_sock.settimeout(0.1)
        self.cam_log = open(Path(GLib.get_user_runtime_dir()) / "swivel-webcam.log", "w")
        self.cam_proc = subprocess.Popen(
            [uv, "run", "--quiet", "--script", str(helper), "--device", self.cfg.get("cam_device", "/dev/video0"),
             "--port", str(CAM_PORT)], stdout=subprocess.DEVNULL, stderr=self.cam_log)
        self.cam_status = "webcam starting"
        self.cam_thread = threading.Thread(target=self._cam_loop, daemon=True)
        self.cam_thread.start()

    def _cam_loop(self):
        last = None
        while self.running:
            try:
                data = self.cam_sock.recv(64)
            except (socket.timeout, OSError):
                data = None
            if self.cam_proc and self.cam_proc.poll() is not None:
                self.cam_status = "webcam helper stopped (see swivel-webcam.log)"
                break
            if not data or len(data) < 56:
                continue
            *_xyz, yaw, pitch, _roll, valid = struct.unpack("<7d", data[:56])
            now = time.monotonic()
            with self.lock:
                if valid < 0.5:
                    self.cam = None
                    self.cam_status = "webcam: no face"
                    continue
                self.cam_raw = (yaw, pitch)
                if self.cam_want_recenter or self.cam_center is None:
                    self.cam_center = (yaw, pitch)
                    self.cam_want_recenter = False
                sy, sp = self.cfg.get("cam_sign", [1, 1])
                cy = sy * (yaw - self.cam_center[0])
                cp = sp * (pitch - self.cam_center[1])
                self.cam = (now, cy, cp)
                self.cam_status = "webcam: tracking"
                if self.mode() == "control":
                    if self.want_recenter:
                        self.cam_center = (yaw, pitch)
                        self.want_recenter = False
                        cy = cp = 0.0
                    if self.calib_sink is not None:
                        self.calib_sink.append((now, (cy, cp)))
                    self.yaw, self.pitch, self.roll = cy, cp, 0.0  # webcam roll is never used
                    dt = now - last if last and 0 < now - last < 0.5 else 1 / 30
                    last = now
                    self._set_target(dt)
                    self.last_packet = now
                    self.status = "Tracking (webcam)"
                    if not self.history or now - self.history[-1][0] >= 0.02:
                        self.history.append((now, self.yaw, self.pitch, self.roll))

    def _webcam_assist(self, dt, speed):
        """Pull the headphone center toward the webcam's drift-free view of the head."""
        if self.mode() != "assist" or self.calibrating or not self.cam or not 0 < dt < 0.5:
            return False
        t, cy, cp = self.cam
        if time.monotonic() - t > CAM_STALE or speed > CAM_MAX_SPEED_DPS:
            return False
        if abs(cy) > CAM_MAX_ANGLE or abs(cp) > CAM_MAX_ANGLE:
            return False
        k = min(1.0, dt / CAM_TAU)
        ya, pa = self.axes()
        ey, ep = (self.yaw - cy) * k, (self.pitch - cp) * k
        self.qc = q_mul(q_mul(self.qc, q_axis_angle(ya, ey)), q_axis_angle(pa, ep))
        self.yaw -= ey
        self.pitch -= ep
        return True

    def _set_target(self, dt):
        c = self.cfg
        tx = self.shape(self.yaw, c["yaw_max"], 0) * (-1 if c["invert_yaw"] else 1)
        ty = self.shape(self.pitch, c["pitch_max"], 1) * (-1 if c["invert_pitch"] else 1)
        mag = math.hypot(tx, ty)
        if mag > 1:  # round gate, like a real stick
            tx, ty = tx / mag, ty / mag
        # Smoothing slider -> One Euro minimum cutoff (0 = snappy, 0.9 = very smooth)
        mc = 0.4 + 4.0 * (1 - c["smooth"])
        fx, fy = self.filters
        fx.min_cutoff = fy.min_cutoff = mc
        self.target = (fx(tx, dt), fy(ty, dt))

    def stop(self, keep_status=False):
        self.running = False
        for t in (self.thread, self.out_thread, self.cam_thread):
            if t and t is not threading.current_thread():
                t.join(2)
        if self.cam_proc and self.cam_proc.poll() is None:
            self.cam_proc.terminate()
            try:
                self.cam_proc.wait(3)
            except subprocess.TimeoutExpired:
                self.cam_proc.kill()
        self.cam_proc = None
        for f in ("cam_sock", "cam_log"):
            if getattr(self, f):
                getattr(self, f).close()
                setattr(self, f, None)
        self.cam = None
        self._kill_proc()
        if self.errlog:
            self.errlog.close()
            self.errlog = None
        if self.pad:
            self.pad.right_stick(0, 0)
            self.pad.close()
            self.pad = None
        if self.sock:
            self.sock.close()
            self.sock = None
        self.sx = self.sy = 0.0
        if not keep_status:
            self.status = "Stopped"

    def _output_loop(self):
        """Glide the stick toward the filtered target at OUTPUT_HZ for smooth camera motion."""
        period = 1.0 / OUTPUT_HZ
        a = 1 - math.exp(-period / GLIDE_SECONDS)
        nxt = time.monotonic()
        while self.running:
            with self.lock:
                tx, ty = self.target
                self.sx += (tx - self.sx) * a
                self.sy += (ty - self.sy) * a
                if self.pad:
                    self.pad.right_stick(self.sx, self.sy)
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _apply_bias(self, dt):
        """Cancel the gyro's constant creep measured during calibration."""
        b = self.cfg.get("bias")
        if not b or self.calibrating or not 0 < dt < 0.5:
            return
        rate = math.sqrt(v_dot(b, b))
        if rate > 1e-6:
            self.qc = q_mul(self.qc, q_axis_angle(v_norm(b), rate * dt))

    def _loop(self, tracker):
        if not sony_hidraw_present():
            connect_hid_profiles()
            deadline = time.monotonic() + 6
            while self.running and not sony_hidraw_present() and time.monotonic() < deadline:
                time.sleep(0.25)
        if not self.running:
            return
        self._spawn(tracker)
        self.status = "Waiting for headphones…"
        while self.running:
            try:
                data = self.sock.recv(64)
            except (socket.timeout, OSError):
                data = None
            if self.proc.poll() is not None:
                self.status = "Tracker stopped: " + self._last_error(f"exit {self.proc.returncode}")
                self.running = False
                break
            limit = min(STALL_SECONDS * 2 ** self.fail_streak, STALL_MAX_SECONDS)
            if time.monotonic() - max(self.last_packet, self.spawned) > limit:
                # The headset stopped streaming (sony-tracker just waits forever).
                # A fresh tracker re-enables the sensor.
                self.restarts += 1
                self.fail_streak += 1
                self.status = f"Headphones went quiet, restarting tracker ({self.restarts})…"
                self._kill_proc()
                self._spawn(tracker)
                # a new tracker calls wherever the head points "zero"; continue from
                # the last known angles instead of jumping
                self.reanchor = True
            with self.lock:
                now = time.monotonic()
                if data and len(data) >= 48:
                    _, _, _, yaw, pitch, roll = struct.unpack("<6d", data[:48])
                    q = q_from_euler(yaw, pitch, roll)
                    pkt_dt = now - self.last_packet
                    self._apply_bias(pkt_dt)
                    if self.want_recenter:
                        self.qc = q
                        self.want_recenter = False
                        self.reanchor = False
                        self.cam_want_recenter = True
                    elif self.reanchor:
                        # new tracker, new zero: keep the last relative orientation
                        self.qc = q_mul(q, q_conj(self.qr))
                        self.reanchor = False
                    self.qr = q_mul(q_conj(self.qc), q)
                    if self.calib_sink is not None:
                        self.calib_sink.append((now, self.qr))
                    prev_yaw = self.yaw
                    self.yaw, self.pitch, self.roll = self.decompose(q_rotvec(self.without_roll(self.qr)))
                    dt = pkt_dt if 0 < pkt_dt < 0.5 else 0.04
                    speed = abs(self.yaw - prev_yaw) / dt
                    if not self._webcam_assist(dt, speed):
                        self._correct_drift(now)
                    self._set_target(dt)
                    self.last_packet = now
                    self.fail_streak = 0
                    self.status = "Tracking"
                    if not self.history or now - self.history[-1][0] >= 0.02:
                        self.history.append((now, self.yaw, self.pitch, self.roll))
                elif now - self.last_packet > 1.0:
                    # hold the camera through short dropouts, recenter on long ones
                    self.target = (self.target[0] * 0.5, self.target[1] * 0.5)
                    if self.status == "Tracking":
                        self.status = "No data from headphones"


# ---- drawing --------------------------------------------------------------------------
SERIES = (("Yaw", (0.90, 0.38, 0.00)), ("Pitch", (0.21, 0.52, 0.89)), ("Roll", (0.15, 0.64, 0.41)))


def _cup(sign):
    """Ear cup as a 14-sided prism, axis along x."""
    n, r, cx, half = 14, 0.45, sign * 1.0, 0.14
    ring = [(math.cos(2 * math.pi * i / n) * r, math.sin(2 * math.pi * i / n) * r) for i in range(n)]
    outer = [(cx + sign * half, y, z) for y, z in ring]
    inner = [(cx - sign * half, y, z) for y, z in ring]
    faces = [(outer, (0.46, 0.46, 0.49)), (inner, (0.22, 0.22, 0.24))]
    for i in range(n):
        j = (i + 1) % n
        faces.append(([outer[i], outer[j], inner[j], inner[i]], (0.36, 0.36, 0.39)))
    return faces


def _band():
    faces, steps, w = [], 20, 0.22
    pts = [(math.cos(math.pi * i / steps) * 1.0, 0.45 + math.sin(math.pi * i / steps) * 1.0)
           for i in range(steps + 1)]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        faces.append(([(x0, y0, -w), (x1, y1, -w), (x1, y1, w), (x0, y0, w)], (0.52, 0.52, 0.55)))
    return faces


HEADPHONES = _cup(1) + _cup(-1) + _band()
PIVOT = (0.0, 0.55, 0.0)


def rotate(p, yaw, pitch, roll):
    x, y, z = p[0] - PIVOT[0], p[1] - PIVOT[1], p[2] - PIVOT[2]
    cr, sr = math.cos(roll), math.sin(roll)
    x, y = x * cr - y * sr, x * sr + y * cr
    cp, sp = math.cos(pitch), math.sin(pitch)
    y, z = y * cp + z * sp, -y * sp + z * cp  # positive pitch = looking up
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, z = x * cy + z * sy, -x * sy + z * cy
    # fixed camera: straight behind the wearer, a little above
    t = math.radians(-24)
    y, z = y * math.cos(t) - z * math.sin(t), y * math.sin(t) + z * math.cos(t)
    return x, y, z


class Painter:
    """Just enough of the cairo API on top of GtkSnapshot + GskPath.

    Avoids a dependency on python3-gi-cairo, which many systems don't have.
    """

    def __init__(self, snapshot, widget):
        self.snap, self.widget = snapshot, widget
        self.rgba = Gdk.RGBA()
        self.width = 1.0
        self.font_px = 10.0
        self.pb = Gsk.PathBuilder.new()
        self.pos = (0.0, 0.0)

    def set_source_rgba(self, r, g, b, a=1.0):
        self.rgba = Gdk.RGBA()
        self.rgba.red, self.rgba.green, self.rgba.blue, self.rgba.alpha = r, g, b, a

    def set_source_rgb(self, r, g, b):
        self.set_source_rgba(r, g, b, 1.0)

    def set_line_width(self, w):
        self.width = w

    def move_to(self, x, y):
        self.pb.move_to(x, y)
        self.pos = (x, y)

    def line_to(self, x, y):
        self.pb.line_to(x, y)
        self.pos = (x, y)

    def close_path(self):
        self.pb.close()

    def arc(self, cx, cy, r, _a0, _a1):  # only full circles are used
        self.pb.add_circle(Graphene.Point().init(cx, cy), r)

    def rectangle(self, x, y, w, h):
        self.pb.add_rect(Graphene.Rect().init(x, y, w, h))

    def _take(self, keep):
        path = self.pb.to_path()
        if not keep:
            self.pb = Gsk.PathBuilder.new()
        return path

    def fill(self, keep=False):
        self.snap.append_fill(self._take(keep), Gsk.FillRule.WINDING, self.rgba)

    def fill_preserve(self):
        self.fill(keep=True)

    def stroke(self):
        self.snap.append_stroke(self._take(False), Gsk.Stroke.new(self.width), self.rgba)

    def set_font_size(self, px):
        self.font_px = px

    def show_text(self, text):
        layout = self.widget.create_pango_layout(text)
        desc = Pango.FontDescription.new()
        desc.set_absolute_size(self.font_px * Pango.SCALE)
        layout.set_font_description(desc)
        baseline = layout.get_baseline() / Pango.SCALE
        self.snap.save()
        self.snap.translate(Graphene.Point().init(self.pos[0], self.pos[1] - baseline))
        self.snap.append_layout(layout, self.rgba)
        self.snap.restore()


class Canvas(Gtk.Widget):
    __gtype_name__ = "SwivelCanvas"

    def __init__(self, draw, width=-1, height=-1, hexpand=False):
        super().__init__(hexpand=hexpand)
        self.draw = draw
        self.set_size_request(width, height)

    def do_snapshot(self, snapshot):
        self.draw(self, Painter(snapshot, self), self.get_width(), self.get_height())


# ---- sound cues ---------------------------------------------------------------------
SOUND_DIR = Path("/usr/share/sounds/freedesktop/stereo")


VOICE_DIR = HERE / "sounds"


def _play(path):
    player = shutil.which("paplay") or shutil.which("pw-play")
    if player and path.exists():
        subprocess.Popen([player, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    return False


def cue(sound=None, say=None, clip=None):
    """Play a chime and/or a voice prompt without blocking.

    `clip` names a recorded prompt in sounds/ (e.g. "calib-up"); if it's missing,
    `say` is spoken with spd-say instead.
    """
    if sound:
        _play(SOUND_DIR / f"{sound}.oga")
    if clip and _play(VOICE_DIR / f"{clip}.ogg"):
        return
    if say and shutil.which("spd-say"):
        subprocess.Popen(["spd-say", "-r", "10", say], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ---- UI -------------------------------------------------------------------------------
class Window(Adw.ApplicationWindow):
    def __init__(self, app, engine):
        super().__init__(application=app, title="Swivel", default_width=760, default_height=860)
        self.engine = engine
        self.cfg = engine.cfg

        self.toggle = Gtk.Switch(valign=Gtk.Align.CENTER, tooltip_text="Start / stop tracking")
        self.toggle.connect("notify::active", self.on_toggle)
        header = Adw.HeaderBar()
        header.pack_end(self.toggle)

        self.status = Gtk.Label(label="Stopped", css_classes=["title-4"])
        self.readout = Gtk.Label(label="", css_classes=["dim-label", "numeric"])

        self.stick = Canvas(self.draw_stick, 230, 230)
        self.model = Canvas(self.draw_model, 300, 230, hexpand=True)
        self.graph = Canvas(self.draw_graph, -1, 170, hexpand=True)

        row = Gtk.Box(spacing=18, halign=Gtk.Align.FILL)
        row.append(self._titled("Right stick", self.stick))
        row.append(self._titled("Headphones", self.model))

        recenter = Gtk.Button(label="Recenter", css_classes=["pill", "suggested-action"],
                              halign=Gtk.Align.CENTER)
        recenter.connect("clicked", lambda *_: setattr(self.engine, "want_recenter", True))
        calibrate = Gtk.Button(label="Calibrate", css_classes=["pill"],
                               tooltip_text="Teach Swivel which way is up and right for how you wear the headset")
        calibrate.connect("clicked", lambda *_: self.start_calibration())
        self.calib_button = calibrate
        self.cancel_button = Gtk.Button(label="Cancel", css_classes=["pill", "flat"], visible=False)
        self.cancel_button.connect("clicked", lambda *_: self._calib_abort("Calibration cancelled"))
        self.calib_stage = None
        self.calib_busy = False
        buttons = Gtk.Box(spacing=12, halign=Gtk.Align.CENTER)
        buttons.append(recenter)
        buttons.append(calibrate)
        buttons.append(self.cancel_button)
        self.calib_steps = None
        self.calib_msg = None

        live = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12, margin_top=12, margin_bottom=6)
        for w in (self.status, row, self._titled(f"Last {HISTORY_SECONDS:.0f} seconds", self.graph),
                  self.readout, buttons):
            live.append(w)

        tune = Adw.PreferencesGroup(title="Tuning", description="Head angle for full stick, and feel")
        self._spin(tune, "Yaw range", "Degrees of left/right turn for full stick", "yaw_max", 10, 90, 1, 0)
        self._spin(tune, "Pitch range", "Degrees of up/down tilt for full stick", "pitch_max", 5, 60, 1, 0)
        self._spin(tune, "Deadzone", "Degrees ignored around center", "deadzone", 0, 20, 0.5, 1)
        self._spin(tune, "Game deadzone compensation",
                   "Skips the game's dead stick zone so small head turns register",
                   "anti_deadzone", 0, 0.5, 0.01, 2)
        self._spin(tune, "Curve", "Above 1 gives finer control near center", "curve", 1, 3, 0.1, 1)
        self._spin(tune, "Smoothing", "Higher is smoother but adds lag", "smooth", 0, 0.9, 0.05, 2)
        self._spin(tune, "Drift correction",
                   "Slowly re-centers left/right while you look roughly ahead. 0 turns it off",
                   "drift", 0, 1, 0.1, 1)

        ctl = Adw.PreferencesGroup(title="Controller")
        kinds = ["playstation", "xbox"]
        pad_row = Adw.ComboRow(title="Controller type",
                               subtitle="PlayStation shows PS button prompts. Applies on next start",
                               model=Gtk.StringList.new(["PlayStation", "Xbox"]))
        want = kinds.index(self.cfg.get("pad", "playstation"))
        pad_row.set_selected(want)

        def hook_pad_row():
            # GTK can reset the selection to 0 while the row is set up; reapply the
            # saved value first, and only then start saving user changes.
            pad_row.set_selected(want)
            pad_row.connect("notify::selected", lambda r, _p: self.set_cfg("pad", kinds[r.get_selected()]))
            return False

        GLib.idle_add(hook_pad_row)
        ctl.add(pad_row)
        modes = ["off", "assist", "control"]
        cam_row = Adw.ComboRow(title="Webcam",
                               subtitle="Drift correction keeps the headphones' center true. "
                                        "Control steers with your face, no headphones. Applies on next start",
                               model=Gtk.StringList.new(["Off", "Drift correction", "Control (no headphones)"]))
        cam_want = modes.index(self.cfg.get("webcam", "off")) if self.cfg.get("webcam") in modes else 0
        cam_row.set_selected(cam_want)

        def hook_cam_row():
            cam_row.set_selected(cam_want)
            cam_row.connect("notify::selected", lambda r, _p: self.set_cfg("webcam", modes[r.get_selected()]))
            return False

        GLib.idle_add(hook_cam_row)
        ctl.add(cam_row)
        self._switch(ctl, "Ignore head tilt (roll)", "ignore_roll")
        self.inv_rows = {k: self._switch(ctl, t, k) for t, k in
                         (("Invert yaw", "invert_yaw"), ("Invert pitch", "invert_pitch"))}
        self.axes_row = Adw.ActionRow(title="Head axes")
        reset_axes = Gtk.Button(label="Reset", valign=Gtk.Align.CENTER)
        reset_axes.connect("clicked", lambda *_: self.reset_axes())
        self.axes_row.add_suffix(reset_axes)
        ctl.add(self.axes_row)
        self._update_axes_row()
        hint = Adw.PreferencesGroup(
            description="Start this before the game. Recenter from a keyboard shortcut "
                        "bound to: swivel --recenter")

        page = Adw.PreferencesPage()
        top = Adw.PreferencesGroup()
        top.add(live)
        for g in (top, tune, ctl, hint):
            page.add(g)
        view = Adw.ToolbarView()
        view.add_top_bar(header)
        view.set_content(page)
        self.set_content(view)
        self.connect("close-request", lambda *_: self.engine.stop() or False)
        GLib.timeout_add(33, self.tick)

    @staticmethod
    def _titled(title, child):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6, hexpand=child.get_hexpand())
        box.append(Gtk.Label(label=title, css_classes=["caption-heading", "dim-label"], xalign=0))
        box.append(child)
        return box

    def _spin(self, group, title, sub, key, lo, hi, step, digits):
        row = Adw.SpinRow.new_with_range(lo, hi, step)
        row.set_title(title)
        row.set_subtitle(sub)
        row.set_digits(digits)
        row.set_value(self.cfg[key])
        row.connect("notify::value", lambda r, _p: self.set_cfg(key, r.get_value()))
        group.add(row)

    def _switch(self, group, title, key):
        row = Adw.SwitchRow(title=title, active=self.cfg[key])
        row.connect("notify::active", lambda r, _p: self.set_cfg(key, r.get_active()))
        group.add(row)
        return row

    # ---- calibration ----
    def _update_axes_row(self):
        done = bool(self.cfg.get("yaw_axis"))
        b = self.cfg.get("bias")
        creep = f", gyro creep {math.sqrt(v_dot(b, b)):.2f}°/s cancelled" if b else ""
        self.axes_row.set_subtitle(f"Calibrated for how you wear the headset{creep}" if done
                                   else "Default. Press Calibrate for accurate axes and less drift")

    def reset_axes(self):
        self.set_cfg("yaw_axis", None)
        self.set_cfg("pitch_axis", None)
        self.set_cfg("bias", None)
        self.engine.want_recenter = True
        self._update_axes_row()

    # Calibration is a short wizard: Swivel says where to look, you look there and
    # press Next (or F8 / `swivel --recenter`, which works while a game has focus).
    CALIB_PROMPTS = {
        "still": "Look straight at the screen, then press Next (or F8)",
        "up": "Look UP and hold it, then press Next (or F8)",
        "right": "Look RIGHT and hold it, then press Next (or F8)",
    }
    CALIB_ORDER = ["still", "up", "right"]
    CALIB_SPOKEN = {
        "still": "Look straight at the screen, then press F 8.",
        "up": "Now look up and hold, then press F 8.",
        "right": "Now look right and hold, then press F 8.",
    }

    def start_calibration(self):
        e = self.engine
        if self.calib_stage:
            self.calib_next()
            return
        if not e.running or not e.status.startswith("Tracking"):
            self._flash("Start tracking first, then press Calibrate")
            return
        self.results = {}
        self.calib_stage = "still"
        self.calib_busy = False
        self.cam_samples = None
        e.calibrating = True
        self.calib_steps = True  # keeps status messages pinned
        self.calib_msg = self.CALIB_PROMPTS["still"]
        cue(say=self.CALIB_SPOKEN["still"], clip="calib-start")
        self.calib_button.set_label("Next")
        self.cancel_button.set_visible(True)
        GLib.timeout_add(33, self._cam_sample)

    def calib_next(self):
        """Measure the current pose (button or F8)."""
        if not self.calib_stage or self.calib_busy:
            return
        if not self.engine.running:
            self._calib_abort("Calibration stopped: tracking is off")
            return
        stage = self.calib_stage
        self.calib_busy = True
        cue("audio-volume-change")
        if stage == "still":
            self.engine.want_recenter = True
            secs, msg = 3.0, "Hold still"
            cue(say="Hold still.", clip="calib-hold-still")
        else:
            secs, msg = 0.7, "Hold it"
        self.engine.calib_sink = []
        self.cam_samples = []
        t0 = time.monotonic()

        def tick():
            left = secs - (time.monotonic() - t0)
            if not self.calib_stage:
                return False
            if left > 0:
                self.calib_msg = f"{msg}…  ({math.ceil(left)})"
                return True
            self._calib_take(stage)
            self.calib_busy = False
            i = self.CALIB_ORDER.index(stage)
            if i + 1 < len(self.CALIB_ORDER):
                self.calib_stage = self.CALIB_ORDER[i + 1]
                self.calib_msg = self.CALIB_PROMPTS[self.calib_stage]
                cue("message", self.CALIB_SPOKEN[self.calib_stage], clip=f"calib-{self.calib_stage}")
            else:
                self._calib_end_ui()
                self._calib_finish()
            return False

        GLib.timeout_add(100, tick)

    def _calib_end_ui(self):
        self.calib_stage = None
        self.calib_steps = None
        self.calib_button.set_label("Calibrate")
        self.cancel_button.set_visible(False)

    def _calib_take(self, name):
        self.results[name] = self.engine.calib_sink or []
        self.engine.calib_sink = None
        self.results["cam_" + name] = list(self.cam_samples or [])
        self.cam_samples = None

    def _cam_sample(self):
        # record raw webcam angles while a pose is being measured
        e = self.engine
        if self.cam_samples is not None and e.cam_raw is not None and e.cam:
            self.cam_samples.append(e.cam_raw)
        return self.calib_stage is not None

    def _calib_abort(self, msg):
        cue("dialog-warning")
        self._calib_end_ui()
        self.engine.calib_sink = None
        self.engine.calibrating = False
        self._flash(msg)

    def _calib_finish(self):
        self.calib_steps = None
        self.engine.calibrating = False
        if self.engine.mode() == "control":
            cam_msg = self._calib_cam_signs(None, None)
            self.engine.want_recenter = True
            self._flash(cam_msg or "Calibration failed: the webcam didn't see your face")
            return

        def mean_rotvec(samples):
            vs = [q_rotvec(q) for _t, q in samples]
            return tuple(sum(v[i] for v in vs) / len(vs) for i in range(3)) if vs else (0.0, 0.0, 0.0)

        up, right = mean_rotvec(self.results.get("up", [])), mean_rotvec(self.results.get("right", []))
        mu, mr = math.sqrt(v_dot(up, up)), math.sqrt(v_dot(right, right))
        if mu < 8 or mr < 8:
            cue("dialog-warning", "Calibration failed. Turn your head further and try again.",
                clip="calib-failed-further")
            self._flash("Calibration failed: turn your head further (at least ~15°) and hold it")
            return
        pa, ya = v_norm(up), v_norm(right)
        if abs(v_dot(pa, ya)) > 0.9:
            cue("dialog-warning", "Calibration failed. Try again.", clip="calib-failed")
            self._flash("Calibration failed: up and right looked the same, try again")
            return
        self.set_cfg("pitch_axis", list(pa))
        self.set_cfg("yaw_axis", list(ya))
        bias = self._measure_bias(self.results.get("still", []))
        self.set_cfg("bias", list(bias) if bias else None)
        # head angles the headphones measured in the holds, with the new axes
        hp = [self.engine.decompose(v) for v in (mean_rotvec(self.results["right"]), mean_rotvec(self.results["up"]))]
        cam_msg = self._calib_cam_signs(hp[0][0], hp[1][1])
        for row in self.inv_rows.values():
            row.set_active(False)  # calibration already defines up = up, right = right
        self.engine.want_recenter = True
        self._update_axes_row()
        self._flash("Calibrated. Up/down and left/right now follow your head" + (f". {cam_msg}" if cam_msg else ""))
        cue("complete", "Calibrated. Look at the screen.", clip="calib-done")

    def _calib_cam_signs(self, hp_yaw, hp_pitch):
        """Learn the webcam's direction and scale: webcam degrees -> headphone degrees."""
        still, up, right = (self.results.get(k) for k in ("cam_still", "cam_up", "cam_right"))
        dbg = {"cam_samples": [len(x or []) for x in (still, up, right)], "hp_yaw": hp_yaw, "hp_pitch": hp_pitch}
        dbg_path = Path(GLib.get_user_cache_dir()) / "swivel/last-calibration.json"
        dbg_path.parent.mkdir(parents=True, exist_ok=True)
        dbg_path.write_text(json.dumps(dbg))
        if not still or not up or not right:
            return "Webcam not calibrated: it didn't see your face during calibration"
        base = (sum(v[0] for v in still) / len(still), sum(v[1] for v in still) / len(still))
        dy = sum(v[0] for v in right) / len(right) - base[0]
        dp = sum(v[1] for v in up) / len(up) - base[1]
        dbg.update(cam_dyaw=dy, cam_dpitch=dp)
        dbg_path.write_text(json.dumps(dbg))
        if abs(dy) < 5 or abs(dp) < 5:
            return "Webcam not calibrated: turn further"

        def gain(hp, cam):
            g = (hp / cam) if hp else (1.0 if cam > 0 else -1.0)
            return g if 0.4 <= abs(g) <= 2.5 else (1.0 if g > 0 else -1.0)

        self.set_cfg("cam_sign", [gain(hp_yaw, dy), gain(hp_pitch, dp)])
        self.engine.cam_want_recenter = True
        return "Webcam calibrated"

    @staticmethod
    def _measure_bias(samples):
        """Slope of the rotation vector while holding still = the gyro's constant creep (deg/s)."""
        samples = samples[len(samples) // 5:]  # skip the first second (recenter settling)
        if len(samples) < 40:
            return None
        ts = [t for t, _q in samples]
        vs = [q_rotvec(q) for _t, q in samples]
        tm = sum(ts) / len(ts)
        den = sum((t - tm) ** 2 for t in ts)
        if den <= 0:
            return None
        slope = []
        for i in range(3):
            vm = sum(v[i] for v in vs) / len(vs)
            slope.append(sum((t - tm) * (v[i] - vm) for t, v in zip(ts, vs)) / den)
        rate = math.sqrt(sum(x * x for x in slope))
        spread = max(max(v[i] for v in vs) - min(v[i] for v in vs) for i in range(3))
        # only trust it if the head really was still and the creep is plausible
        return tuple(slope) if 0.005 < rate < 2.0 and spread < 6.0 else None

    def _flash(self, msg, secs=4.0):
        self.calib_msg = msg
        until = time.monotonic() + secs

        def clear():
            if time.monotonic() >= until and not self.calib_steps:
                self.calib_msg = None
                return False
            return True

        GLib.timeout_add(200, clear)

    def set_cfg(self, key, val):
        self.cfg[key] = val
        CONF.parent.mkdir(parents=True, exist_ok=True)
        CONF.write_text(json.dumps(self.cfg, indent=2))

    def on_toggle(self, sw, _p):
        if sw.get_active() and not self.engine.running:
            try:
                self.engine.start()
            except OSError as exc:
                self.engine.stop()
                self.engine.status = f"Could not start: {exc.strerror or exc}"
                sw.set_active(False)
        elif not sw.get_active() and self.engine.running:
            self.engine.stop()

    def tick(self):
        e = self.engine
        if DEMO and not e.running:
            self.demo_step()
        if not e.running and e.pad is not None:  # tracker died: clean up, keep its message
            e.stop(keep_status=True)
            self.toggle.set_active(False)
        self.status.set_label(self.calib_msg or e.status)
        cam = ""
        if e.running and e.mode() != "off":
            cam = f"    ·    {e.cam_status}"
            if e.cam and e.mode() == "assist":
                cam += f" (yaw {e.cam[1]:+.0f}°, pitch {e.cam[2]:+.0f}°)"
        self.readout.set_label(f"yaw {e.yaw:+6.1f}°    pitch {e.pitch:+6.1f}°    roll {e.roll:+6.1f}°{cam}")
        for w in (self.stick, self.model, self.graph):
            w.queue_draw()
        return True

    def demo_step(self):
        """SWIVEL_DEMO=1: fake head motion, for screenshots and trying the UI without a headset."""
        e, t = self.engine, time.monotonic()
        e.yaw = 32 * math.sin(t * 0.9)
        e.pitch = 12 * math.sin(t * 1.3 + 1)
        e.roll = 6 * math.sin(t * 0.7 + 2)
        e.sx = e.shape(e.yaw, self.cfg["yaw_max"]) * (-1 if self.cfg["invert_yaw"] else 1)
        e.sy = e.shape(e.pitch, self.cfg["pitch_max"]) * (-1 if self.cfg["invert_pitch"] else 1)
        e.status = "Demo (no headset)"
        with e.lock:
            e.history.append((t, e.yaw, e.pitch, e.roll))

    def _fg(self, cr, alpha):
        c = self.get_color()
        cr.set_source_rgba(c.red, c.green, c.blue, alpha)

    def draw_stick(self, _a, cr, w, h):
        r = min(w, h) / 2 - 8
        cx, cy = w / 2, h / 2
        cr.set_line_width(2)
        self._fg(cr, 0.25)
        cr.arc(cx, cy, r, 0, 2 * math.pi)
        cr.stroke()
        cr.move_to(cx - r, cy)
        cr.line_to(cx + r, cy)
        cr.move_to(cx, cy - r)
        cr.line_to(cx, cy + r)
        cr.stroke()
        cr.set_source_rgba(0.91, 0.33, 0.13, 1.0 if self.engine.running else 0.35)
        cr.arc(cx + self.engine.sx * r, cy - self.engine.sy * r, 11, 0, 2 * math.pi)
        cr.fill()

    def draw_model(self, _a, cr, w, h):
        e = self.engine
        sy = -1 if self.cfg["invert_yaw"] else 1
        sp = -1 if self.cfg["invert_pitch"] else 1
        yaw, pitch, roll = (math.radians(e.yaw) * sy, math.radians(e.pitch) * sp, math.radians(e.roll))
        scale, dist = min(w, h) * 0.40, 6.0
        light = (0.0, 0.45, -0.89)

        def project(p):
            x, y, z = p
            f = dist / (dist + z)
            return w / 2 + x * scale * f, h * 0.52 - y * scale * f, z

        polys = []
        for pts, color in HEADPHONES:
            rp = [rotate(p, yaw, pitch, roll) for p in pts]
            ax, ay, az = (rp[1][i] - rp[0][i] for i in range(3))
            bx, by, bz = (rp[2][i] - rp[0][i] for i in range(3))
            nx, ny, nz = ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
            nl = math.sqrt(nx * nx + ny * ny + nz * nz) or 1
            shade = 0.45 + 0.75 * abs((nx * light[0] + ny * light[1] + nz * light[2]) / nl)
            depth = sum(p[2] for p in rp) / len(rp)
            polys.append((depth, [project(p) for p in rp], tuple(min(1, c * shade) for c in color)))
        # the way you're facing
        tip = project(rotate((0, 0.55, 1.6), yaw, pitch, roll))
        base = project(rotate((0, 0.55, 0.0), yaw, pitch, roll))
        arrow_depth = (tip[2] + base[2]) / 2
        drawn_arrow = False
        for depth, pts, color in sorted(polys, key=lambda t: -t[0]):
            if not drawn_arrow and depth < arrow_depth:
                self._arrow(cr, base, tip)
                drawn_arrow = True
            cr.set_source_rgb(*color)
            cr.move_to(pts[0][0], pts[0][1])
            for x, y, _z in pts[1:]:
                cr.line_to(x, y)
            cr.close_path()
            cr.fill_preserve()
            cr.set_source_rgba(0, 0, 0, 0.35)
            cr.set_line_width(0.8)
            cr.stroke()
        if not drawn_arrow:
            self._arrow(cr, base, tip)

    def _arrow(self, cr, base, tip):
        cr.set_source_rgba(0.91, 0.33, 0.13, 0.95)
        cr.set_line_width(3)
        cr.move_to(base[0], base[1])
        cr.line_to(tip[0], tip[1])
        cr.stroke()
        ang = math.atan2(tip[1] - base[1], tip[0] - base[0])
        for d in (2.6, -2.6):
            cr.move_to(tip[0], tip[1])
            cr.line_to(tip[0] + 10 * math.cos(ang + d), tip[1] + 10 * math.sin(ang + d))
        cr.stroke()

    def draw_graph(self, _a, cr, w, h):
        e = self.engine
        left, right, top, bottom = 34, w - 8, 8, h - 18
        rng = 60.0
        cr.set_line_width(1)
        self._fg(cr, 0.12)
        for v in (-60, -30, 0, 30, 60):
            y = top + (rng - v) / (2 * rng) * (bottom - top)
            cr.move_to(left, y)
            cr.line_to(right, y)
            cr.stroke()
            self._fg(cr, 0.55)
            cr.set_font_size(10)
            cr.move_to(4, y + 3.5)
            cr.show_text(f"{v:+d}°" if v else "0°")
            self._fg(cr, 0.12)
        with e.lock:
            hist = list(e.history)
        now = time.monotonic()
        lx = left + 4
        for idx, (label, col) in enumerate(SERIES):
            cr.set_source_rgb(*col)
            cr.set_line_width(2)
            started = False
            for t, *vals in hist:
                age = now - t
                if age > HISTORY_SECONDS:
                    continue
                x = right - age / HISTORY_SECONDS * (right - left)
                v = max(-rng, min(rng, vals[idx]))
                y = top + (rng - v) / (2 * rng) * (bottom - top)
                if started:
                    cr.line_to(x, y)
                else:
                    cr.move_to(x, y)
                    started = True
            cr.stroke()
            cr.rectangle(lx, h - 11, 10, 3)
            cr.fill()
            self._fg(cr, 0.7)
            cr.set_font_size(11)
            cr.move_to(lx + 14, h - 6)
            cr.show_text(label)
            lx += 70


class App(Adw.Application):
    def __init__(self):
        super().__init__(application_id=APP_ID)
        cfg = dict(DEFAULTS)
        try:
            cfg.update(json.loads(CONF.read_text()))
        except (OSError, ValueError):
            pass
        self.engine = Engine(cfg)

    def do_activate(self):
        (self.get_active_window() or Window(self, self.engine)).present()

    def do_startup(self):
        Adw.Application.do_startup(self)
        PIDFILE.write_text(str(os.getpid()))
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGUSR1, self._on_usr1)
        for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, sig, lambda: self.quit() or False)

    def _on_usr1(self):
        # F8 / `swivel --recenter`: confirms a calibration step, otherwise recenters
        win = self.get_active_window()
        if win is not None and getattr(win, "calib_stage", None):
            win.calib_next()
        else:
            self.engine.want_recenter = True
        return True

    def do_shutdown(self):
        self.engine.stop()
        PIDFILE.unlink(missing_ok=True)
        Adw.Application.do_shutdown(self)


def recenter_running():
    """`swivel.py --recenter`: tell the running Swivel to recenter (bind this to a key)."""
    import sys
    try:
        os.kill(int(PIDFILE.read_text()), signal.SIGUSR1)
    except (OSError, ValueError):
        sys.exit("Swivel isn't running")


if __name__ == "__main__":
    import sys
    if "--recenter" in sys.argv[1:]:
        recenter_running()
    else:
        App().run(None)
