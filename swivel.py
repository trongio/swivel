#!/usr/bin/python3
"""Swivel: Sony headphone head tracking as a virtual gamepad's right stick.

Linux, GTK4 + libadwaita. Starts `sony-tracker` (https://github.com/kdani3/SonyTrackerLinux),
which reads the headset's orientation over Bluetooth HID and streams OpenTrack
UDP poses to 127.0.0.1:4242. Head yaw/pitch become right-stick deflection on a
virtual controller created through /dev/uinput, so any game that lets you look
around with the right stick gets head tracking, including Proton games with no
TrackIR/OpenTrack support.

Recenter from a keyboard shortcut: kill -USR1 "$(cat "$XDG_RUNTIME_DIR/swivel.pid")"
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
            "invert_pitch": False, "pad": "playstation", "tracker": ""}
HISTORY_SECONDS = 10.0
STALL_SECONDS = 2.5  # headsets sometimes stop streaming; restart the tracker after this
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

    def _axis(self, v):
        v = max(-1.0, min(1.0, v))
        return int(round((v + 1) / 2 * (self.hi - self.lo) + self.lo))

    def right_stick(self, x, y):
        # Only send changes, so the game doesn't keep treating this pad as the
        # active controller (and switching button prompts) while your head is still.
        val = (self._axis(x), self._axis(-y))  # head up = stick up
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

    def shape(self, deg, max_deg):
        dz = self.cfg["deadzone"]
        mag = abs(deg) - dz
        if mag <= 0:
            return 0.0
        x = min(mag / max(max_deg - dz, 1e-3), 1.0) ** self.cfg["curve"]
        ad = self.cfg["anti_deadzone"]  # jump past the game's own stick deadzone
        return math.copysign(ad + (1 - ad) * x, deg)

    def start(self):
        tracker = find_tracker(self.cfg.get("tracker"))
        if not tracker:
            raise FileNotFoundError("sony-tracker not found (install it or set its path)")
        self.pad = VirtualPad(self.cfg.get("pad", "playstation"))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", PORT))
        self.sock.settimeout(0.1)
        self.running = True
        self.want_recenter = True
        self.restarts = 0
        self.last_packet = self.spawned = time.monotonic()
        self.status = "Connecting to headphones…"
        self.thread = threading.Thread(target=self._loop, args=(tracker,), daemon=True)
        self.thread.start()

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

    def stop(self, keep_status=False):
        self.running = False
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(2)
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
            if time.monotonic() - max(self.last_packet, self.spawned) > STALL_SECONDS:
                # The headset stopped streaming (sony-tracker just waits forever).
                # A fresh tracker re-enables the sensor.
                self.restarts += 1
                self.status = f"Headphones went quiet, restarting tracker ({self.restarts})…"
                self._kill_proc()
                if not sony_hidraw_present():
                    connect_hid_profiles()
                self._spawn(tracker)
            c = self.cfg
            with self.lock:
                now = time.monotonic()
                if data and len(data) >= 48:
                    _, _, _, yaw, pitch, roll = struct.unpack("<6d", data[:48])
                    if self.want_recenter:
                        self.center = (yaw, pitch, roll)
                        self.want_recenter = False
                    self.yaw = (yaw - self.center[0] + 180) % 360 - 180
                    self.pitch = pitch - self.center[1]
                    self.roll = (roll - self.center[2] + 180) % 360 - 180
                    tx = self.shape(self.yaw, c["yaw_max"]) * (-1 if c["invert_yaw"] else 1)
                    ty = self.shape(self.pitch, c["pitch_max"]) * (-1 if c["invert_pitch"] else 1)
                    mag = math.hypot(tx, ty)
                    if mag > 1:  # round gate, like a real stick
                        tx, ty = tx / mag, ty / mag
                    k = c["smooth"]
                    self.sx = k * self.sx + (1 - k) * tx
                    self.sy = k * self.sy + (1 - k) * ty
                    self.last_packet = now
                    self.status = "Tracking"
                    if not self.history or now - self.history[-1][0] >= 0.02:
                        self.history.append((now, self.yaw, self.pitch, self.roll))
                else:
                    self.sx *= 0.5
                    self.sy *= 0.5
                    if now - self.last_packet > 1 and self.status == "Tracking":
                        self.status = "No data from headphones"
                self.pad.right_stick(self.sx, self.sy)


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

        live = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12, margin_top=12, margin_bottom=6)
        for w in (self.status, row, self._titled(f"Last {HISTORY_SECONDS:.0f} seconds", self.graph),
                  self.readout, recenter):
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

        ctl = Adw.PreferencesGroup(title="Controller")
        kinds = list(PADS)
        pad_row = Adw.ComboRow(title="Controller type",
                               subtitle="PlayStation shows PS button prompts. Applies on next start",
                               model=Gtk.StringList.new(["PlayStation", "Xbox"]))
        pad_row.set_selected(kinds.index(self.cfg.get("pad", "playstation")))
        pad_row.connect("notify::selected", lambda r, _p: self.set_cfg("pad", kinds[r.get_selected()]))
        ctl.add(pad_row)
        self._switch(ctl, "Invert yaw", "invert_yaw")
        self._switch(ctl, "Invert pitch", "invert_pitch")
        hint = Adw.PreferencesGroup(
            description="Start this before the game. Recenter from a keyboard shortcut: "
                        'kill -USR1 "$(cat "$XDG_RUNTIME_DIR/swivel.pid")"')

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
        self.status.set_label(e.status)
        self.readout.set_label(f"yaw {e.yaw:+6.1f}°    pitch {e.pitch:+6.1f}°    roll {e.roll:+6.1f}°")
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
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGUSR1,
                             lambda: setattr(self.engine, "want_recenter", True) or True)
        for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, sig, lambda: self.quit() or False)

    def do_shutdown(self):
        self.engine.stop()
        PIDFILE.unlink(missing_ok=True)
        Adw.Application.do_shutdown(self)


if __name__ == "__main__":
    App().run(None)
