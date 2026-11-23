# Swivel

Look around in games by turning your head, using the motion sensor already inside
your Sony headphones. Swivel turns head yaw and pitch into the **right stick of a
virtual gamepad**, so it works in any game where the right stick moves the camera,
including Proton games with no TrackIR or OpenTrack support (Ace Combat, racing and
flight games, third-person games).

Linux only. GTK4 + libadwaita. Your real controller keeps working; the game reads
both.

![Swivel](docs/screenshot.png)

## Features

- Live views: right-stick position, a 3D model of the headphones following your
  head, and a 10-second yaw/pitch/roll graph
- Tuning that applies instantly: range, deadzone, response curve, smoothing, and
  **game deadzone compensation** (skips the game's own dead stick zone so small
  head turns register instead of feeling sticky)
- PlayStation (default) or Xbox virtual controller. If your buttons get scrambled
  with Steam Input on, switch to Xbox. Swivel only sends input when your head moves,
  so it doesn't keep stealing the "active controller" from your real pad
- **Calibrate** wizard: look straight, up, then right, pressing Next or F8 when you're
  in position (spoken prompts and chimes, so it works with the game in front). Swivel learns
  how the headset's sensor sits on your head. Fixes nodding showing up as turning
  (or roll), which happens because the sensor in the ear cup is tilted. It also
  measures the gyro's constant creep while you hold still and cancels it
- Smooth camera: the headset reports ~25 times a second; Swivel filters with a One
  Euro filter (steady when you're still, quick when you turn) and glides the stick
  at 120 Hz
- **Webcam** (optional, needs [uv](https://docs.astral.sh/uv/)): *Drift correction*
  uses OpenTrack's neural-net head-pose model (tracks to ~65 degrees; MediaPipe is
  available with --backend mediapipe) as a drift-free reference and slowly pulls the
  headphones' center back while your head is still; *Control* steers the camera with
  your face alone, no headphones needed. Video stays on this computer: the helper sends
  Swivel head angles plus a small preview image over localhost
  - In drift correction the headphones drive the stick (fast, smooth) and the camera only
    corrects the center, at any angle, matched to the camera's measured lag (about 100 ms)
  - If the headset re-zeroes itself or its Bluetooth stream drops out, the webcam steers
    until it's back, then lines the headset up with the camera right away
  - Camera poses are corrected for where your head is in the frame (leaning or sliding
    no longer reads as turning) and use the nose direction, so tilting your head doesn't
    move yaw or pitch
  - Learns from the camera if turning makes the headset read slightly up or down, and
    removes it
  - Live webcam preview with the tracked head box, and webcam zoom / tilt / pan controls
- **Phone as a camera**: the Swivel Cam Android app (`android/`) runs OpenTrack's
  head-pose network on the phone (Snapdragon NPU via ONNX Runtime QNN, CPU fallback) and
  streams head angles over USB (`adb reverse`), falling back to WiFi (TCP 4244; allow it
  in your firewall). *Camera source: Both* averages the PC webcam and the phone, each
  calibrated separately, weighted by how squarely each one sees your face (a camera
  below you sees mostly chin when you look up, so the one above takes over). Build with
  `cd android && ./gradlew assembleDebug`
- Connects the headset's head-tracking channel automatically (most Sony headsets
  connect for audio only)
- Recenter button and keyboard shortcut, plus automatic drift correction: the
  headset's yaw comes from a gyro with no compass, so center slowly slides; Swivel
  eases it back while you look roughly ahead
- Demo mode with simulated motion: `SWIVEL_DEMO=1 python3 swivel.py`

## How it works

```
Sony headset ──Bluetooth HID──► sony-tracker ──UDP 127.0.0.1:4242──► Swivel ──uinput──► virtual gamepad ──► game
```

Reading the headset is done by
[SonyTrackerLinux](https://github.com/kdani3/SonyTrackerLinux) (`sony-tracker`),
a Linux port of [sony-head-tracker](https://github.com/NicholasSlattery/sony-head-tracker).
Swivel adds the gamepad output and the app around it.

## Install

Debian/Ubuntu dependencies:

```bash
sudo apt install python3-gi gir1.2-gtk-4.0 gir1.2-adw-1 git build-essential
```

Then:

```bash
git clone https://github.com/trongio/swivel
cd swivel
./install.sh
sudo install -m644 udev/71-swivel.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`install.sh` builds `sony-tracker` if it isn't already on your `PATH`, and adds
Swivel to your app menu. The udev rule lets your user read the headset and create
the virtual controller without root.

## Use

1. Pair and connect your headphones as usual.
2. Open Swivel and flip the switch in the header bar.
3. Start the game, face the screen, and press **Recenter**.

To recenter from a keyboard shortcut (works while the game is fullscreen), bind
`swivel --recenter` to a key. On GNOME the installer can do it for you:

```bash
./install.sh --gnome-shortcut F8
```

If the camera turns the wrong way, use **Invert yaw** / **Invert pitch**. If it
feels sticky near the center, raise **Game deadzone compensation**; if the camera
drifts while you look straight ahead, lower it.

## Compatibility

Tested with a WH-1000XM5 on Ubuntu 24.04 (GNOME 46, X11) playing Ace Combat 8
through Proton. Other Sony models that `sony-tracker` supports should work the same.

## License

MIT. Voice prompts in `sounds/` were generated with ElevenLabs (voice: River).
