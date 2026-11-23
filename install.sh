#!/usr/bin/env bash
# Install Swivel for the current user. Re-run any time to update.
set -euo pipefail

SRC=$(cd "$(dirname "$0")" && pwd)
DEST="${XDG_DATA_HOME:-$HOME/.local/share}/swivel"
APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"

need() { command -v "$1" >/dev/null || { echo "missing: $1 ($2)" >&2; exit 1; }; }
# GTK bindings live in the distro Python, not in pyenv/uv/Homebrew ones
PY=/usr/bin/python3
[[ -x $PY ]] || PY=$(command -v python3)
need git "git"
need make "build tools (make, gcc)"
need gcc "build tools (make, gcc)"
need bluetoothctl "BlueZ"
need busctl "systemd"
"$PY" - <<'EOF' || { echo "missing GTK4/libadwaita Python bindings: install python3-gi, gir1.2-gtk-4.0 and gir1.2-adw-1 (Debian/Ubuntu) or python-gobject + libadwaita (Arch/Fedora)" >&2; exit 1; }
import gi
gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Adw, Gtk  # noqa: F401
EOF

mkdir -p "$DEST" "$APPS"
install -m755 "$SRC/swivel.py" "$DEST/swivel.py"
install -m644 "$SRC/data/io.github.trongio.Swivel.svg" "$DEST/swivel.svg"

# sony-tracker does the actual headset reading
if ! command -v sony-tracker >/dev/null; then
  if [[ -d $DEST/SonyTrackerLinux/.git ]]; then
    git -C "$DEST/SonyTrackerLinux" pull --ff-only
  else
    git clone --depth 1 https://github.com/kdani3/SonyTrackerLinux "$DEST/SonyTrackerLinux"
  fi
  make -C "$DEST/SonyTrackerLinux"
fi

sed -e "s|@EXEC@|$PY $DEST/swivel.py|" -e "s|@ICON@|$DEST/swivel.svg|" \
  "$SRC/data/io.github.trongio.Swivel.desktop.in" >"$APPS/io.github.trongio.Swivel.desktop"
update-desktop-database "$APPS" 2>/dev/null || true

echo "Installed to $DEST"
if [[ ! -f /etc/udev/rules.d/71-swivel.rules ]]; then
  cat <<EOF

One-time permission setup (lets Swivel read the headset and create the virtual
controller without root):

  sudo install -m644 "$SRC/udev/71-swivel.rules" /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger

EOF
fi
