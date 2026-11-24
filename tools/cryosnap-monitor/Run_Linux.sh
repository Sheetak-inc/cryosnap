#!/bin/bash
# CryoSnap Monitor Launcher (Linux / Raspberry Pi)
# Works on 32-bit and 64-bit Raspberry Pi OS, and on Debian/Ubuntu desktops.
#
# Run it from the file manager (choose "Execute in Terminal") or from a terminal:
#     bash Run_Linux.sh
#
# First run needs internet and may ask for your password once, to install
# Python packages and to grant access to USB serial ports.

cd "$(dirname "$0")"

pause() { echo ""; read -p "Press Enter to close..."; }

# ── Find Python 3 ─────────────────────────────────────────────────────────────
if ! command -v python3 >/dev/null 2>&1; then
    echo "Python 3 not found."
    echo "Install it with:  sudo apt install python3"
    pause; exit 1
fi
echo "Python found ($(python3 --version 2>&1))."

# ── Install dependencies ──────────────────────────────────────────────────────
# System packages (apt) are used instead of pip: they are prebuilt for every
# Pi architecture (armhf and arm64), install in seconds, and avoid the
# "externally-managed-environment" error newer Raspberry Pi OS gives for pip.
deps_ok() { python3 -c "import serial, matplotlib, tkinter" >/dev/null 2>&1; }

if deps_ok; then
    echo "Dependencies OK."
elif command -v apt-get >/dev/null 2>&1; then
    echo "Installing dependencies (pyserial, matplotlib, tkinter) ..."
    sudo apt-get update -qq && \
    sudo apt-get install -y python3-serial python3-matplotlib python3-tk
    if ! deps_ok; then
        echo "Dependency install failed. Check your internet connection."
        pause; exit 1
    fi
    echo "Dependencies OK."
else
    # Non-Debian Linux: fall back to a private virtual environment.
    echo "apt not found, using a virtual environment instead ..."
    VENV_DIR=".CryoSnap_venv"
    [ -d "$VENV_DIR" ] || python3 -m venv --system-site-packages "$VENV_DIR"
    source "$VENV_DIR/bin/activate"
    python3 -m pip install pyserial matplotlib --quiet
    if ! deps_ok; then
        echo "Dependency install failed. tkinter may need your distro's python3-tk package."
        pause; exit 1
    fi
    echo "Dependencies OK."
fi

# ── Serial port access ────────────────────────────────────────────────────────
# USB serial devices belong to the "dialout" group. Raspberry Pi OS adds the
# default user to it already; other setups may not.
if ! id -nG "$USER" | grep -qw dialout; then
    echo ""
    echo "Your user needs access to USB serial ports (the 'dialout' group)."
    sudo usermod -aG dialout "$USER"
    echo "Done. Log out and back in (or reboot), then run this launcher again."
    pause; exit 0
fi

# ── Run monitor ───────────────────────────────────────────────────────────────
echo ""
echo "Starting CryoSnap Monitor ..."
echo "A window opens with the serial terminal on the left and live graphs on the right."
echo "Type commands in the terminal pane; 'x' + Enter drops and reconnects the serial port."
echo ""
python3 cryosnap_monitor.py

echo ""
echo "Monitor closed."
pause
