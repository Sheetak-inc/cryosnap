#!/bin/bash
# CryoSnap Monitor Launcher (macOS)
# Double-click this file in Finder to start the CryoSnap Monitor.
#
# First time only: macOS may block it as "from an unidentified developer."
# Right-click the file, choose Open, then confirm. After that, double-click works normally.

cd "$(dirname "$0")"

# ── Find Python 3 ─────────────────────────────────────────────────────────────
PYTHON=""
for cmd in python3 python; do
    if command -v "$cmd" >/dev/null 2>&1; then
        if "$cmd" --version 2>&1 | grep -q "Python 3"; then
            PYTHON="$cmd"
            break
        fi
    fi
done

if [ -z "$PYTHON" ]; then
    echo ""
    echo "Python 3 not found."
    echo "Download and install it from: https://www.python.org/downloads/"
    echo "(On a Mac you can also run: brew install python3)"
    echo ""
    read -p "Press Enter to close..."
    exit 1
fi

echo "Python found ($PYTHON)."

# ── Install / update dependencies ─────────────────────────────────────────────
echo "Checking dependencies (pyserial, matplotlib) ..."
"$PYTHON" -m pip install pyserial matplotlib --quiet --upgrade

if [ $? -ne 0 ]; then
    echo "pip install failed. Check your internet connection."
    read -p "Press Enter to close..."
    exit 1
fi

echo "Dependencies OK."
echo ""
echo "Starting CryoSnap Monitor ..."
echo "A window opens with the serial terminal on the left and live graphs on the right."
echo "Type commands in the terminal pane; 'x' + Enter drops and reconnects the serial port."
echo ""

# ── Run monitor ────────────────────────────────────────────────────────────────
"$PYTHON" cryosnap_monitor.py

echo ""
echo "Monitor closed."
read -p "Press Enter to close..."
