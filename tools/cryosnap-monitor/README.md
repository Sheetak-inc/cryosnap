# CryoSnap Monitor

A live monitor for the CryoSnap thermoelectric controller. It connects to the
CryoSnap Nano over USB, plots temperature and power in real time, lets you send
commands to the device, and can log a session to CSV.

## Requirements

- Windows or Mac
- A CryoSnap Nano connected with a USB data cable (not a charge-only cable)
- Python 3 (see below if you don't have it)
- Internet access the first time you run it, to pull in pyserial and matplotlib

## Setup: Python

Skip this if `python3` (Mac) or `python` (Windows) already runs from a terminal.

**Windows**
1. https://www.python.org/downloads/, download and run the installer.
2. On the first installer screen, check "Add python.exe to PATH", then "Install Now".

**Mac**
1. https://www.python.org/downloads/, download and run the installer. Or, if you use Homebrew:
   ```
   brew install python3
   ```

One-time setup, either way. The launcher scripts find Python automatically after this.

## Running it

**Windows:** plug in the Nano, double-click `Run_Windows.bat`.

**Mac:** plug in the Nano. The first time only, the launcher needs permission to
run as a program. Open Terminal in this folder and run:
```
chmod +x Run_Mac.command
```
Then double-click `Run_Mac.command`. Gatekeeper will still block it once as
"from an unidentified developer": right-click the file, choose Open, confirm.
After that, double-click works normally every time.

Either platform, the first run installs pyserial and matplotlib (about a minute, needs internet). After that it starts immediately. The window opens on your second monitor if you have one, serial terminal on the left, live graphs on the right.

Windows may also show a blue "Windows protected your PC" box the first time, this is just because the script isn't signed: click "More info" -> "Run anyway".

## Using the monitor

Layout:

- Left pane: serial terminal. Device output appears here; type commands into the box at the bottom.
- Right pane: live graphs (temperature, power, full-session view), plus a status bar showing current device settings.

Interacting:

- Click the window, type, Enter to send.
- `x` + Enter: drop and reconnect the serial connection (use after unplugging/replugging).
- `note <text>` + Enter: tags the next logged row with your note.

Top-of-graph buttons:

- Log: start/stop writing the session to a timestamped CSV in this folder.
- Run Test: runs the currently selected script from `tests/`.
- Circular arrow: cycles through available test scripts.

Common device commands (typed into the terminal, sent straight to the Nano):

- `status`: full device state snapshot
- `bright <0-255>`: LED brightness
- `fan <0-255>`: fan duty
- `pid <0|1>`: control law (0 = bang-bang, 1 = PID)
- `kp <val>`, `ki <val>`, `kd <val>`: PID gains
- `console`: enter/exit console mode (required before `enable`, `set`, `mode`)
- `enable <0|1>`: TEC drive off/on (console mode required)
- `set <temp_C>`: target setpoint (console mode required)

Close the graph window to exit.

## Tests

`tests/` holds scripts named `test_*.py`; each one shows up as an option for the
Run Test button. Drop in your own `test_yourname.py` and cycle to it with the
arrow button. Current scripts: `test_cryosnap.py`, `test_c1.py`, `test_c1_v1_1.py`,
`test_nopsu.py`, `test_nopsu_bounds.py`, `test_pid1.py`, `test_trunc.py`.

## Data

CSV logs land in this folder as `cryosnap_log_YYYYMMDD_HHMMSS.csv`.

## Troubleshooting

- **"No Nano found"**: check it's a data cable (not charge-only), confirm nothing
  else has the port open (e.g. Arduino Serial Monitor), then `x` + Enter or
  restart the monitor.
- **Nothing plots, or "Unknown: st"**: the monitor auto-resends the start
  command. If it persists, `x` + Enter.
- **"Python was not found"**: Python isn't installed, or (Windows) wasn't added
  to PATH during install, reinstall and check that box.
- **Single monitor**: the window just opens on it; move/resize as normal.
- **Mac Gatekeeper blocks `Run_Mac.command` every time, or it won't open at
  all**: the executable bit is probably missing (common after a zip/email
  round-trip). From Terminal in this folder:
  ```
  chmod +x Run_Mac.command
  ```

## Files in this folder

- `Run_Windows.bat`: Windows launcher, double-click to start
- `Run_Mac.command`: Mac launcher, double-click to start (run `chmod +x` on it first, see above)
- `monitor_cryosnap.ps1`: what the Windows launcher calls (installs deps, starts the monitor)
- `cryosnap_monitor.py`: the monitor program itself, same on both platforms
- `tests/`: test scripts available from the Run Test button
