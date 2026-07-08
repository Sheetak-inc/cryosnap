"""
CryoSnap Serial Monitor v4

Layout (single window):
  Row 0  Temperature graph          |  Temperature stats table
  Row 1  Power + Drive + I(mA)      |  Power stats table
  Row 2  Static / control params bar  (full width)
  Row 3  Full session graph           (≈ 1/6 window height)

Buttons:
  Run Test  (top-left)  — loads test_cryosnap.py if present and runs it
  Log       (top-right) — starts / stops timestamped CSV logging

Terminal commands:
  x              drop the serial port and reconnect to whatever Nano is present
  note <text>    attaches text to next stream row's CSV note column
  anything else  forwarded to device
"""

import csv, importlib.util, queue, re, sys, threading, time
from datetime import datetime
from pathlib import Path

import tkinter as tk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button
import matplotlib.ticker as mticker
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import serial.tools.list_ports

# ── Config ─────────────────────────────────────────────────────────────────────
BAUD           = 115200
WINDOW_SECS    = 120
DISCONN_THRESH = -25.0

TEMP_KEYS        = ["Setpoint_C", "Tcold_C", "Thot_C", "Tamb_C"]
POWER_GRAPH_KEYS = ["P_W", "V_V"]          # left axis only (I_A moved to right)
POWER_TABLE_KEYS = ["P_W", "V_V", "Drive_mA", "I_A", "Fan_RPM"]
STREAM_KEYS      = ["Setpoint_C","Tcold_C","Thot_C","Tamb_C",
                    "Drive_mA","I_A","V_V","P_W","Fan_duty","Fan_RPM"]

TEMP_COLORS  = {"Tcold_C":   "#00cfff",
                "Thot_C":    "#ff4444",
                "Tamb_C":    "#ffaa00",
                "Setpoint_C":"#88ff88"}
POWER_COLORS = {"P_W": "#ffff00", "V_V": "#00ff88"}   # left axis
DRIVE_COLOR  = "#ff8800"    # Drive_mA  — right axis
I_A_COLOR    = "#ff55ff"    # I_A×1000  — right axis
FAN_COLOR    = "#888888"    # Fan_RPM stats table only

POWER_TABLE_COLORS = {**POWER_COLORS,
                      "Drive_mA": DRIVE_COLOR,
                      "I_A":      I_A_COLOR,
                      "Fan_RPM":  FAN_COLOR}

BG    = "#111111"
AX_BG = "#0a0a0a"
TXT   = "#cccccc"

SCRIPT_DIR = Path(__file__).parent

# ── Shared state ───────────────────────────────────────────────────────────────
DATA_LOCK    = threading.Lock()
ROWS         = []
EVER_VALID   = {}
S_MIN        = {}
S_MAX        = {}
STATIC_INFO  = {}
DEVICE_STATE = {"en": "—"}
PENDING_NOTE = {"text": None}
LOG_EVENTS   = []
LOG_STATE    = {"active":False, "path":None, "writer":None, "fh":None}

ser           = None
session_start = None

# ── Test state ─────────────────────────────────────────────────────────────────
TEST_STATE = {"running": False, "thread": None, "stop_event": None}
TEST_DONE  = threading.Event()   # set by test thread; cleared by animate()

# ── Status-block accumulator ───────────────────────────────────────────────────
class _StatusAcc:
    def __init__(self):
        self.active = False; self.lines = []; self.t = 0.0; self.closes = 0

    def start(self, line, t):
        self.active = True; self.lines = [line]; self.t = t; self.closes = 0

    def feed(self, line):
        self.lines.append(line)
        if re.match(r"^-{6,}\s*$", line):
            self.closes += 1
            if self.closes >= 2: return self._flush()
        return None

    def _flush(self):
        text = " — ".join(re.sub(r"\t+", " ", ln).strip()
                          for ln in self.lines if ln.strip())
        r = (self.t, "status", text)
        self.active = False; self.lines = []; self.closes = 0
        return r

    def force_flush(self):
        return self._flush() if (self.active and self.lines) else None

STATUS_ACC = _StatusAcc()

# ── Helpers ────────────────────────────────────────────────────────────────────

def find_port():
    usb = [p for p in serial.tools.list_ports.comports()
           if "USB" in (p.description or "")]
    if usb: return usb[0].device
    all_p = list(serial.tools.list_ports.comports())
    return all_p[0].device if all_p else None

def parse_stream(line):
    out = {}
    for m in re.finditer(r"([\w_]+):\s*(-?[\d.]+)", line):
        try: out[m.group(1)] = float(m.group(2))
        except ValueError: pass
    return out

def update_bounds(key, val):
    if key not in S_MIN or val < S_MIN[key]: S_MIN[key] = val
    if key not in S_MAX or val > S_MAX[key]: S_MAX[key] = val

def padded_lim(lo, hi, pct=0.08, minpad=0.5):
    pad = max(minpad, (hi - lo) * pct)
    return lo - pad, hi + pad

def elapsed():
    return (time.time() - session_start) if session_start else 0.0

def categorize_event(line):
    ll = line.lower()
    if "error" in ll or "fault" in ll: return "error"
    if "warn"  in ll or ll.startswith("diag"): return "warning"
    if "--- status" in ll or "--- reg" in ll: return "status"
    return "init"

# ── Parsing ────────────────────────────────────────────────────────────────────
# Called on EVERY non-stream line so terminal-echoed changes are caught immediately.

def try_parse_static(line):
    si = STATIC_INFO

    m = re.search(r"CryoSnap\s+(v[\S]+)", line, re.I)
    if m: si["version"] = m.group(1)

    m = re.search(r"PD:\s*(\d+V/\d+mA)", line, re.I)
    if m: si["pd"] = f"PD:{m.group(1)}"
    m = re.search(r"(\d+V)\s*/\s*(\d+A)\s+already negotiated", line, re.I)
    if m: si["pd"] = f"PD:{m.group(1)}/{m.group(2)}"

    # Each field below matches BOTH the full status dump and the short reply the
    # device echoes for a single command (e.g. "Bright=10", "DB=0.20"), so the
    # green bar updates from those replies without re-querying status.

    # Vmax — status "Vmax=12000mV", reply "Vmax=12000"
    m = re.search(r"Vmax=(\d+)", line, re.I)
    if m: si["vmax"] = f"Vmax:{float(m.group(1))/1000:.1f}V"

    # Fan duty (status line). Stream overwrites this with the live value each row.
    m = re.search(r"(?<!\w)Fan=(\d+)", line, re.I)
    if m: si.setdefault("fan", f"Fan:{m.group(1)}")

    # LED brightness — status "Br=64", reply "Bright=10"
    m = re.search(r"\bBr(?:ight)?=(\d+)", line, re.I)
    if m: si["br"] = f"Br:{m.group(1)}"

    # Control law — status "Ctrl:PID"/"Ctrl:BB", reply "PID:1"/"PID:0"
    m = re.search(r"Ctrl:(\w+)", line, re.I)
    if m: si["ctrl"] = m.group(1).upper()
    m = re.search(r"(?<!\w)PID:(\d)", line)
    if m: si["ctrl"] = "PID" if m.group(1) == "1" else "BB"

    # PID gains — status has all three on one line; replies arrive individually
    def _trim(v): return (v.rstrip("0").rstrip(".") if "." in v else v) or "0"
    m = re.search(r"Kp=([\d.]+)", line, re.I)
    if m: si["kp"] = _trim(m.group(1))
    m = re.search(r"Ki=([\d.]+)", line, re.I)
    if m: si["ki"] = _trim(m.group(1))
    m = re.search(r"Kd=([\d.]+)", line, re.I)
    if m: si["kd"] = _trim(m.group(1))

    # Deadband — status "DB=0.20C", reply "DB=0.20"
    m = re.search(r"DB=(-?[\d.]+)C?", line, re.I)
    if m: si["db"] = f"DB:{m.group(1)}°C"

    # Damping width + percent — status "Damp=3.00C/50%", replies "Damp=3.00" / "Dpct=50"
    m = re.search(r"Damp=([\d.]+)", line, re.I)
    if m: si["damp_c"] = m.group(1)
    m = re.search(r"Damp=[\d.]+C?/(\d+)%", line, re.I)
    if m: si["damp_pct"] = m.group(1)
    m = re.search(r"Dpct=(\d+)", line, re.I)
    if m: si["damp_pct"] = m.group(1)
    if "damp_c" in si or "damp_pct" in si:
        si["damp"] = f"Damp:{si.get('damp_c','?')}°/{si.get('damp_pct','?')}%"

    # Hot-side cutoff — status "MaxHot=60.0C", reply "MaxHot=60.0"
    m = re.search(r"MaxHot=([\d.]+)C?", line, re.I)
    if m: si["maxhot"] = f"MaxHot:{m.group(1)}°C"

    # Imax — accept with or without mA suffix
    m = re.search(r"Imax=(\d+)(?:mA)?", line, re.I)
    if m: si["imax"] = f"Imax:{m.group(1)}mA"

    if "I2C scan done" in line:
        m = re.search(r"(\d+)\s+device", line)
        if m: si["i2c"] = f"I2C:{m.group(1)}dev"

CONSOLE_ON = False   # tracks whether device console mode is currently active

def try_parse_state(line):
    global CONSOLE_ON
    # Enable state — status line "En:0", command reply "Enable:1"/"Enable:0"
    m = re.search(r"(?<!\w)En(?:able)?:(\d)", line, re.I)
    if m: DEVICE_STATE["en"] = "YES" if m.group(1) == "1" else "NO"
    ll = line.lower()
    if "enable: on"  in ll: DEVICE_STATE["en"] = "YES"
    if "enable: off" in ll: DEVICE_STATE["en"] = "NO"
    if "Console:1" in line: CONSOLE_ON = True
    if "Console:0" in line: CONSOLE_ON = False

def _send_console_off():
    """Turn off console mode if it's currently on (called after test ends / window closes)."""
    global CONSOLE_ON
    if CONSOLE_ON and ser and ser.is_open:
        try:
            ser.write(b"console\r\n")
            CONSOLE_ON = False
            print("[INFO]  console mode disabled")
        except Exception:
            pass

# ── CSV logging ────────────────────────────────────────────────────────────────
CSV_HEADERS = ["Time_s"] + STREAM_KEYS + ["note","init","status","error","warning"]

def log_open():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = SCRIPT_DIR / f"cryosnap_log_{ts}.csv"
    fh = open(path, "w", newline="", encoding="utf-8")
    wr = csv.DictWriter(fh, fieldnames=CSV_HEADERS, extrasaction="ignore")
    wr.writeheader()
    LOG_STATE.update({"fh":fh, "writer":wr, "path":path})
    for row in ROWS:      _wrow(row)
    for ev in LOG_EVENTS: _wevent(*ev)
    fh.flush()
    print(f"[LOG] → {path.name}")

def log_close():
    if LOG_STATE["fh"]: LOG_STATE["fh"].flush(); LOG_STATE["fh"].close()
    LOG_STATE.update({"fh":None, "writer":None, "active":False})
    print("[LOG] Stopped.")

def _wrow(row):
    if not LOG_STATE["writer"]: return
    out = {k:"" for k in CSV_HEADERS}
    out["Time_s"] = f"{row.get('t',0):.3f}"
    for k in STREAM_KEYS:
        v = row.get(k)
        if v is not None: out[k] = v
    out["note"] = row.get("note","")
    LOG_STATE["writer"].writerow(out)

def _wevent(t, etype, msg):
    if not LOG_STATE["writer"]: return
    out = {k:"" for k in CSV_HEADERS}
    out["Time_s"] = f"{t:.3f}"
    out[etype if etype in CSV_HEADERS else "init"] = msg
    LOG_STATE["writer"].writerow(out)

def _queue_event(t, etype, msg):
    with DATA_LOCK:
        LOG_EVENTS.append((t, etype, msg))
        if LOG_STATE["active"]:
            _wevent(t, etype, msg)
            LOG_STATE["fh"].flush()

def log_toggle(_evt=None):
    with DATA_LOCK:
        if not LOG_STATE["active"]:
            LOG_STATE["active"] = True
            log_open()
            btn_log.label.set_text("LOGGING")
            btn_log.color = "#6a1500"; btn_log.hovercolor = "#882200"
        else:
            log_close()
            btn_log.label.set_text("Log")
            btn_log.color = "#1a3320"; btn_log.hovercolor = "#2a4430"
    fig1.canvas.draw_idle()

# ── Non-stream line router ─────────────────────────────────────────────────────

def process_nonstream(line, t):
    # "--- status ---" always starts a fresh buffer (force-flush any stale one).
    # "--- registers ---" is part of the SAME status dump — if STATUS_ACC is
    # already active, feed it in rather than restarting. Restarting caused
    # registers to only ever accumulate 1 close, keeping STATUS_ACC permanently
    # active and swallowing every subsequent non-stream line into the buffer.
    if "--- status ---" in line:
        stale = STATUS_ACC.force_flush()
        if stale: _queue_event(*stale)
        STATUS_ACC.start(line, t)
        return

    if STATUS_ACC.active:
        result = STATUS_ACC.feed(line)
        if result: _queue_event(*result)
        return

    # Registers block arriving without a preceding status block (unusual).
    if "--- registers ---" in line:
        STATUS_ACC.start(line, t)
        return

    _queue_event(t, categorize_event(line), line)

# ── Test infrastructure ────────────────────────────────────────────────────────

class TestContext:
    """Passed to run_test(ctx) in test_cryosnap.py."""
    def __init__(self, stop_event):
        self.stop_event = stop_event
        self.report     = []

    def send(self, cmd):
        print(f"[TEST] > {cmd}")
        if ser and ser.is_open:
            ser.write((cmd + "\r\n").encode())

    def wait(self, seconds):
        """Sleep up to seconds. Returns True if test was externally stopped."""
        return self.stop_event.wait(seconds)

    def is_stopped(self):
        return self.stop_event.is_set()

    def latest(self):
        with DATA_LOCK:
            return dict(ROWS[-1]) if ROWS else {}

    def get_static(self):
        """Snapshot of STATIC_INFO (version, imax, vmax, etc.)."""
        return dict(STATIC_INFO)

    def now(self):
        """Current elapsed time in seconds — same timescale as stream rows and LOG_EVENTS."""
        return elapsed()

    def recent_messages(self, n=20):
        """Last n log events as (t, etype, msg). Prefer all_events_since() for reliable checks."""
        with DATA_LOCK:
            return list(LOG_EVENTS[-n:])

    def all_events_since(self, t_cutoff):
        """All log events (any type) with timestamp >= t_cutoff as (etype, msg) pairs."""
        with DATA_LOCK:
            return [(et, msg) for ts, et, msg in LOG_EVENTS if ts >= t_cutoff]

    def log_index(self):
        """Current length of LOG_EVENTS. Capture before sending a command,
        then pass to events_after() to get only responses to that command."""
        with DATA_LOCK:
            return len(LOG_EVENTS)

    def events_after(self, index):
        """All events appended after the given index, as (etype, msg) pairs.
        Index-based — immune to timestamp races."""
        with DATA_LOCK:
            return [(et, msg) for _, et, msg in LOG_EVENTS[index:]]

    def rows_since(self, t_cutoff):
        with DATA_LOCK:
            return [dict(r) for r in ROWS if r.get("t",0) >= t_cutoff]

    def errors_since(self, t_cutoff):
        """Return list of (etype, msg) error/fault events since t_cutoff."""
        with DATA_LOCK:
            return [(et, msg) for ts, et, msg in LOG_EVENTS
                    if ts >= t_cutoff and et in ("error","warning")]

    def log(self, msg):
        self.report.append(msg)
        print(f"[TEST] {msg}")

    def wait_for_stable(self, key, target, tolerance=0.5, max_seconds=120):
        """Wait until a stream value is within tolerance of target."""
        steps = int(max_seconds * 2)
        for _ in range(steps):
            if self.is_stopped(): return False
            self.wait(0.5)
            val = self.latest().get(key)
            if val is not None and abs(val - target) <= tolerance:
                return True
        return False


_selected_test = [0]   # index into sorted test_*.py list (mutable for closure)


def _test_files():
    """All test_*.py files in the script folder, sorted by name."""
    return sorted((SCRIPT_DIR / "tests").glob("test_*.py"))


def _test_btn_color():
    return "#1a2a3a" if _test_files() else "#252525"


def _test_btn_label():
    files = _test_files()
    if not files:
        return "Run Test"
    idx = _selected_test[0] % len(files)
    # Show short name: "test_nopsu.py" → "nopsu"
    stem = files[idx].stem
    name = stem[5:] if stem.startswith("test_") else stem
    return f"▶ {name}" if len(files) == 1 else f"▶ {name} ({idx+1}/{len(files)})"


def run_test_clicked(_evt=None):
    files = _test_files()
    if not TEST_STATE["running"]:
        if not files:
            print("[TEST] No test_*.py files found in script folder.")
            return

        # If there are multiple tests and we're not running, first click cycles;
        # only run if we're already on the selected test (button already shows it).
        # Simpler: single click always runs the currently selected test.
        # Ctrl+click (detected as second rapid click) cycles. Not possible in
        # matplotlib easily — so: left-click runs, we add a separate "next" area.
        # Practical solution: just run whichever test is currently selected.
        idx = _selected_test[0] % len(files)
        test_file = files[idx]

        stop_ev = threading.Event()
        ctx     = TestContext(stop_ev)
        TEST_STATE.update({"running":True, "stop_event":stop_ev})

        def _run():
            mod_name = test_file.stem
            spec = importlib.util.spec_from_file_location(mod_name, test_file)
            mod  = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            try:
                mod.run_test(ctx)
            except Exception as e:
                ctx.log(f"Test error: {e}")
            finally:
                print("\n" + "="*52)
                print("  TEST REPORT")
                print("="*52)
                for ln in ctx.report:
                    print(f"  {ln}")
                print("="*52 + "\n")
                _send_console_off()
                TEST_DONE.set()

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        TEST_STATE["thread"] = t
        btn_test.label.set_text("Stop Test")
        btn_test.color = "#6a1500"; btn_test.hovercolor = "#882200"
        fig1.canvas.draw_idle()
    else:
        TEST_STATE["stop_event"].set()
        btn_test.label.set_text(_test_btn_label())
        btn_test.color = _test_btn_color(); btn_test.hovercolor = "#2a3a4a"
        TEST_STATE["running"] = False
        fig1.canvas.draw_idle()


def cycle_test_clicked(_evt=None):
    """Cycle through available tests without running."""
    if not TEST_STATE["running"]:
        files = _test_files()
        if len(files) > 1:
            _selected_test[0] = (_selected_test[0] + 1) % len(files)
            btn_test.label.set_text(_test_btn_label())
            btn_test.color = _test_btn_color()
            fig1.canvas.draw_idle()

# ── Serial thread ──────────────────────────────────────────────────────────────

RECONNECT = threading.Event()   # set by input_reader on "x"+Enter — drop & reconnect


def _boot_handshake():
    """Read boot output until the device is ready (or assumed already running).
    Returns True when ready to start streaming, False if a reconnect was requested
    (or the port errored) mid-handshake.

    Boot handshake. Older firmware ended boot with "ready. 'help' for commands",
    and the monitor triggered on that "help" string to start streaming. To save
    flash, newer firmware (v0.7.11+) boots nearly silently, so that string never
    arrives and the monitor would otherwise hang here forever. Instead: detect
    the version banner, then wait for the serial line to fall quiet (boot output
    finished) before kicking things off. The legacy "help" hint still triggers
    immediately, and a timeout covers the case where the device booted before we
    connected (e.g. an 'x' reconnect to an already-running Nano).
    """
    booted      = False   # have we seen the "CryoSnap v…" banner?
    quiet_reads = 0       # consecutive ~1 s read timeouts after the banner
    wait_start  = time.time()
    while True:
        if RECONNECT.is_set():
            return False
        try:
            raw = ser.readline()
        except Exception as e:
            print(f"[ERROR] {e}"); return False
        if raw:
            line = raw.decode("utf-8", errors="replace").rstrip()
            if line:
                print(line)
                try_parse_static(line); try_parse_state(line)
                process_nonstream(line, elapsed())
                if re.search(r"CryoSnap\s+v", line, re.I): booted = True
                quiet_reads = 0
            if "help" in line.lower():        # legacy fast path
                return True
        else:
            # readline timed out (~1 s of silence)
            if booted:
                quiet_reads += 1
                if quiet_reads >= 2: return True   # boot output has finished
            elif time.time() - wait_start > 8:
                print("[INFO]  No boot banner seen — assuming already running.")
                return True


def _safe_close():
    """Close the serial port, ignoring errors. Safe to call from any thread —
    closing here also unblocks a readline() that's hung on a vanished device,
    which is what lets an 'x' reconnect take effect immediately."""
    global ser
    try:
        if ser and ser.is_open: ser.close()
    except Exception: pass


def _serial_session():
    """One connect → handshake → stream cycle. Returns when the link should be
    re-established: no port, handshake aborted, read error, or 'x' pressed."""
    global ser, session_start
    RECONNECT.clear()
    try:
        port = find_port()
    except Exception as e:
        print(f"[ERROR] port scan failed: {e}"); port = None
    if not port:
        print("[ERROR] No Nano found. Plug one in (or press 'x'+Enter to retry) …")
        RECONNECT.wait(2.0); return
    print(f"[INFO]  Connecting to {port} @ {BAUD} …")
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except Exception as e:
        print(f"[ERROR] {e}"); RECONNECT.wait(2.0); return

    print("[INFO]  Waiting for device ready …")
    if not _boot_handshake():
        _safe_close(); return

    # Newer firmware no longer auto-hints "help", so drive the handshake here:
    # one status dump to populate the static params bar, then start streaming.
    # On boot the Nano occasionally swallows part of a command (it comes back as
    # e.g. "Unknown: st" for "stream"). Guard against that: clear any boot remnants,
    # send a lone newline first so a stray partial line can't merge with our command,
    # write each command, and flush() so every byte is transmitted before the next.
    try:
        try: ser.reset_input_buffer()
        except Exception: pass
        ser.write(b"\r\n"); ser.flush(); time.sleep(0.2)
        ser.write(b"status\r\n"); ser.flush(); print("> status  [sent]")
        time.sleep(0.5)
        ser.write(b"stream\r\n"); ser.flush(); print("> stream  [sent]")
    except Exception as e:
        print(f"[ERROR] {e}"); _safe_close(); return
    # Keep the session timeline continuous across reconnects.
    if session_start is None:
        session_start = time.time()

    stream_ok      = False          # has streaming actually started?
    stream_sent_at = time.time()
    stream_retries = 0

    while True:
        if RECONNECT.is_set():
            print("[INFO]  Dropping serial connection — reconnecting …")
            break
        # Boot-time safety net: if 'stream' was garbled on the wire, no data
        # arrives — re-send it. Only while nothing has streamed yet (and capped at
        # 3 tries), so a working stream is never toggled back off.
        if not stream_ok and stream_retries < 3 and (time.time() - stream_sent_at) > 3.0:
            try:
                ser.reset_input_buffer()
                ser.write(b"\r\nstream\r\n"); ser.flush()
            except Exception: pass
            stream_retries += 1
            stream_sent_at = time.time()
            print(f"[INFO]  No stream yet — re-sending 'stream' (try {stream_retries}) …")
        try:
            raw = ser.readline()
            if not raw: continue
            line = raw.decode("utf-8", errors="replace").rstrip()
            if not line: continue

            if line.startswith("Setpoint_C:"):
                stream_ok = True
                parsed = parse_stream(line)
                if parsed:
                    t = elapsed(); row = {"t": t}
                    with DATA_LOCK:
                        for key in TEMP_KEYS:
                            val = parsed.get(key)
                            if val is not None:
                                if val > DISCONN_THRESH: EVER_VALID[key] = True
                                row[key] = val if EVER_VALID.get(key) else None
                                if EVER_VALID.get(key) and val > DISCONN_THRESH:
                                    update_bounds(key, val)
                        for key in ["Drive_mA"] + POWER_GRAPH_KEYS + ["I_A","Fan_duty","Fan_RPM"]:
                            val = parsed.get(key)
                            if val is not None:
                                row[key] = val; update_bounds(key, val)
                        # Live horizontal bar updates
                        if "Fan_duty"   in parsed: STATIC_INFO["fan"] = f"Fan:{int(parsed['Fan_duty'])}"
                        if PENDING_NOTE["text"]:
                            row["note"] = PENDING_NOTE["text"]; PENDING_NOTE["text"] = None
                        ROWS.append(row)
                        if LOG_STATE["active"]:
                            _wrow(row); LOG_STATE["fh"].flush()
            else:
                if "Stream:1" in line: stream_ok = True
                print(line)
                try_parse_static(line); try_parse_state(line)
                process_nonstream(line, elapsed())

        except Exception as e:
            print(f"[SERIAL ERR] {e}")
            break

    _safe_close()


def serial_reader():
    # Immortal supervisor: when a session ends (or hits any unexpected error) we
    # just loop back into a fresh connect attempt, so the reader thread never dies
    # and an 'x' reconnect always has something listening for it.
    while True:
        try:
            _serial_session()
        except Exception as e:
            print(f"[SERIAL ERR] reader loop: {e}")
            _safe_close()
            time.sleep(0.5)

# ── Input thread ───────────────────────────────────────────────────────────────

def input_reader():
    while True:
        try:
            cmd = sys.stdin.readline()
            if not cmd: break
            cmd = cmd.rstrip()
            if cmd.lower() == "x":
                print("[RECONNECT] Dropping serial port and reconnecting to whatever Nano is present …")
                RECONNECT.set()
                _safe_close()
            elif cmd.lower().startswith("note "):
                with DATA_LOCK: PENDING_NOTE["text"] = cmd[5:].strip()
                print(f"[NOTE queued: {cmd[5:].strip()}]")
            else:
                print(f"> {cmd}")
                if ser and ser.is_open: ser.write((cmd + "\r\n").encode())
        except Exception as e: print(f"[INPUT ERR] {e}")

# ── Single window: serial terminal (left 22%) | live graphs (right 78%) ─────────
root = tk.Tk()
root.title("CryoSnap Live Monitor")
root.configure(bg=BG)

root.columnconfigure(0, weight=22, uniform="cols")
root.columnconfigure(1, weight=78, uniform="cols")
root.rowconfigure(0, weight=1)

term_frame  = tk.Frame(root, bg=BG)
term_frame.grid(row=0, column=0, sticky="nsew")
graph_frame = tk.Frame(root, bg=BG)
graph_frame.grid(row=0, column=1, sticky="nsew")

# Embedded serial terminal: scrollable output log + command entry.
tk.Label(term_frame, text="Serial Terminal", bg=BG, fg="#888888", anchor="w",
         font=("Consolas", 9, "bold")).pack(side=tk.TOP, fill=tk.X, padx=4, pady=(4, 0))
term_out = tk.Text(term_frame, bg="#000000", fg="#cccccc", insertbackground="#cccccc",
                   font=("Consolas", 9), wrap="word", borderwidth=0, highlightthickness=0)
term_out.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=2)
term_out.configure(state="disabled")
term_in = tk.Entry(term_frame, bg="#0a0a0a", fg="#ffffff", insertbackground="#ffffff",
                   font=("Consolas", 10), borderwidth=0, highlightthickness=1,
                   highlightbackground="#333333", highlightcolor="#5599ff")
term_in.pack(side=tk.BOTTOM, fill=tk.X, padx=4, pady=4)

# Live monitor figure embedded in the right pane.
fig1 = Figure(figsize=(15, 10), facecolor=BG)
_canvas = FigureCanvasTkAgg(fig1, master=graph_frame)
_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

gs = gridspec.GridSpec(
    4, 2, figure=fig1,
    height_ratios=[3.0, 2.5, 0.45, 1.2],
    width_ratios=[3.5, 1.0],
    hspace=0.55, wspace=0.06,
    left=0.07, right=0.97, top=0.92, bottom=0.06,
)

ax_temp   = fig1.add_subplot(gs[0, 0])
ax_tstats = fig1.add_subplot(gs[0, 1])
ax_power  = fig1.add_subplot(gs[1, 0])   # left axis #1 — Volts (green)
ax_drive  = ax_power.twinx()         # right axis for Drive_mA + I_A×1000 — created ONCE
ax_pw     = ax_power.twinx()         # left axis #2 — Watts (yellow), offset further left
ax_pstats = fig1.add_subplot(gs[1, 1])
ax_static = fig1.add_subplot(gs[2, :])
ax_sess   = fig1.add_subplot(gs[3, :])

fig1.suptitle("CryoSnap Live Monitor", color="white", fontsize=12, y=0.965)

# Log button — top right
ax_lbtn = fig1.add_axes([0.882, 0.938, 0.092, 0.038])
btn_log  = Button(ax_lbtn, "Log", color="#1a3320", hovercolor="#2a4430")
btn_log.label.set_color("white"); btn_log.label.set_fontsize(9)
btn_log.on_clicked(log_toggle)

# Run Test button — top left (shows selected test name)
ax_tbtn  = fig1.add_axes([0.07, 0.938, 0.115, 0.038])
btn_test = Button(ax_tbtn, _test_btn_label(), color=_test_btn_color(), hovercolor="#2a3a4a")
btn_test.label.set_color("white"); btn_test.label.set_fontsize(8)
btn_test.on_clicked(run_test_clicked)

# Cycle button — steps through test_*.py files without running
ax_cycle  = fig1.add_axes([0.188, 0.938, 0.030, 0.038])
btn_cycle = Button(ax_cycle, "⟳", color="#1a2233", hovercolor="#2a3244")
btn_cycle.label.set_color("white"); btn_cycle.label.set_fontsize(10)
btn_cycle.on_clicked(cycle_test_clicked)

for ax in [ax_temp, ax_power, ax_sess]:
    ax.set_facecolor(AX_BG)
    ax.tick_params(colors="#aaaaaa", labelsize=7)
    for sp in ax.spines.values(): sp.set_edgecolor("#333333")

ax_drive.set_facecolor("none")

# Watts axis: second LEFT scale, spine pushed outward so it sits left of the Volts ticks.
ax_pw.set_facecolor("none")
ax_pw.spines["left"].set_position(("outward", 40))
ax_pw.spines["left"].set_visible(True)
ax_pw.spines["right"].set_visible(False)
ax_pw.yaxis.set_ticks_position("left")
ax_pw.yaxis.set_label_position("left")

for ax in [ax_tstats, ax_pstats]:
    ax.set_facecolor(BG); ax.axis("off")

ax_static.set_facecolor("#0c1a0c")
ax_static.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
for sp in ax_static.spines.values(): sp.set_edgecolor("#224422"); sp.set_linewidth(0.8)

# ── Stats table renderer ───────────────────────────────────────────────────────

def fmt_val(v):
    if v is None: return "—"
    a = abs(v)
    if a >= 10000: return f"{v:.0f}"
    if a >= 1000:  return f"{v:.0f}"
    if a >= 100:   return f"{v:.1f}"
    if a >= 10:    return f"{v:.2f}"
    return f"{v:.3f}"

def draw_stats_table(ax, table_keys, colors, win_rows, col_label, bottom_text):
    ax.cla(); ax.axis("off"); ax.set_facecolor(BG)
    w_min, w_max = {}, {}
    for key in table_keys:
        vals = [r[key] for r in win_rows if r.get(key) is not None]
        if vals: w_min[key] = min(vals); w_max[key] = max(vals)

    cx = [0.02, 0.34, 0.60, 0.83]
    for i, h in enumerate([col_label, "Live", "Min", "Max"]):
        ax.text(cx[i], 0.97, h, transform=ax.transAxes,
                color="#666666" if i > 0 else "#aaaaaa",
                fontsize=7, va="top", fontweight="bold")
    ax.axhline(0.88, color="#2a2a2a", linewidth=0.6)

    n = len(table_keys); row_h = 0.68 / max(n, 1)
    for i, key in enumerate(table_keys):
        y     = 0.83 - i * row_h
        color = colors.get(key, TXT)
        label = re.sub(r"_(C|mA|RPM|duty)$", "", key)
        live  = ROWS[-1].get(key) if ROWS else None
        ax.text(cx[0], y, label,               transform=ax.transAxes,
                color=color, fontsize=7.5, va="center", fontweight="bold")
        ax.text(cx[1], y, fmt_val(live),       transform=ax.transAxes,
                color=color, fontsize=7.5, va="center")
        ax.text(cx[2], y, fmt_val(w_min.get(key)), transform=ax.transAxes,
                color="#888888", fontsize=7, va="center")
        ax.text(cx[3], y, fmt_val(w_max.get(key)), transform=ax.transAxes,
                color="#888888", fontsize=7, va="center")

    ax.axhline(0.10, color="#2a2a2a", linewidth=0.6)
    ax.text(0.02, 0.04, bottom_text, transform=ax.transAxes,
            color="#dddddd", fontsize=7.5, va="center")

# ── Static params bar ──────────────────────────────────────────────────────────

def draw_static_params(ax):
    ax.cla(); ax.set_facecolor("#0c1a0c")
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    for sp in ax.spines.values(): sp.set_edgecolor("#224422"); sp.set_linewidth(0.8)

    si = STATIC_INFO; parts = []
    for k in ["version","pd","vmax","fan","br"]:   # sp removed (shown in temp table)
        if k in si: parts.append(si[k])

    ctrl_parts = []
    if "ctrl" in si: ctrl_parts.append(f"Ctrl:{si['ctrl']}")
    if "kp"   in si: ctrl_parts.append(f"Kp:{si['kp']}")
    if "ki"   in si: ctrl_parts.append(f"Ki:{si['ki']}")
    if "kd"   in si: ctrl_parts.append(f"Kd:{si['kd']}")
    if ctrl_parts: parts.append(" ".join(ctrl_parts))

    db_damp = []
    if "db"   in si: db_damp.append(si["db"])
    if "damp" in si: db_damp.append(si["damp"])
    if db_damp: parts.append(" ".join(db_damp))

    for k in ["maxhot","imax","i2c"]:
        if k in si: parts.append(si[k])

    text = "  |  ".join(parts) if parts else "Waiting for device info…"
    ax.text(0.008, 0.5, text, transform=ax.transAxes,
            color="#aaffaa", fontsize=7.5, va="center",
            fontfamily="monospace", clip_on=True)

# ── Animation ──────────────────────────────────────────────────────────────────

def style_plot(ax, title, ylabel, xlabel=None):
    ax.cla(); ax.set_facecolor(AX_BG)
    ax.set_title(title, color="white", fontsize=9, pad=3)
    ax.set_ylabel(ylabel, color=TXT, fontsize=8)
    if xlabel: ax.set_xlabel(xlabel, color=TXT, fontsize=8)
    ax.tick_params(colors="#aaaaaa", labelsize=7)
    for sp in ax.spines.values(): sp.set_edgecolor("#333333")

def apply_ylim(ax, keys, scale=1.0):
    lo = [S_MIN[k]*scale for k in keys if k in S_MIN]
    hi = [S_MAX[k]*scale for k in keys if k in S_MAX]
    if lo and hi: ax.set_ylim(*padded_lim(min(lo), max(hi)))

def small_legend(ax, extra_h=None, extra_l=None):
    h, l = ax.get_legend_handles_labels()
    if extra_h: h += extra_h
    if extra_l: l += extra_l
    leg = ax.legend(h, l, loc="upper left", fontsize=7, facecolor="#1e1e1e",
                    labelcolor="white", edgecolor="#444", framealpha=0.85)
    if leg:
        leg.set_zorder(100)   # always on top of plotted lines


def animate(_frame):
    # Handle test completion (update button from main thread)
    if TEST_DONE.is_set():
        TEST_DONE.clear()
        TEST_STATE["running"] = False
        btn_test.label.set_text(_test_btn_label())
        btn_test.color = _test_btn_color()
        btn_test.hovercolor = "#2a3a4a"

    # Refresh Run Test button label + color when not running
    if not TEST_STATE["running"]:
        new_label = _test_btn_label()
        new_color = _test_btn_color()
        if btn_test.label.get_text() != new_label:
            btn_test.label.set_text(new_label)
        if btn_test.color != new_color:
            btn_test.color = new_color

    with DATA_LOCK:
        draw_static_params(ax_static)
        if not ROWS: return

        now       = ROWS[-1]["t"]
        win_start = now - WINDOW_SECS
        win_rows  = [r for r in ROWS if r["t"] >= win_start]

        last    = ROWS[-1]
        iA      = last.get("I_A") or 0.0
        tps_str = "ON" if abs(iA) > 0.05 else "OFF"

        # ── Temperature graph ─────────────────────────────────────────
        style_plot(ax_temp, "Temperatures (°C)", "°C")
        plotted_t = False
        for key, color in TEMP_COLORS.items():
            pairs = [(r["t"], r[key]) for r in win_rows if r.get(key) is not None]
            if pairs:
                tx, vy = zip(*pairs)
                ax_temp.plot(tx, vy, color=color,
                             label=key.replace("_C",""), linewidth=1.5)
                plotted_t = True
        ax_temp.set_xlim(win_start, now)
        if plotted_t:
            apply_ylim(ax_temp, list(TEMP_COLORS))
            small_legend(ax_temp)

        draw_stats_table(ax_tstats, list(TEMP_COLORS), TEMP_COLORS, win_rows,
                         "Temp °C", f"TPS: {tps_str}")

        # ── Power graph: two LEFT scales — V (green) and W (yellow), each 0 →
        #    session max with 1-decimal ticks; right axis = Drive mA + I mA ─────
        style_plot(ax_power, "", "", "Elapsed (s)")

        # Left axis #1 — Volts (green ticks), at the normal left spine
        ax_power.yaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{x:g}"))
        ax_power.yaxis.set_major_locator(mticker.MaxNLocator(nbins=6))
        ax_power.tick_params(axis="y", colors=POWER_COLORS["V_V"], labelsize=7,
                             left=True, labelleft=True)
        ax_power.spines["left"].set_edgecolor(POWER_COLORS["V_V"])

        # Left axis #2 — Watts (yellow ticks), spine offset further to the left
        for ln in list(ax_pw.get_lines()): ln.remove()
        ax_pw.set_facecolor("none")
        ax_pw.yaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{x:g}"))
        ax_pw.yaxis.set_major_locator(mticker.MaxNLocator(nbins=6))
        ax_pw.tick_params(axis="y", colors=POWER_COLORS["P_W"], labelsize=7,
                          left=True, labelleft=True, right=False, labelright=False)
        ax_pw.spines["left"].set_edgecolor(POWER_COLORS["P_W"])

        # Right axis — Drive mA + I mA (unchanged)
        for ln in list(ax_drive.get_lines()): ln.remove()
        try: ax_drive.get_legend().remove()
        except Exception: pass
        ax_drive.set_facecolor("none")
        ax_drive.set_ylabel("")                        # no right-axis title
        ax_drive.tick_params(
            color=I_A_COLOR,           # tick marks → pink (matches I_A line)
            labelcolor=DRIVE_COLOR,    # tick numbers → orange (matches Drive line)
            labelsize=7,
            right=True, left=False, labelleft=False
        )
        ax_drive.yaxis.set_major_formatter(
            mticker.FuncFormatter(lambda x, _: f"{x/1000:g}")
        )
        for nm, sp in ax_drive.spines.items():
            sp.set_edgecolor("#444444" if nm == "right" else "#333333")

        # Volts on left axis #1, Watts on left axis #2
        hp = None
        vpairs = [(r["t"], r["V_V"]) for r in win_rows if r.get("V_V") is not None]
        if vpairs:
            tx, vy = zip(*vpairs)
            ax_power.plot(tx, vy, color=POWER_COLORS["V_V"], label="V_V", linewidth=1.5)
        ppairs = [(r["t"], r["P_W"]) for r in win_rows if r.get("P_W") is not None]
        if ppairs:
            tx, py = zip(*ppairs)
            hp, = ax_pw.plot(tx, py, color=POWER_COLORS["P_W"], label="P_W", linewidth=1.5)

        # Right axis: Drive_mA and I_A×1000 (both in mA scale)
        right_h, right_l = [], []
        for key, color, label, scale in [
            ("Drive_mA", DRIVE_COLOR, "Drive mA", 1.0),
            ("I_A",      I_A_COLOR,   "I (mA)",   1000.0),
        ]:
            pairs = [(r["t"], r[key] * scale) for r in win_rows if r.get(key) is not None]
            if pairs:
                tx, vy = zip(*pairs)
                ln, = ax_drive.plot(tx, vy, color=color, label=label, linewidth=1.5)
                right_h.append(ln); right_l.append(label)

        ax_power.set_xlim(win_start, now)
        ax_pw.set_xlim(win_start, now)
        ax_drive.set_xlim(win_start, now)

        # Each left scale: 0 → session max (5% headroom so the peak is visible)
        vmax = S_MAX.get("V_V")
        ax_power.set_ylim(0, vmax * 1.05 if (vmax and vmax > 0) else 1.0)
        pmax = S_MAX.get("P_W")
        ax_pw.set_ylim(0, pmax * 1.05 if (pmax and pmax > 0) else 1.0)

        # Session-locked right axis: combined Drive_mA and I_A×1000 range
        right_lo = [S_MIN[k] * sc for k, sc in [("Drive_mA",1.0),("I_A",1000.0)] if k in S_MIN]
        right_hi = [S_MAX[k] * sc for k, sc in [("Drive_mA",1.0),("I_A",1000.0)] if k in S_MAX]
        if right_lo and right_hi:
            lo_raw, hi_raw = min(right_lo), max(right_hi)
            # Sub-1-amp range: enforce ±0.1 A minimum span and 0.1 A tick steps
            if hi_raw < 1000 and lo_raw > -1000:
                lo_raw = min(lo_raw, -100)
                hi_raw = max(hi_raw,  100)
                ax_drive.yaxis.set_major_locator(mticker.MultipleLocator(100))
            else:
                ax_drive.yaxis.set_major_locator(mticker.AutoLocator())
            ax_drive.set_ylim(*padded_lim(lo_raw, hi_raw))

        # V_V comes from ax_power; P_W lives on ax_pw so add it explicitly.
        extra_h = ([hp] + right_h) if hp is not None else right_h
        extra_l = (["P_W"] + right_l) if hp is not None else right_l
        small_legend(ax_power, extra_h, extra_l)

        draw_stats_table(ax_pstats, POWER_TABLE_KEYS, POWER_TABLE_COLORS, win_rows,
                         "Power", f"Enable: {DEVICE_STATE['en']}")

        # ── Full session graph ─────────────────────────────────────────
        style_plot(ax_sess, "Full Session", "Value", "Elapsed (s)")
        all_t = [r["t"] for r in ROWS]
        pw    = [r.get("P_W") for r in ROWS]
        if any(v is not None for v in pw):
            ax_sess.plot(all_t, pw, color="#ffff00", label="P_W", linewidth=1)
        cold = [(r["t"], r["Tcold_C"]) for r in ROWS if r.get("Tcold_C") is not None]
        if cold:
            tx, vy = zip(*cold)
            ax_sess.plot(tx, vy, color="#00cfff", label="Tcold", linewidth=1)
        apply_ylim(ax_sess, ["P_W","Tcold_C"])
        small_legend(ax_sess)

# ── Window placement helpers ────────────────────────────────────────────────────
def _enum_monitors():
    """List of (left, top, right, bottom) rects for every monitor (Windows only)."""
    import ctypes
    from ctypes import wintypes
    rects = []
    MonEnum = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong,
                                 ctypes.POINTER(wintypes.RECT), ctypes.c_double)
    def _cb(hmon, hdc, lprc, data):
        r = lprc.contents
        rects.append((r.left, r.top, r.right, r.bottom))
        return 1
    ctypes.windll.user32.EnumDisplayMonitors(0, 0, MonEnum(_cb), 0)
    return rects

def _second_monitor_rect():
    """Rect of the secondary monitor (the one not anchored at 0,0). Falls back to
    the only monitor present if there's just one."""
    mons = _enum_monitors()
    if not mons: return None
    if len(mons) == 1: return mons[0]
    non_primary = [m for m in mons if not (m[0] == 0 and m[1] == 0)]
    return non_primary[0] if non_primary else mons[1]

def place_window():
    """Fill the second monitor (full height); the 22/78 grid splits the terminal
    pane from the graphs pane. Falls back to the only monitor present."""
    try:
        rect = _second_monitor_rect()
        if not rect: return
        left, top, right, bottom = rect
        root.geometry(f"{right - left}x{bottom - top}+{left}+{top}")
    except Exception:
        pass

# ── Embedded console plumbing ───────────────────────────────────────────────────
# stdout is tee'd into a queue; a periodic Tk callback drains it into term_out so
# device output appears in the left pane (Tk widgets must only be touched from the
# main thread, hence the queue).
CONSOLE_QUEUE = queue.Queue()

class _ConsoleTee:
    def __init__(self, real): self.real = real
    def write(self, s):
        if self.real:
            try: self.real.write(s)
            except Exception: pass
        if s: CONSOLE_QUEUE.put(s)
        return len(s) if s else 0
    def flush(self):
        if self.real:
            try: self.real.flush()
            except Exception: pass

def _drain_console():
    chunks = []
    try:
        while True: chunks.append(CONSOLE_QUEUE.get_nowait())
    except queue.Empty:
        pass
    if chunks:
        term_out.configure(state="normal")
        term_out.insert("end", "".join(chunks))
        # Keep the widget light — trim oldest lines once it grows large.
        if int(term_out.index("end-1c").split(".")[0]) > 4000:
            term_out.delete("1.0", "1000.end")
        term_out.see("end")
        term_out.configure(state="disabled")
    root.after(100, _drain_console)

def _submit_command(_evt=None):
    cmd = term_in.get(); term_in.delete(0, "end")
    cmd = cmd.rstrip()
    if not cmd: return
    if cmd.lower() == "x":
        print("[RECONNECT] Dropping serial port and reconnecting to whatever Nano is present …")
        RECONNECT.set()
        _safe_close()   # force-abort any readline() blocked on a vanished device
    elif cmd.lower().startswith("note "):
        with DATA_LOCK: PENDING_NOTE["text"] = cmd[5:].strip()
        print(f"[NOTE queued: {cmd[5:].strip()}]")
    else:
        print(f"> {cmd}")
        if ser and ser.is_open: ser.write((cmd + "\r\n").encode())

def _show_console(show):
    """Hide/show the spawned console window (Windows) for a clean single window."""
    try:
        import ctypes
        hwnd = ctypes.windll.kernel32.GetConsoleWindow()
        if hwnd:
            ctypes.windll.user32.ShowWindow(hwnd, 5 if show else 0)  # SW_SHOW / SW_HIDE
    except Exception:
        pass

def _on_close():
    _send_console_off()
    _show_console(True)
    root.destroy()

# ── Start ───────────────────────────────────────────────────────────────────────
sys.stdout = _ConsoleTee(sys.stdout)

threading.Thread(target=serial_reader, daemon=True).start()

ani = animation.FuncAnimation(fig1, animate, interval=500, cache_frame_data=False)

def _route_key(event):
    """Type anywhere in the window and it goes to the command entry — no need to
    click the box first. Keys already aimed at the entry, modifier shortcuts
    (e.g. Ctrl+C to copy from the log), and navigation keys are left alone."""
    if event.widget is term_in:
        return
    if event.state & 0x0004 or event.state & 0x20000:   # Control or Alt held
        return
    ks = event.keysym
    if ks == "Return":
        term_in.focus_set(); _submit_command(); return "break"
    if ks == "BackSpace":
        term_in.focus_set()
        s = term_in.get()
        if s: term_in.delete(len(s) - 1, "end")
        return "break"
    if event.char and event.char.isprintable():
        term_in.focus_set(); term_in.insert("end", event.char); return "break"
    return

term_in.bind("<Return>", _submit_command)
root.bind_all("<Key>", _route_key)
term_in.focus_set()
root.protocol("WM_DELETE_WINDOW", _on_close)

place_window()
_show_console(False)
root.after(100, _drain_console)
root.mainloop()

# Window closed — restore console + disable device console mode if active
_show_console(True)
_send_console_off()
