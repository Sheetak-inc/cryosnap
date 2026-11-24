"""
test_pid1.py  —  PID-1 Integrator Windup in MODE_COOL
Version 1.0

ISSUE (colleague firmware audit, PID-1, HIGH severity):
  task_100ms() calls pid_compute() and THEN applies the MODE_COOL/HEAT
  constraint (CryoSnap.ino lines 841-842):

      float out = pid_compute(g_setpoint, t_cold, dt, (float)g_imax_mA);
      if (g_mode == MODE_COOL && out > 0) out = 0;   // ← clamp AFTER integrate

  Inside pid_compute(), the integral accumulates unconditionally BEFORE the
  caller can clip it. In MODE_COOL, when Tcold < setpoint (error positive →
  controller wants to heat), the caller clamps the output to 0. But the
  integral has already grown by err*dt this tick and is NOT rolled back.

  When Tcold later rises above setpoint (cooling IS needed), the wound-up
  positive integral opposes the negative error. The output remains positive
  (heat direction → clamped to 0 again) until the integral unwinds completely.

HYPOTHESIS:
  After WINDUP_S seconds with Tcold < SP in MODE_COOL, changing the setpoint
  to just below Tcold (small negative error) causes the system to NOT cool for
  many tens of seconds while the integral unwinds. A fresh-integral baseline
  with the same small error cools within ~100ms.

QUANTITATIVE PREDICTION (imax=2000mA, Ki=5.50, Kp=200, error_cool=-3°C):
  Integral clamp:    i_lim = imax / Ki = 2000 / 5.50 = 363.6
  Cooling threshold: integral must reach i_thresh = |Kp × err| / Ki
                               = (200 × 3) / 5.50 = 109.1
  Unwind required:   363.6 - 109.1 = 254.5 units
  Each tick adds:    err × dt = -3 × 0.1 = -0.3
  Ticks to unwind:   254.5 / 0.3 ≈ 848 ticks = ~85 seconds

  Baseline (integral = 0):  output = 200 × (-3) = -600mA → cools on tick 1 (~100ms)

PROTOCOL:
  Phase 0  Setup: mode 0 (COOL), pid 1 (PID mode), imax 2000, deadband 0
           Verify mode via 'status' command
  Phase 1  Baseline: enable 0 → enable 1 (fresh integral), set SP 3°C below Tcold,
           measure time until Drive_mA < -100mA
  Phase 2  Wind-up: enable 0 → enable 1, set SP 12°C ABOVE Tcold, hold WINDUP_S,
           then set SP 3°C below current Tcold, measure same delay
  Report: Phase 1 delay, Phase 2 delay, ratio. If PID-1 present: ratio >> 1.
"""

# ── Parameters ─────────────────────────────────────────────────────────────────
IMAX          = 2000    # mA — controls out_max and integral clamp
KP            = 200.0   # must match firmware default (will set explicitly)
KI            = 5.50    # must match firmware default (will set explicitly)
WINDUP_S      = 65      # seconds to saturate the integrator
DELTA_HIGH    = 12.0    # °C ABOVE Tcold for wind-up phase
DELTA_COOL    = 3.0     # °C BELOW Tcold for cooling demand after wind-up
COOL_THRESH   = -100    # mA — threshold for "cooling has started"
RESPONSE_TIMEOUT = 150  # seconds max wait for cooling to start
VMAX          = 12000   # mV — keep moderate for safety


# ── Entry point ────────────────────────────────────────────────────────────────

def run_test(ctx):
    _hdr(ctx, "PID-1 INTEGRATOR WINDUP TEST  v1.0")

    # Setup
    ctx.log("Phase 0: Setup")
    _enable_console(ctx)

    _send_verified(ctx, "mode 0",         "Mode")       # COOL mode
    _send_verified(ctx, "pid 1",          "PID:1")
    _send_verified(ctx, f"imax {IMAX}",   f"Imax={IMAX}")
    _send_verified(ctx, f"vmax {VMAX}",   f"Vmax={VMAX}")
    _send_verified(ctx, f"kp {KP}",       f"Kp")
    _send_verified(ctx, f"ki {KI}",       f"Ki")
    _send_verified(ctx, "kd 0",           "Kd")
    _send_verified(ctx, "deadband 0",     "DB")
    _send_verified(ctx, "damping 100",    "Damp")       # large damping band → won't enter it
    ctx.wait(0.5)

    # Verify mode via status
    st = _get_mode(ctx)
    ctx.log(f"  Mode confirmed: {st}")
    if "Cool" not in st and "0" not in st:
        ctx.log("  WARNING: mode may not be COOL — check firmware 'mode' command format")

    init_sp = ctx.latest().get("Setpoint_C", 20.0)
    results = {}

    # ── Phase 1: Baseline (fresh integral) ────────────────────────────────────
    _hdr(ctx, "PHASE 1: Baseline — fresh integral, small cooling step")

    ctx.send("enable 0"); ctx.wait(0.5)
    ctx.send("enable 1"); ctx.wait(0.6)    # pid_reset() called on enable

    row     = ctx.latest()
    t_now   = row.get("Tcold_C", 20.0)
    sp_cool = round(t_now - DELTA_COOL, 1)
    ctx.log(f"  Tcold={t_now:.1f}°C  SP→{sp_cool}°C  (error={-DELTA_COOL:.1f}°C)")
    ctx.log(f"  Integral=0 (fresh).  Expected: cooling within ~100ms")

    _send_verified(ctx, f"set {sp_cool}", f"SP={sp_cool}")
    t_start = ctx.now()
    delay1  = _wait_for_cooling(ctx, COOL_THRESH, RESPONSE_TIMEOUT)

    if delay1 is None:
        ctx.log(f"  ERROR: cooling never started in baseline — check mode/hardware")
        results["p1"] = None
    else:
        ctx.log(f"  ✓ Cooling started in {delay1:.2f}s")
        results["p1"] = delay1

    if ctx.is_stopped(): return _finish(ctx, results, init_sp)
    ctx.wait(2)
    ctx.send("enable 0"); ctx.wait(0.5)

    # ── Phase 2: Wind-up then same small step ─────────────────────────────────
    _hdr(ctx, f"PHASE 2: {WINDUP_S}s wind-up in MODE_COOL, then same -3°C step")

    ctx.send("enable 0"); ctx.wait(0.3)
    row      = ctx.latest()
    t_base   = row.get("Tcold_C", 20.0)
    sp_high  = round(t_base + DELTA_HIGH, 1)
    i_lim    = IMAX / KI
    sat_s    = i_lim / (DELTA_HIGH * 0.1 * 10)  # ticks / 10 = seconds
    ctx.log(f"  Tcold={t_base:.1f}°C  SP→{sp_high}°C  (error=+{DELTA_HIGH:.0f}°C → heats)")
    ctx.log(f"  Integral will wind to +{i_lim:.1f} (saturates in ~{sat_s:.0f}s)")
    ctx.log(f"  TEC output: 0 (clamped by MODE_COOL).  TEC stays OFF.")

    _send_verified(ctx, f"set {sp_high}", f"SP={sp_high}")
    ctx.send("enable 1"); ctx.wait(0.6)   # fresh integral, then wind it up

    for tick in range(WINDUP_S * 2):
        if ctx.is_stopped(): return _finish(ctx, results, init_sp)
        ctx.wait(0.5)
        if tick % 20 == 0:
            row = ctx.latest()
            ctx.log(f"  t={row.get('t',0):.0f}s  Tcold={row.get('Tcold_C','?'):.1f}°C  "
                    f"Drive={row.get('Drive_mA',0):.0f}mA  "
                    f"[integral winding — Drive should be 0]")

    # Switch SP to same small negative error as Phase 1
    row       = ctx.latest()
    t_after   = row.get("Tcold_C", t_base)
    sp_cool2  = round(t_after - DELTA_COOL, 1)
    error_cool = sp_cool2 - t_after

    # Theoretical unwind delay
    i_thresh = abs(KP * DELTA_COOL) / KI
    unwind   = (i_lim - i_thresh) / (DELTA_COOL * 0.1 * 10)
    ctx.log(f"")
    ctx.log(f"  Windup complete.  Tcold={t_after:.1f}°C  SP→{sp_cool2}°C  (error={error_cool:.1f}°C)")
    ctx.log(f"  Integral ≈ +{i_lim:.1f} (saturated).  Ki×integral ≈ +{IMAX}mA")
    ctx.log(f"  Predicted output: Kp×({error_cool:.1f}) + {IMAX} = {KP*error_cool + IMAX:.0f}mA → clamped 0")
    ctx.log(f"  Predicted unwind delay: ~{unwind:.0f}s  (vs baseline {results.get('p1', '?')}s)")

    _send_verified(ctx, f"set {sp_cool2}", f"SP={sp_cool2}")
    delay2 = _wait_for_cooling(ctx, COOL_THRESH, RESPONSE_TIMEOUT)

    if delay2 is None:
        ctx.log(f"  Cooling did not start within {RESPONSE_TIMEOUT}s  — PID-1 confirmed (severe)")
        results["p2"] = RESPONSE_TIMEOUT
    else:
        ctx.log(f"  Cooling started in {delay2:.2f}s")
        results["p2"] = delay2

    _finish(ctx, results, init_sp)


# ── Helpers ────────────────────────────────────────────────────────────────────

def _wait_for_cooling(ctx, threshold_mA, timeout_s):
    """Poll every 0.3s until Drive_mA < threshold. Returns elapsed seconds or None."""
    t0    = ctx.now()
    steps = int(timeout_s / 0.3) + 1
    for i in range(steps):
        if ctx.is_stopped(): return None
        ctx.wait(0.3)
        drive = ctx.latest().get("Drive_mA", 0) or 0.0
        if drive < threshold_mA:
            return ctx.now() - t0
        if i % 30 == 0 and i > 0:
            ctx.log(f"  +{ctx.now()-t0:.0f}s  Drive={drive:.0f}mA  (integral unwinding …)")
    return None


def _get_mode(ctx):
    """Send 'status' and extract the Mode field."""
    idx = ctx.log_index()
    ctx.send("status")
    ctx.wait(3.0)
    for _, msg in ctx.events_after(idx):
        if "Mode:" in msg:
            import re
            m = re.search(r"Mode:(\w+)", msg)
            if m: return m.group(1)
    return "?"


def _enable_console(ctx):
    idx = ctx.log_index()
    ctx.send("console")
    last = _wait_for_echo(ctx, idx, "Console:", startswith=True)
    if last == "Console:0":
        idx2 = ctx.log_index()
        ctx.send("console")
        _wait_for_echo(ctx, idx2, "Console:", startswith=True)


def _send_verified(ctx, cmd, expected_echo, max_wait=3.0):
    idx = ctx.log_index()
    ctx.send(cmd)
    found = _wait_for_echo(ctx, idx, expected_echo)
    if not found:
        ctx.log(f"  WARNING: '{cmd}' — echo '{expected_echo}' not confirmed")
    return bool(found)


def _wait_for_echo(ctx, start_idx, needle, startswith=False, max_wait=3.0):
    steps = int(max_wait / 0.25) + 1
    for _ in range(steps):
        if ctx.is_stopped(): return ""
        ctx.wait(0.25)
        for _, msg in ctx.events_after(start_idx):
            if startswith and msg.startswith(needle): return msg
            if not startswith and needle in msg:      return msg
    return ""


def _hdr(ctx, title):
    ctx.log(""); ctx.log("─" * 58)
    ctx.log(f"  {title}"); ctx.log("─" * 58)


def _finish(ctx, results, original_sp):
    ctx.log("")
    ctx.log("Restoring …")
    ctx.send("enable 0");         ctx.wait(0.3)
    ctx.send("mode 2");           ctx.wait(0.3)   # AUTO
    ctx.send("pid 1");            ctx.wait(0.3)
    ctx.send(f"imax 2000");       ctx.wait(0.3)
    ctx.send(f"vmax 12000");      ctx.wait(0.3)
    ctx.send("deadband 0.2");     ctx.wait(0.3)
    ctx.send("damping 3.0");      ctx.wait(0.3)   # restore default 3°C
    ctx.send(f"set {original_sp:.1f}"); ctx.wait(0.3)
    ctx.send("enable 1");         ctx.wait(0.5)

    ctx.log("")
    ctx.log("=" * 58)
    ctx.log("  RESULTS — PID-1 Integrator Windup")
    ctx.log("=" * 58)

    d1 = results.get("p1")
    d2 = results.get("p2")

    ctx.log(f"Phase 1 (baseline, fresh integral):    {f'{d1:.2f}s' if d1 is not None else 'FAILED'}")
    ctx.log(f"Phase 2 (after {WINDUP_S}s wind-up): {f'{d2:.2f}s' if d2 is not None else 'timed out'}")

    if d1 is not None and d2 is not None:
        ratio = d2 / max(d1, 0.05)
        ctx.log(f"Ratio (windup / baseline):             {ratio:.1f}×")
        ctx.log("")

        theory_s = (IMAX/KI - abs(KP*DELTA_COOL)/KI) / (DELTA_COOL * 0.1 * 10)

        if d2 > 5.0 and d1 < 2.0:
            ctx.log("PID-1 CONFIRMED.")
            ctx.log(f"  Theoretical delay: ~{theory_s:.0f}s   Measured: {d2:.1f}s")
            ctx.log("")
            ctx.log("Impact: In COOL mode, after any cold-side undershoot")
            ctx.log("  (cold start, overshoot, or ambient dip below setpoint),")
            ctx.log("  the controller may not cool for 30-150s when cooling is")
            ctx.log("  needed — proportional to wind-up depth and integral gain.")
            ctx.log("")
            ctx.log("Fix (from audit recommendation):")
            ctx.log("  Option A — Conditional integration: freeze integral when")
            ctx.log("    the unconstrained output is in the forbidden direction.")
            ctx.log("  Option B — Pass mode into pid_compute() and skip integral")
            ctx.log("    accumulation on ticks where output would be clamped.")
            ctx.log("  Option C — Back-calculation anti-windup: subtract a term")
            ctx.log("    proportional to (out_pre_clamp − out_post_clamp) each tick.")
        elif d2 <= 2.0:
            ctx.log("PID-1 NOT reproduced — bug may have been fixed.")
        else:
            ctx.log(f"Ambiguous result — d2={d2:.1f}s. Rerun or check mode setting.")

    ctx.log("=" * 58)
