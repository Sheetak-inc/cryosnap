#ifndef VERSION_H
#define VERSION_H

// Firmware version — update on each meaningful change.
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  7
#define FW_VERSION_PATCH  12
#define FW_VERSION_STR    "0.7.12"

/*
  Changelog (newest first):

  0.7.12  Seebeck LOW-V powered flip (Rev B) — replaces the OE-off wait

    Branched off v0.7.11. On the Rev B board the v0.7.11 OE-off
    measured-wait recipe is fatal: dropping OE to reverse lets the TPS
    output cap decay into the Seebeck band and then puts the REVERSED
    EMF across a de-energised rail, dragging it below ground until the
    TPS wedges off the I2C bus (bench: 54/54 kills, recovery only via a
    full chip re-init). The cap/bridge topology that tolerated the
    dwell on the Rev A rig does not on Rev B.

    Fix: keep the converter POWERED through the entire reversal. New
    compile-time recipe SEEBECK_POWERED_FLIP (Pins.h: default 1 on
    Rev B, 0 on Rev A = the legacy OE-off path, untouched). The Rev B
    LOW-V flip sequence (SB_LOWV_SETTLE -> SB_LOWV_DWELL):
      (1) on a polarity change, KEEP the old direction but drop V_limit
          to SEEBECK_LOWV_FLOOR_MV (800 mV) and I_limit to
          SEEBECK_LOWV_I_MA (1500 mA), OE on. The rail falls to the
          I-limited floor; the converter never de-energises.
      (2) after SEEBECK_LOWV_SETTLE_MS (400 ms), flip the H-bridge LIVE
          (hb_setDirection — OE stays on, no off/on blink). At the low
          I-limited rail the reversal transient is bounded and there is
          no converter-restart inrush (the per-flip SCP source).
      (3) hold at the floor for SEEBECK_LOWV_DWELL_MS (2000 ms) so the
          reversal settles, then return to SB_IDLE; actuate restores
          Vmax and soft-starts the current to full in the new direction.

    Bench validation (Rev B, 2026-06-15 two-flow comparison, ~70 W
    gradient): LOW-V reverse 3/3 survived, 5.99 A back, no SCP, chip
    on-bus (CDC A0) the whole time; the OE-off reverse killed the chip
    2/2 on the same rig (STATUS+CDC NACK, recovery via Enable:0 + chip
    reconfigure). Max-power end-to-end (powered flip, both directions)
    was 4/4 survived, 0 kills, 0 SCP, CDC A0 every poll.

    Two pre-ship fixes carried for the powered-flip path:
      M1  CDC-drift recovery in Diagnostics.h (task_diag, 1 Hz) is now
          an OE-SAFE targeted CDC register write, not _tps_only_recover/
          tps_init — the old path dropped OE, which during the dwell
          (reversed EMF on the load) would itself cause the dead-rail
          kill. Actuate re-writes V/I every tick, so CDC is the only
          register a silent reset leaves stale. This is intentional for
          BOTH targets (a 1 Hz CDC drift no longer silently disarms a
          latched PD clamp); a real chip loss still gets the full
          recovery via the task_100ms TPS-presence path. _tps_only_recover
          now also resets _seebeck_state to SB_IDLE so a chip-loss
          recovery mid-flip restarts cleanly.
      M2  the powered-flip SM is skipped under a PD-budget clamp so the
          clamp's reduced V/I limits are not stranded by the floor
          writes; the flip then falls through to actuate, which on Rev B
          does a LIVE hb_setDirection (see "Post-flip dwell hardening").

    Post-flip dwell hardening (from a multi-agent adversarial review;
    these guard concurrent events the 3/3 bench run did not exercise):
      - On Rev B, actuate NEVER uses the OE-off hb_safeDirectionChange:
        any direction change that bypasses the LOW-V SM (PD clamp, or a
        mid-dwell intent reversal that lands at dwell-exit) does a LIVE
        hb_setDirection instead. A bare live flip survives (may trip a
        recoverable SCP) where the OE-off path would wedge the chip.
      - A fault/disable latching DURING the dwell (OVERTEMP being the
        likely one, since the fan is on VOUT and dips during the hold)
        used to drop OE while the bridge was reversed vs the gradient =
        the kill. Now the disable path re-aligns the bridge to the
        pre-flip direction (live, OE still on) BEFORE OE drops, so the
        OE-off floats the rail to +Vs. Relatedly, on Rev B the disable
        else-branch no longer forces the bridge to HB_COOL after OE-off
        (which could reverse it vs the gradient).

    Knobs (Config.h): SEEBECK_LOWV_FLOOR_MV / _I_MA / _SETTLE_MS /
    _DWELL_MS. Cost: a ~2.4 s low-power window per large-ΔT reversal
    (settle + dwell; fixed, deterministic). Fan is on VOUT so airflow
    dips during the hold (OVERTEMP backstops; Rev C: fan on input rail).

    Still open before a FIELD default: Rev A vs Rev B topology contrast;
    fresh-board confirmation (all data is the one Rev B unit); scope of
    the below-ground excursion; the SAFETY_FAULTS=1 fault-latch chain
    not yet cleanly bench-tested on Rev B (D5 fan-tach false-fault until
    a tach pull-up is added); a slow deadband-crossing reversal vs the
    large setpoint swings tested.

  0.7.11  Seebeck-measure-and-adjust, non-blocking state machine

    Observed symptom: under sustained high-current cooling followed
    by a controller-commanded polarity flip at large ΔT, the TPS55288
    intermittently locked up — registers reset to defaults, chip
    silent on I2C, recovery only via power cycle. Bench reproduction
    (test_nopsu.py Phase 5, v0.7.10 baseline) showed 3 lock-up
    events per run, 5/5 runs.

    Mechanism: at the moment of polarity reversal, the TEC's
    accumulated Seebeck EMF (α·ΔT, observed ~2 V at Phase-5-class
    ΔT≈40 K) combines with H-bridge body-diode kickback during the
    few-microsecond PIN_DIR transition. The combined transient
    couples into the TPS55288's EN/VCC pin and triggers its
    EN-shutdown protection. The v0.7.10 soft-start ramp limits
    steady-state CURRENT but does not address the TRANSIENT.

    Approach (rejected variants — preserved here so future work
    does not re-tread the same paths):

      No mitigation (v0.7.10 baseline):  5/5 runs, 3 lock-ups per run
      Active V_limit discharge to 1V:    0 lock-ups but 3 FAULT[SCP]
                                          in Phase 3 from chip-side
                                          OVP-discharge inrush
      Active V_limit discharge to 5V:    gentler but still 2 SCP /
                                          1 lock-up
      Passive OE-off 150/500 ms:         no SCP, no Phase-5 fix
      Passive OE-off 1000 ms:            partial (2 runs: 0 then 2
                                          lock-ups)
      Passive OE-off 2000 ms (fixed):    3/3 runs, 0 lock-ups, 0 SCP
      INA-polled V<2500 mV early-exit:   0/3 (fails: bus settles in
                                          <500 ms but the chip needs
                                          longer internal recovery)
      Setpoint-magnitude scaled wait:    3/3 runs, 0 lock-ups
      Seebeck-MEASURED wait (shipping):  3/3 runs, 0 lock-ups
                                          (CDC=0xE0 resets: 1/0/2,
                                          all self-recovered within
                                          the same Phase-4 toggle
                                          window)

    Setpoint magnitude was rejected as a proxy because heat-removal
    quality varies across deployments. A TEC on a chilled-water loop
    keeps ΔT (and therefore Seebeck) small even on a large setpoint
    swing and should not be made to wait; a poorly-heatsunk one
    accumulates large ΔT even on a small swing and needs the full
    mitigation. Measuring Seebeck directly tracks the actual
    physical condition.

    Design: a non-blocking state machine living across task_100ms
    ticks (SB_IDLE → SB_OE_OFF_SETTLE → SB_OE_OFF_WAIT → SB_IDLE).
    On a detected drive_dir change while tec_on, after FAULT_GRACE_MS
    has elapsed since enable:

      (1) tps_setOutput(false), record SETTLE deadline (now+20 ms),
          enter SB_OE_OFF_SETTLE. If the OE-off write NACKs the
          chip is still driving in the old direction; skip the state
          machine entirely and let normal actuate retry (its NACKs
          accumulate toward FAULT[NOPSU] through the existing path).

      (2) SETTLE deadline elapses → read INA bus voltage. The
          DRV8701E H-bridge has EN tied high (always enabled), so
          the commanded MOSFETs stay in low-Rds linear region and
          provide a near-zero-resistance bidirectional path between
          the INA-side rail and the TEC terminals. With OE off, the
          TPS output cap equilibrates to the Seebeck EMF magnitude
          across that path (only a few mV of MOSFET drop), so V_meas
          is a direct proxy for |Seebeck EMF| down to the INA226
          1.25 mV LSB. Compute
          wait_ms = SEEBECK_HB_OFF_BASE_MS +
                    V_meas * SEEBECK_HB_OFF_PER_VOLT_MS
          clamped to SEEBECK_HB_OFF_MAX_MS. Push the deadline
          forward, enter SB_OE_OFF_WAIT.

      (3) WAIT deadline elapses → return to SB_IDLE; actuate runs
          this tick in the new direction. Reset _pid_last_ms = 0 so
          the next pid_compute treats the wait as one nominal tick
          of dt (the existing first-call fallback) instead of the
          wait's full duration clamped at 0.5 s — without this the
          integrator would take a 5x-oversized step on the post-flip
          tick (BUG-008 regression protection). Update
          _last_actuate_ms so the soft-start gap-based re-arm does
          not fire on the post-WAIT tick — a polarity flip on the
          same TEC is not an inrush event (same load, opposite
          polarity), so the ramp deliberately is NOT applied here.
          Record the new direction so this tick's drive does not
          re-trigger the state machine via the immediate fall-
          through into actuate.

    The gate `if (_seebeck_state == SB_IDLE)` wraps the entire
    actuate write block, so during SETTLE and WAIT no V/I/dir/OE
    writes happen. The rest of task_100ms — fault polls, fan PWM,
    LEDs, operator command pump, INA / HUSB monitors — continues
    running every tick. Operator emergency-off (button or serial)
    is honoured within one scheduler tick (~100 ms).

    History gating: _last_drive_dir_exp only updates on ticks where
    tec_on is true. Deadband ticks leave drive_dir at its default
    HB_COOL, so without the gate a HEAT-driving tick followed by a
    deadband tick would stamp history as COOL and trigger a spurious
    100–3000 ms wait on the next active drive. _have_last_dir
    (explicit bool, not a sentinel inside the direction value) is
    cleared on the operator-enable rising edge (both the button
    handler and the serial enable command) so first-actuate after
    fault recovery does not compare against pre-fault history.

    Grace-window gate: the state machine only fires after
    FAULT_GRACE_MS has elapsed since enable. Without this gate a
    first-tick flip could consume the entire 3 s grace window
    inside the wait, preventing task_slow's first fan-tach poll
    from running and producing a spurious FAULT[FAN] on the
    following tick.

    Tuning: SEEBECK_HB_OFF_BASE_MS (100 ms), SEEBECK_HB_OFF_SETTLE_MS
    (20 ms), SEEBECK_HB_OFF_PER_VOLT_MS (1000 ms / V),
    SEEBECK_HB_OFF_MAX_MS (3000 ms). Defaults map a measured 2 V
    Seebeck to a 2100 ms wait and a 0.1 V Seebeck to a 200 ms blip.
    Setting SEEBECK_HB_OFF_MAX_MS = 0 compiles the state machine
    out entirely (polarity flips proceed as in v0.7.10).

    Hysteresis floor: SEEBECK_MIN_SAME_DIR_MS (default 1000 ms). After
    a wait completes the SM refuses to re-arm for this many ms. Aimed
    at limit-cycle oscillation in bang-bang + Mode Auto: a controller
    that flips direction every few seconds around setpoint would
    otherwise force a fresh wait on every crossing. The prior wait has
    already protected the chip from polarity-flip stress, so a flip
    within the floor proceeds without a fresh wait. Set to 0 to
    disable. All five knobs live in Config.h alongside SOFT_START_*.

    Build cost: ~290 B flash and 7 B RAM for the non-blocking SM;
    SF#3 hysteresis floor adds ~60 B flash and 4 B RAM; trace lines
    (ENABLE_SEEBECK_TRACE, on by default when ENABLE_DIAGNOSTICS=1)
    add another ~150 B flash for the operator-facing `DIAG: Seebeck
    V=...` decay log; symmetric history-and-floor clears on every
    state-loss event (operator enable, button enable, chip recovery)
    add another ~50 B for the invariant "one clear per state-loss
    event". ENABLE_I2C_BOOT_SCAN and ENABLE_VERBOSE_BOOT default
    OFF in this revision so the build clears the Nano's 30720 B
    ceiling. Driver-specific WARN messages at boot (WARN TPS / INA /
    HUSB / OLED) report missing chips by name, so the full bus walk
    is a bring-up convenience rather than a production requirement.
    The version banner is emitted regardless of ENABLE_VERBOSE_BOOT
    so operator and log-parsing tools can still identify the running
    build. Final size: 30696 / 30720 B (99.9 %, 24 B free). The
    next feature on this branch must reclaim flash before adding
    code — see Config.h ENABLE_* flags or set ENABLE_SEEBECK_TRACE=0
    (~150 B), ENABLE_DIAGNOSTICS=0 (~500 B), or
    SEEBECK_MIN_SAME_DIR_MS=0 (~60 B) to recover headroom.

    Caveat: this is a firmware-side mitigation. The underlying
    hardware coupling (Seebeck EMF + H-bridge inductive kickback →
    EN/VCC transient) remains. A future hardware revision should
    add a TVS clamp on the TPS output or move to a driver with
    less-sensitive SCP / EN protection.

    Known untested paths in this revision (no Phase-5-class regression
    expected, but worth a bench check on the next opportunity):
    re-enable during an active WAIT, fault-latch during WAIT,
    HAS_INA226 = 0 build, and MINIMAL_BUILD path.

    Rev B bring-up (2026-06-10): boots clean on the first assembled
    Rev B board. All four I2C peripherals enumerate at the expected
    Rev B addresses (TPS55288 0x74, OLED 0x3C, INA226 0x40,
    HUSB238 0x08). PD negotiates 20 V / 3250 mA. NTC1 reads
    sensible room temperature (23.3 C). H-bridge polarity inherited
    from Rev A is bench-confirmed correct: 200 mA Cool drive
    produced a monotonic 1.00 C drop in Tcold over 47 s. Fan tach
    on D5 reports 0 RPM (operator-confirmed fan is connected; the
    new D5 net likely needs an external 10K pull-up to 5 V that the
    Rev A board had on D2). All TEC tests in this session used
    fan = 0 to suppress the FAULT[FAN] check. Not yet exercised on
    Rev B: mid-current envelope (500 mA / 1 A), Seebeck SM polarity-
    flip regression at sustained drive, HEAT-mode inverse polarity,
    OLED render, button D7, mode pot, LED chain D4.

  0.7.10  2026-06-09  Soft-start I_limit ramp at the actuate layer

    Motivation: BUG-003 addendum bench evidence (logs/23.txt) showed
    enable-edge inrush as a recurring trigger for FAULT[PD] events.
    Multiple Phase-4 toggles knocked the upstream USB-PD source off
    for 500 ms each, and the eventual Phase-5 chip-lockup was
    preceded by repeated brown-outs the PD source could not sustain.

    Mechanism: I_limit linear ramp from SOFT_START_I_MA up to
    g_imax_mA over SOFT_START_MS at the actuate layer, above PID
    and bang-bang so both controllers benefit. Three re-arm
    triggers — operator enable rising edge, _tps_only_recover
    success leg, and any actuate gap > SOFT_START_RESET_GAP_MS
    (deadband-exit protection). Configurable via Config.h
    (ENABLE_SOFT_START, SOFT_START_MS, SOFT_START_I_MA,
    SOFT_START_RESET_GAP_MS). Disabling (ENABLE_SOFT_START=0)
    collapses to today's instant-jump writes byte-for-byte.

    Implementation notes:
    - I_limit only. For a TEC (low-impedance + Seebeck EMF) the
      V_limit is just a ceiling that never engages during drive;
      V_limit is written through unmodified.
    - Monotonic ceiling formula (ramp_cap_mA = SOFT_START_I_MA +
      proportional climb to g_imax_mA, then `ramp_mA = min(drive_mA,
      ramp_cap_mA)`). Avoids the dithering hazard where a controller
      oscillating across SOFT_START_I_MA would produce non-monotonic
      IOUT_LIMIT register writes — the chip's I_limit must climb
      monotonically during the ramp regardless of controller activity.
    - Ramp window strictly < FAULT_GRACE_MS so the supply-Vlim and
      fan-tach grace checks fully cover the ramp. Compile-time
      #error guards in CryoSnap.ino enforce this invariant.
    - One-shot `_ramp_armed` flag (NOT an `_enable_time` inequality)
      to avoid millis()-rollover, boot-zero, and silent-rename
      failure modes.
    - Stream telemetry mutates drive_mA in place after the ramp so
      the operator-visible plotter shows the value actually written
      (not the controller's commanded value). Without this the
      plotter would read full-drive while INA showed ramped current,
      causing diagnostic confusion.

    Pre-push review (workflow wf_e9e5d234-632, 2026-06-09): six
    parallel adversarial lenses found four majors and seven minors;
    four were applied in this commit, three were deferred (see risk
    register below). Cross-lens hits — _tps_only_recover not
    re-arming the ramp, deadband-exit slamming because the comment
    claimed it was harmless without evidence, PID dithering producing
    non-monotonic writes — all closed by the monotonic-ceiling +
    _ramp_armed redesign above.

    Bench validation: MECHANISM CONFIRMED, EFFICACY INCONCLUSIVE.
    The m4 drive_mA-mutate-in-place change makes the ramp visible
    in stream telemetry. Run
    ~/testing/logs/nopsu_v710_run1_20260609_023947.log (with the
    v0.7.10 banner) shows the first INA stream sample after every
    Enable:1 catching the ramp at varying points:
      Enable A first sample:  Drive=-200 mA   INA=0.001 A  (t≈0)
      Enable B first sample:  Drive=-1186 mA  INA=0.159 A  (t≈100 ms)
      Enable C first sample:  Drive=-3129 mA  INA=2.086 A  (t≈300 ms)
      Enable D first sample:  Drive=-5043 mA  INA=2.086 A  (t≈500 ms)
      Enable * second sample: Drive=-6000 mA  INA≈5.6 A    (post-ramp)
    The monotonic Drive value across each sample corresponds to
    `ramp_cap_mA` advancing through the SOFT_START_MS window. The
    ramp is mechanically working as designed.

    Efficacy is INCONCLUSIVE at this N. Across 3 paired Phase-4
    sessions with PD attached:
      v0.7.9 Run B (no ramp):                2 CDC=0xE0 events
      v0.7.10 pre-banner-fix (ramp in code): 1 CDC=0xE0 event
      v0.7.10 banner-fixed (this commit):    2 CDC=0xE0 events
    Range 1–2 with no clear directionality. The pre-push adversarial
    review (workflow wf_e9e5d234-632) is right that N≥5 paired runs
    with controlled ambient + uptime are needed before any "X%
    reduction" claim. Phase-5 cumulative chip-lockup still fires in
    all v0.7.10 runs — that lockup is the BUG-003-addendum hardware
    ceiling (no I2C soft-reset, EN tied to AREF on Rev A; firmware
    has no PVIN-cycle path).

    Pre-push review (workflow wf_e9e5d234-632): six parallel
    adversarial lenses found 4 majors and 7 minors. Applied here:
    M1 (_tps_only_recover now re-arms the ramp via _ramp_armed
    flag), M2 (deadband-exit re-arms via SOFT_START_RESET_GAP_MS
    gap), M3 (monotonic ceiling formula replaces the dithering-
    prone conditional scale), m1 (Config.h doc no longer mentions
    the dropped V_limit ramp), m2 (_ramp_armed flag replaces the
    _enable_time inequality, closing 3 minor failure modes), m4
    (drive_mA mutated in place so plotter shows ramp shape).
    Deferred: M4 (runtime imax/vmax/load/defaults race against the
    ramp window — narrow, only triggers if operator retunes within
    the first 600 ms after enable), m3 (MINIMAL_BUILD soft-start
    enable — debatable), m5 (runtime `softstart 0|1` toggle —
    convenience), m6 (SOFT_START_MS=600 provenance — needs a scope
    measurement on the rig that we don't yet have).

    Flash: +200 B (30506 -> 30706; 99.95% / 14 B free). RAM: +9 B
    (1 B bool _ramp_armed + 4 B _ramp_start_ms + 4 B _last_actuate_ms).
    The 14 B free is a hard ceiling — any further addition to this
    branch needs to either fit in 14 B or earn its way in by
    trimming something else.

  0.7.9  2026-06-05  BUG-003 addendum — NACK-aware writes/reads, CDC drift detect

    Bench validation (rig: Arduino Nano + TPS55288 EVM,
    2026-06-08, full test_nopsu.py v1.1 protocol end-to-end).
    Two runs:

      Run A — no Vin on screw terminal, USB power only
      (FAULT[NOPSU] expected, since the chip cannot sustain
      V_limit > 5 V without Vin). Phase 5 (field-trigger
      replication) latched FAULT[NOPSU] at +3.41 s
      post-enable, trigger `trg V=5001` — the legitimate
      V_limit-floor trigger (5001 mV < SUPPLY_VLIM_FLOOR of
      5500 mV; the TPS auto-resets to its 5 V safety default
      under USB-only power). Phase 4 toggles 1-7 read
      CDC=0xE0 (chip reset by transients), toggles 8-11
      recovered to CDC=0xA0 via task_diag's drift check
      calling _tps_only_recover between toggles, toggles
      12-15 back to 0xE0 (chip resetting faster than the
      1 Hz drift check could catch — expected hardware-limit
      symptom). 4 NOPSU latches across the test (~16 actual
      triggers, all V_limit-floor), 11 CDC drifts. Log:
      ~/testing/logs/nopsu_v0.7.9_20260608_041928.log.

      Run B — USB-PD source attached (20 V / 3250 mA;
      negotiated budget=61750 mW). NOPSU faults: 0. The
      original BUG-003 was firmware mis-tripping NOPSU even
      when the chip was healthy; with adequate supply the
      fault no longer fires spuriously. Phase 5 (field-
      trigger replication) drove a clean 2000 mA for the
      full 4 s grace window with no fault — INA at trip-
      window end: I=1.934 A, V=2.426 V, P=4.688 W. Phase 3
      sustained 6000 mA / ~60 W for 90 s with no faults and
      CDC stayed 0xA0 throughout. Phase 4 saw 2 CDC=0xE0
      drifts (toggles 7 and 15), both correlated with
      legitimate FAULT[PD]: lost 20V for 500 ms events from
      the upstream source briefly dropping under rapid load
      swings — the fault chain correctly latched
      FAULT_HUSB_20V for those (not NOPSU), and CDC
      recovered to 0xA0 by the next toggle. Total: 0 NOPSU,
      2 FAULT[PD], 2 CDC drifts (both recovered). Log:
      ~/testing/logs/nopsu_v0.7.9_PD_20260608_083017.log.

    Caveat: the new Fix-1 NACK-counter trigger was NOT
    exercised by either run (chip ACKed every write — the
    failure mode in Run A was register state, not bus
    state); it remains a theoretical fix verified by code
    review only. The PD-clamp NACK-aware behavior was also
    not exercised (PD budget never reached the clamp
    threshold in Run B's drive profile).


    The 0.7.8 fix for BUG-003 (tps_isPresent ACK probe before the
    V_limit read) closed the "chip fell completely off the bus"
    case but bench evidence in the BUG-003 addendum (logs/Bug-003
    Addendum/BUG-003_addendum_v0.7.8.md, 2026-06-05) showed a
    second failure mode after a v1.1 NOPSU test ended cleanly:
    FAULT[NOPSU] tripping later, with the chip register dump
    showing stale Phase-4 PD-clamp residuals (CDC=0xE0,
    Vlim=15.25V from clamp_mV write, Ilim=5.00A from clamp_mA)
    that should have been wiped by Phase-5's enable -> _pd_reinit
    -> tps_init sequence.

    Root cause: two independent gaps combined.

      Gap A (writes silently NACK): _tps_write discarded
      Wire.endTransmission's return value. During Phase 4's
      high-current toggles the TPS Vin was dipping enough to
      NACK individual write transactions even though the
      address probe still ACK'd. _pd_reinit -> tps_init
      "succeeded" by return value while its writes never
      landed; firmware then cleared g_pd_clamped and
      _supply_fault_count, thinking the chip was at safe
      defaults while it was actually still at the Phase-4 clamp
      residuals.

      Gap B (probe-then-read race): tps_isPresent is one
      transaction; tps_getVoltageLimitMV does three more. If
      the chip NACKs one of the three reads, Wire.read() returns
      0, the decoder produces ~200 mV, and _supply_fault_count
      increments for a non-event. Combined with 0.7.5's sticky
      counter, two glitches anywhere in a session = NOPSU latch.

    Fixes (all four addendum recommendations):

    Fix 1 (Gap A): _tps_write now returns Wire.endTransmission's
    status. _tps_read tracks a sticky `_tps_last_read_ok` flag,
    exposed via the new tps_lastReadOk() helper. tps_init,
    tps_setVoltageLimit, tps_setCurrentLimit, tps_setOutput all
    return bool — true only when ALL their writes ACK'd and the
    read-modify reads succeeded. _tps_only_recover retries once
    after a 20 ms settle (closer to TPS UVLO de-glitch / soft-
    start than the original 2 ms first attempt); if it still
    fails, it logs `DIAG: tps_init writes NACKed -- chip state
    uncertain, retry deferred`, increments the new
    _tps_write_nack_count, and clears g_pd_clamped so actuate
    can take over (otherwise the clamp gate starves the NACK
    fault path of failed writes). _supply_fault_count,
    _supply_fault_last_bad_mv, and _tried_drive_last_tick are
    preserved on failure so a persistent supply problem
    correctly re-latches NOPSU after FAULT_GRACE_MS.

    The task_100ms actuate stage now tracks whether its V/I
    writes succeeded and only arms _tried_drive_last_tick when
    they did. Without this gate, a NACK'd actuate write would
    let the next supply check mis-interpret the chip's
    previous (unchanged) V_limit as our deliberate command.

    Fix 2 (CDC drift detect): task_diag (1 s cadence) now reads
    the CDC register via the new tps_getCDC() helper and compares
    to TPS_CDC_OPMODE. A mismatch means the chip silently reset
    its registers — recover via _tps_only_recover(). This
    catches the exact addendum dump state (chip alive on I2C,
    CDC reverted to 0xE0).

    Fix 3 (NACK-aware V_limit read): tps_getVoltageLimitMV
    returns the new TPS_VLIM_READ_FAIL sentinel (0xFFFF) if any
    of its three register reads NACK'd. The supply check
    interprets the sentinel as "this tick is unreliable, skip"
    rather than as a 65535 mV reading or as the decoded ~200 mV
    that the old code would have produced.

    Fix 4 (trip telemetry): every increment of
    _supply_fault_count now logs `DIAG: NoPSU counter=N/M
    Vlim=X mV` so the trigger trajectory is visible in serial
    logs (max 2 lines per session at debounce=2). The
    FAULT[NOPSU] trip print also includes the last bad V_limit
    value (or "NACK during read burst" if the sentinel fired),
    so post-trip debugging doesn't have to guess what drove the
    counter when the post-trip register dump looks healthy.

    Validation per the addendum: re-run test_nopsu.py v1.1 in
    logs/Bug-003 Addendum/ (engagement-internal test tooling,
    not shipped with the firmware) unchanged; expect no NOPSU
    during the test (already passes in 0.7.8) AND no NOPSU
    after the test ends and the monitor disables console mode.
    The addendum also recommends a Phase 6 extension that holds
    enable for another 10-20 s past Phase 5 to catch future
    regressions of this exact pattern without depending on
    monitor housekeeping timing.

    Adversarial-review follow-ups (also in this commit)
    ---------------------------------------------------
    A 5-lens / 8-subagent workflow caught three real majors plus
    several minors, all fixed before commit.

    - Forward declaration: _tps_only_recover is now called from
      Diagnostics.h (task_diag CDC branch) but was defined AFTER
      that #include. The build worked only via Arduino IDE's
      auto-prototyping for static .ino functions — fragile and
      inconsistent with the explicit forward-decl pattern used
      for _pd_reinit. Added the matching forward declaration.

    - Alive-but-NACKing chip silently never triggered NOPSU.
      The Fix-1 gating set _tried_drive_last_tick = writes_ok;
      if writes kept NACKing, _tried_drive stayed false forever,
      the supply check fell into its reset branch every tick,
      and the counter never accumulated. New
      _tps_write_nack_count tracks consecutive ticks where the
      actuate stage tried to drive but its writes NACK'd;
      TPS_NACK_FAULT_DEBOUNCE (5) consecutive failures latch
      FAULT_NO_SUPPLY with a distinct "TPS writes NACK xN --
      supply likely brown-out" trigger string. Also incremented
      when _tps_only_recover fails both attempts (same root
      cause, different code path).

    - g_pd_clamped unreleasable when _tps_only_recover keeps
      failing. The failure leg deliberately preserved
      g_pd_clamped (to keep firmware state aligned with the
      chip), but that meant actuate skipped V/I writes (gated
      on !g_pd_clamped), the NACK counter never advanced from
      fresh failed writes, and the operator was stuck with a
      latched clamp and no path to recovery short of power
      cycling. Failure leg now also clears g_pd_clamped, so
      actuate tries fresh writes which either succeed (chip
      back) or surface as NACKs through the new fault path.

    Smaller items in the same round:
      * tps_setVoltageLimit early-returns when its VOUT_FS read
        NACKs — was writing garbage `fs` bits before returning
        false. Matches tps_setOutput's pattern.
      * TPS_VLIM_READ_FAIL #define moved above tps_getVoltageLimitMV
        so the function body uses the macro name, not the bare
        literal. Sentinel value is now defined in one place.
      * _supply_fault_last_bad_mv no longer overwritten on
        NACK ticks (those don't contribute to the trip; the
        per-tick DIAG line still records NACKs in the serial
        log). Prevents the trip print from mis-attributing a
        real low-Vlim trigger as "NACK during read burst".
      * _supply_fault_last_bad_mv now cleared in
        _tps_only_recover's success leg alongside the other
        supply-check state, so the trio resets atomically.
      * COMPACT_FAULT_MSGS FLT[6] print now handles all three
        cases (NACK-counter trigger, sentinel, real value) —
        was printing "last=65535" on the sentinel case.

    Second-pass adversarial review (this commit also closes the
    follow-up findings from that pass):

    - PD CLAMP block was discarding write returns and latching
      g_pd_clamped=true unconditionally. A NACK'd clamp write
      under brown-out would short-circuit ALL THREE fault paths
      (clamp re-fire, NOPSU via V_limit, NOPSU via NACK count)
      because actuate skips writes under clamp, gated branches
      go quiet, chip stays at full drive. Fixed: check both
      tps_setVoltageLimit and tps_setCurrentLimit returns, only
      latch g_pd_clamped if both ACK'd. On write NACK, log
      `WARN clamp write NACK -- supply brown-out, NOPSU will
      latch via NACK path` and leave g_pd_clamped false so the
      NACK counter accumulates normally.

    - NACK counter coupling to controller state — three related
      defects:
        (a) _pid_full_reset() was clearing _tps_write_nack_count
            on every enable / pid / mode gesture. The enable
            path runs _pd_reinit() -> _tps_only_recover() (which
            may bump the counter on failure) THEN _pid_full_reset
            (which wiped it). Recovery-leg increments dead on
            the enable path.
        (b) Same coupling meant mode-switch pot noise (poll runs
            every loop iteration via _set_mode) would wipe the
            counter every flicker, indefinitely deferring the
            FAULT[NOPSU] latch under a real brown-out.
        (c) The actuate else branch was zeroing the counter on
            every non-driving tick. Deadband-edge oscillation
            under a marginal-Vin chip resets the watchdog every
            other tick — same shape as the original
            silent-NACK bug.
      Fix: _tps_write_nack_count is now hardware-health state,
      decoupled from controller intent. Removed the zero from
      _pid_full_reset and from the actuate else branch. Natural
      success paths (writes_ok=true actuate tick, successful
      _tps_only_recover) clear it; a chip that has genuinely
      recovered sees writes_ok=true and the counter drops to 0
      without help.

    - _tps_only_recover silently released g_pd_clamped. CDC
      drift recovery at 1 Hz could disarm a deliberately-latched
      clamp with no operator-visible message. Fixed by clearing
      g_pd_clamped in both legs so the actuate stage can take
      over. NOTE: the explicit `DIAG: PD clamp released ...`
      print added during review was REMOVED in the final
      revision to fit the 30720 B flash budget (see flash
      budget note below); the release is now silent but
      observable indirectly via the resumed actuate writes.

    Flash budget note (bench, 2026-06-08, Arduino Nano,
    arduino:avr:nano:cpu=atmega328 → 30720 B sketch max):
    the v0.7.8 baseline shipped at 30206 B (98%, 514 B free).
    The full set of addendum fixes plus their reviewer-requested
    DIAG/WARN prints came in at 31516 B — 796 B over budget.
    To fit, the following messages were trimmed from the verbose
    branch (COMPACT_FAULT_MSGS=0, the default):
      * `DIAG: tps_init writes NACKed ...` (recover-failure leg)
      * `DIAG: TPS not on I2C -- attempting reinit` (task_safety)
      * Per-tick `DIAG: NoPSU counter=N/M Vlim=N mV` (task_safety
        supply-check; the trip-time `trg V=N` line below FAULT
        [NOPSU] still records the latch-point value)
      * `DIAG: PD clamp released by TPS recover (success/failure
        leg)` — both legs now clear silently
      * `WARN clamp write NACK -- supply brown-out, ...`
        (PD-clamp NACK leg now silent)
      * `  clamp latched until re-enable` (clamp success leg)
      * `DIAG: TPS CDC drift (got 0xN expected 0xN) -- chip
        reset detected, reinit` (CDC drift now silent)
      * FAULT[NOPSU] message body shortened from the original
        100-char operator-friendly form to
        `FAULT[NOPSU]: no USB-PD or Vin<12V`
      * Trip-time trigger string `  trigger: TPS writes NACK
        xN -- supply likely brown-out` shortened to `  trg
        NACKxN`; `  trigger: last bad Vlim = N mV` to
        `  trg V=N`
      * The dead `_supply_fault_last_bad_mv == TPS_VLIM_READ_FAIL`
        sentinel branches in both COMPACT and verbose trip
        prints removed (the value is never assigned to the
        sentinel — verified by reading every write site)
    Behavior is unchanged; only the human-readable string layer
    is terser. Final size: 30506 B (99%, 214 B free). If a
    future change adds another DIAG channel that pushes over,
    consider gating the v0.7.9 DIAG channel block behind a new
    NACK_DIAG_VERBOSE define so the strings come back for
    bench debugging on a non-flash-constrained build.

    - Documentation drift: the original Fix-1 paragraph
      (retry delay, state-clear list) referenced 2 ms and an
      outdated state-preserve list. Updated to match the final
      behavior (20 ms retry, only g_pd_clamped + NACK counter
      touched on failure, supply counters preserved). Stale
      comments in TPS55288.h tps_init() and CryoSnap.ino
      _pd_reinit() / _tps_only_recover() updated too.

  0.7.8  2026-06-05  PID rework — conditional integration, deriv-on-measurement, real dt

    Closes five PID-related items from the 2026-06-03 audit log
    in one coherent refactor of PID.h plus the task_100ms caller.

    - BUG-002 / PID-1 (high): the integrator wound up in the
      mode-forbidden direction. Cool mode + temp below setpoint
      meant the PID "wanted to heat"; caller post-clamped that
      output to 0 but the integrator kept growing. Bench-measured
      383x delay (118 s vs 0.31 s baseline) in the audit.
      pid_compute() now takes [out_min, out_max] bounds expressing
      the Mode constraint and freezes integration the moment the
      proposed total output would push past the forbidden bound.
      No residual builds in a direction the actuator can't use.

    - BUG-006 / PID-2 (medium): old anti-windup was a fixed clamp
      at +/-out_max/Ki, which let the integral grow up to its own
      clamp even while the proportional term was already pinning
      the output at the bound. Long settling tails after big
      setpoint steps. The new conditional-integration design from
      BUG-002 fixes this case for free: same anti-windup decision
      handles both hard PID saturation and direction-forbidden
      saturation through one check.

    - BUG-007 / PID-3 (medium, latent): derivative was computed
      on error, which produces a one-tick deriv = err/dt spike
      whenever the setpoint steps (and whenever pid_reset() runs
      with a real measurement off setpoint). Masked today because
      DEFAULT_KD = 0, but anyone raising Kd hit unexplained
      drive_mA kicks. Switched to derivative-on-measurement
      (mathematically equivalent for constant sp, free of the
      kick), plus a _pid_seeded flag that suppresses the
      derivative on the very first call after reset. New
      pid_observe(measured) helper called from the deadband
      branch keeps _pid_last_measured fresh so leaving the
      deadband doesn't produce a stale-derivative spike either.

    - BUG-008 / PID-4 (medium): task_100ms() passed
      dt = LOOP_INTERVAL_MS/1000 = 0.1, but real ticks stretch
      well past that (~100 ms fan tach poll on the slow task,
      5 ms direction-flip delay, long fault dumps). Ki and Kd
      effectively drifted with system load. Caller now measures
      actual elapsed time via millis() and passes it as dt,
      clamped to [0.05, 0.5] s so pathological scheduler stalls
      can't produce degenerate math. The first PID tick after
      boot, after _pid_full_reset, or after a deadband stay
      falls back to the nominal LOOP_INTERVAL_MS via a separate
      sentinel branch (no stale wall-clock delta).
      Follow-up to the adversarial review: _pid_last_ms is
      hoisted to file-static and cleared inside
      _pid_full_reset(), so every disable/enable, fault recovery,
      and `pid` toggle starts with a fresh dt sentinel. The
      deadband branch also advances _pid_last_ms each tick so
      leaving a multi-tick deadband stay doesn't see the full
      stay as one inflated dt.

    - BUG-002 follow-up (adversarial review): mode changes now
      drain the PID state via a _set_mode() helper at both
      writer sites (analog mode switch in task_50ms and the
      `mode` serial command). With conditional integration in
      place but no mode-change drain, an integrator accumulated
      under one Mode's bounds would unwind at err*dt per tick
      after the swap — measured-equivalent to BUG-002's
      original 100+ s residual delay just triggered by mode
      toggle instead of by mode-forbidden integration. The
      _set_mode() helper change-detects so steady analog reads
      don't keep wiping the integrator every 50 ms.

    - BUG-010 / PID-6 (low, latent): pid_compute had no guard
      against dt <= 0 or NaN setpoint/measurement. A future
      caller passing bad inputs would push inf/NaN into the TPS
      current command. Added early-return-zero on non-finite or
      non-positive inputs so a bad input can't poison the
      actuator.

    API change: pid_compute() signature changed from
      pid_compute(sp, m, dt, out_max)              // symmetric +/-
    to
      pid_compute(sp, m, dt, out_min, out_max)     // explicit bounds
    The only caller is task_100ms in CryoSnap.ino, updated to
    derive out_min/out_max from g_mode.

  0.7.6  2026-06-03  Fix TEC direction on Rev A + TPS-lost recovery
    - BUG-000: H-bridge direction was inverted on TARGET_REVA. The
      previous comment claimed Rev A wiring used LOW=Cool / HIGH=Heat
      and a bench audit on Rev A silicon confirmed the opposite —
      commanding HB_COOL was actively heating the cold plate. Both
      the PID and the bang-bang controllers were affected across all
      three Modes (Cool / Heat / Auto) because they all map drive
      direction through the same HB_COOL / HB_HEAT symbols. Fixed by
      aligning Rev A polarity with the prototype's
      HB_COOL=HIGH / HB_HEAT=LOW. TARGET_REVB inherits the same
      polarity but the inheritance is explicitly unverified — the
      build emits a #warning at compile time and a boot-time WARN
      banner under TARGET_REVB so the operator notices before
      trusting cooling commands on Rev B silicon.
    - BUG-003: FAULT_NO_SUPPLY was firing spuriously when the
      TPS55288 dropped off the I2C bus (Vin brownout during
      high-current cycling). _tps_read returns 0 on a NACK'd
      device, so tps_getVoltageLimitMV() decoded ~200 mV — below
      SUPPLY_VLIM_FLOOR — and accumulated the supply counter
      toward FAULT_NO_SUPPLY for the wrong reason. New
      tps_isPresent() helper (cheap ACK probe) gates the V_limit
      read. When the chip is absent during a drive session, the
      firmware logs `DIAG: TPS not on I2C -- attempting reinit`
      and runs a lightweight `_tps_only_recover()` (tps_init plus
      supply-state reset; deliberately does NOT touch HUSB fault
      tracking or call husb_init's 2 s blocking path, so the
      concurrent PD-drop fault stays armed and the 100 ms task
      budget is preserved). The TPS-present edge is re-armed when
      the chip comes back, when the firmware stops driving, or on
      the next enable cycle, so a future disappearance triggers a
      fresh DIAG + recovery. `g_pd_clamped` also clears as part of
      recovery since `tps_init()` resets the chip's V/I registers
      and any clamp the firmware had asserted is stale.
  0.7.7  2026-06-05  Audit-quick-wins: SCP gate, PD clamp, truncation, input clamps

    - BUG-001 / S-2: the FAULT_TPS_PG short-circuit latch was wrapped
      in `#if ENABLE_DIAGNOSTICS`. Diagnostics is advisory-only and
      MINIMAL_BUILD turns it off to reclaim flash, which silently
      removed a safety latch. Re-gated under ENABLE_SAFETY_FAULTS
      so any build with safety faults on now has SCP protection.
      Also tightened the SCP block: added `g_fault == FAULT_NONE`
      guard so an earlier fault (e.g. OVERTEMP) keeps its trip
      cause, and renamed the operator message from `DIAG:` to
      `FAULT[SCP]:` to match other latching faults.
      Behavior loss: a build with `ENABLE_DIAGNOSTICS=1` +
      `ENABLE_SAFETY_FAULTS=0` no longer gets the advisory SCP
      print either. Accept this for now — in practice safety
      faults are on for any build that cares about the SCP
      signal.

    - BUG-004 / C-1: the PD power-budget clamp was producing an
      off/on oscillation at up to ~10 Hz under sustained load. Old
      code wrote reduced V/I limits when over budget, then the
      actuate stage turned OE off entirely (gated on
      !g_pd_clamped); INA then read 0 W, the clamp cleared, the
      actuate stage re-wrote full drive_mA, power spiked, the clamp
      re-fired, repeat. Two changes: (a) actuate now keeps OE on
      under clamp but skips its V/I writes so the reduced limits
      stick; (b) the clamp latches — it no longer auto-clears when
      power dips, the operator releases it via the next enable
      cycle. `_pd_reinit()` clears `g_pd_clamped` so re-enable
      restores full drive. The clamp section also now gates on
      g_enabled so a noisy INA reading during the disabled window
      can't spuriously latch.
      Interaction fix: the supply-Vlim NOPSU check (added in
      0.7.5) reads the chip's V_limit register back and trips
      FAULT_NO_SUPPLY when it drops below 5500 mV. The PD clamp
      writes its own reduced V_limit which can land below that
      floor if the supply rail had drooped at trip — that would
      false-trip NOPSU within a few ticks of the clamp latching.
      Added `&& !g_pd_clamped` to the NOPSU check condition so
      the clamp's deliberate low V_limit doesn't get mistaken for
      the chip's auto-reset signal.

    - BUG-005 / CAL + PID-5: float-to-uint truncation in two
      places. (a) INA226 `_INA_CAL_VALUE` evaluated to 1023.9998
      and truncated to 1023 (0x3FF) instead of the correct
      1024 (0x400) — every current/power reading biased low by
      ~0.1%, feeding into the PD budget decision and the supply
      fault floor. (b) `tps_setCurrentLimit()` encoded
      `steps = (uint8_t)(v_sense / 0.0005f)` without rounding —
      40 mA truncated to 0 steps (full sub-50 mA dead zone), and
      2000 mA truncated to 39 steps = 1950 mA delivered. Added
      + 0.5f in both spots so the cast rounds to the nearest
      valid step.

    - BUG-009 / C-2: `imax <mA>` and `vmax <mV>` used `atoi()`
      (16-bit int on AVR) then a bare `(uint16_t)` cast. Bench
      evidence: `imax 70000` → echoed 4464 mA (the wrap of
      70000 mod 65536). Switched to `atol()` and clamped to the
      TPS-encodable ceilings (6350 mA / 20000 mV) before the
      cast, so over-range entries saturate instead of wrapping.

  0.7.5  2026-05-22  Sticky supply-fault counter — catches oscillating chip
    - The 0.7.4 supply-sufficiency check (periodic V_limit re-read
      with symmetric debounce-2) never fired on bench: the TPS55288
      oscillates between "trying to drive at the commanded V_limit"
      and "protected at 5 V" because the firmware re-writes V_limit
      every 100 ms tick. A symmetric debounce kept resetting to 0
      on the "trying" tick and never accumulated.
    - Drop the reset-on-good branch while the firmware is actively
      driving. The counter is now sticky: any two bad reads anywhere
      during a drive session latch FAULT_NO_SUPPLY. Good reads while
      not driving still reset (so a fresh enable starts clean).
    - No new config or behaviour change; just makes the detection
      that 0.7.4 intended actually work.

  0.7.4  2026-05-21  Direct-supply boot no longer trips PD fault
    - On a bench rig powered from a direct DC supply (e.g. 24 V
      into the barrel jack with no USB-C source attached), the
      HUSB238 reports unattached / 0 V. The FAULT_HUSB_20V chain
      was counting every tick toward the fault and latching
      within 500 ms of the first enable.
    - New sticky `_husb_was_attached` flag flips true the first
      tick HUSB reports any negotiated voltage (>= 5 V). The
      FAULT_HUSB_20V chain now only counts when the flag is set,
      so "never had PD" cannot fault. PD-loss after attach (the
      real failure case) still trips normally.
    - `_pd_reinit()` clears the sticky flag on every enable
      rising edge. This lets the operator re-enable after a PD
      fault and have the firmware reassess the supply from
      scratch — e.g. if the cable was swapped out for a direct
      DC supply, the next enable comes up clean instead of
      re-faulting on the stale "was attached" verdict.
    - No new config or console command; behaviour is automatic.

    - New FAULT_NO_SUPPLY (code 6) — catches the case where
      there is neither USB-PD nor a sufficient Vin (~12 V) to
      drive the TEC. Without this check the previous fixes
      correctly skip the false-positive PD fault but the user
      then enables and gets silent nothing because the TPS55288
      can't sustain the commanded voltage.
      Detection: when the firmware just wrote
      V_limit = g_vmax_mV and turned OE on, the TPS silently
      snaps V_limit back to its 5 V safety default if the supply
      can't actually sustain. The next task_100ms tick reads
      V_limit back; <= 5.5 V means the chip auto-reset. Two
      consecutive bad reads (200 ms) plus the same FAULT_GRACE_MS
      window the fan tach uses (3 s post-enable) latch the fault.
      Bench-validated: a low-current synchronous probe at enable
      did NOT trigger the chip's protection (it can hold 12 V
      into a tiny load), so the only reliable signal is to
      observe V_limit while the chip is actually driving at the
      user-configured current.
      Operator message:
        "USBPD not available and supply voltage is insufficient
         (connect to USBPD or Vin > 12v)"
      OLED + serial labels: "NoPSU".

  0.7.3  2026-05-11  `controller` one-shot user-function hook
    - Scaffolded "controller" console command that fires
      user_controller(elapsed_s) ONCE at a chosen elapsed time.
      Useful as a demo-prep trigger that adjusts setpoint / PID
      gains / mode at a known time.
    - New Controller.h:
        ctrl_arm()              fire one-shot at t=0 on next enable
        ctrl_start(fire_s)      fire NOW once at elapsed_s = fire_s
        ctrl_stop()             cancel a pending / armed fire
        ctrl_tick()             called every 100 ms from task_100ms;
                                fires when scheduled time reached
        user_controller(t_s)    the customisation point — empty by
                                default, edit in place for demo logic
    - Console:
        controller         arm; fire on next enable button press
        controller <s>     fire NOW at elapsed_s = s
        controller -1      cancel
    - Enable rising edge (button + serial cmd) calls ctrl_on_enable()
      to kick off any armed schedule.
    - One-shot semantics: user_controller runs exactly once per
      command. For multi-step profiles, the body can chain by
      calling ctrl_start(next_s) — example shown in Controller.h.

  0.7.2  2026-05-09  Align temp LED bar with setpoint bar range
    - The temperature bar (LEDs 1-10) used a fixed 0..50 C
      mapping while the setpoint bar already used the pot's
      POT_TEMP_MIN..POT_TEMP_MAX range (-10..+40 C). The temp
      column no longer lined up with NTC1 once the setpoint was
      below zero.
    - Map the temp bar through the same range as the setpoint
      bar so the two bars share a scale. Now you can eyeball
      the offset between cold-side and setpoint directly.

  0.7.1  2026-05-08  OLED: 128x32 panel support
    - OLED_HEIGHT config knob (Config.h) with 32 / 64 branches
      in the SSD1306 init sequence — picks the right MUX (0x1F
      vs 0x3F) and COM-pin config (0x02 vs 0x12). Default is 32
      to match the supplied panel. The previous build sent the
      64-line init to a 32-line panel, which mostly worked but
      drove the lower half of the framebuffer off-screen.
    - OLED_PAGES now derived from OLED_HEIGHT (4 pages on 32px,
      8 on 64px).
    - Render redesigned to a 4-row layout that fits both panels:
        row 0: "v0.7.1 ON Cool"
        row 1: "Set:25.0 T:23.4C"
        row 2: "1.23A 12.0V 14.8W"
        row 3: "Flt:- H:30.1C"
      On a 128x64 panel the bottom 4 rows stay blank.

  0.7.0  2026-05-06  PID controller (selectable at runtime)
    - New PID.h: minimal closed-loop controller, output in signed
      milliamps, anti-windup integrator clamp, no library deps.
      Sign convention: +mA = heat, -mA = cool, |out| = drive_mA.
    - task_100ms branches on g_use_pid: PID path or bang-bang +
      damping. Common deadband + Mode-direction guard apply to
      both. Bang-bang stays available for A/B comparison and as a
      fallback if PID tuning goes sideways.
    - Serial cmds:
        pid           toggle controller (or `pid 0|1` to set)
        kp / ki / kd  show or set tuning constants
      All persisted in EEPROM (SETTINGS_VERSION 2 -> 3, four new
      fields: use_pid, kp, ki, kd). Existing saved settings are
      rejected once on first boot.
    - PID state (integrator + last_err) resets on enable rising
      edge, on Ki change, and on `pid` toggle so a stale residual
      can't kick the TEC at restart.
    - status print now includes "Ctrl:PID/BB" and Kp/Ki/Kd row.
    - Defaults from Config.h: DEFAULT_USE_PID=true, Kp=200,
      Ki=5, Kd=0 (conservative starters; bench tune from there).

  0.6.2  2026-05-06  Latch fault on TPS SCP
    - When the TPS reports SCP (short
      circuit) the firmware now also drops g_enabled and latches
      FAULT_TPS_PG, instead of just printing the DIAG message.
      Reading the STATUS register clears the chip's SCP bit, so
      without latching we'd re-enable on the next tick and the
      chip would re-trip SCP if the short persists — looping
      forever. User has to clear the latch explicitly via the
      enable button or `enable 1`.
    - Status snapshot also auto-prints on the trip, matching the
      other latched-fault paths.

  0.6.1  2026-05-06  Move TPS SCP poll to 100 ms task
    - The lightweight TPS STATUS read for SCP detection moved
      into task_100ms right after the PD-clamp section so a short
      surfaces within 100 ms (was 1 s after the 0.6.0 split).
      Heavier presence pings (HUSB / TPS / INA) stay in task_diag
      at 1 s — their 25 ms timeout-on-missing-chip would blow the
      100 ms task budget if a chip ever drops.

  0.6.0  2026-05-06  Cross-chip diagnostics + serial command UX
    - Serial UX: typing a command without a number now reports the
      current value instead of silently writing zero. Affected:
      set, fan, mode, imax, vmax, maxhot, deadband, damping,
      damping_pct, bright. The `enable` command is special-cased
      to TOGGLE on no-arg (still requires console mode).
    - Read forms are allowed regardless of console mode; only
      writes to enable / set / mode are console-gated.
    - A0 (HW_TPS_FAULT) polling removed entirely —
      the chip handles its own catastrophic shutdowns. The fault
      chain no longer has a TPS arm; OCP_MASK + FB/INT are now
      a no-op for our control flow.
    - New ENABLE_DIAGNOSTICS subsystem (default on) — task_diag()
      runs in the slow task and emits state-change messages:
        * PD ok but TPS not responding (power switch / cable)
        * INA226 offline while TPS responds
        * TPS reports SCP (short circuit on TEC)
        * TPS driving but INA reads ~0 A (disconnected TEC)
      None latch a fault — purely advisory.
    - OLED probe failure now logs `WARN OLED` at boot (matches
      tps/ina/husb pattern). Firmware continues normally without
      a display fitted; every oled_*() call is gated by a runtime
      _oled_present flag set by oled_init().

  0.5.3  2026-05-05  Mask TPS55288 OCP indication on FB/INT
    - Per TPS55288 datasheet §7.6.5, OCP_MASK in the CDC register
      must be 0 around any OE 0->1 transition or the chip pulls
      FB/INT (= HW_TPS_FAULT on Rev A) LOW during the soft-start
      ramp into a load. We use current limit as a *normal*
      operating mode for TEC drive, so OCP_MASK stays cleared
      permanently. SC_MASK and OVP_MASK remain enabled — those
      are real catastrophes that should still surface as
      FAULT[TPS]. CDC register dump now decodes the mask bits.
    - Explains why the FAULT[TPS] trip showed up only on Rev A
      (FB/INT actually wired to A0) and not on the prototype
      (pin floated, internal pullup masked the signal entirely).

  0.5.2  2026-05-04  TPS PG grace + auto status print on fault
    - TPS55288 PG check now skips the first 200 ms after enable so
      the chip's soft-start ramp doesn't latch FAULT[TPS] on the
      very first post-enable tick. Real regulation faults still
      trip within 2-3 ticks once the grace expires.
    - When any fault latches, the full `status` snapshot is now
      printed automatically (sensor readings, mode, PD state).
      Saves typing `status` after every trip during bring-up.

  0.5.1  2026-05-01  OLED + Apr 30 hardware-test fixes merged
    - Brings the 0.4.1 dev fixes onto the OLED branch:
        - Adjustable LED brightness (default 64/255) — `bright`
          serial command, persisted via EEPROM (Settings v2)
        - Setpoint bar now maps the pot's POT_TEMP_MIN..POT_TEMP_MAX
          range to the 10-LED span; full-scale pot lights all 10
        - Wire.setWireTimeout(25 ms, reset) so a missing or stuck
          I2C device no longer freezes the boot scan or driver init
        - PD power-budget headroom raised from 90% to 95%
        - HW_TPS_FAULT now uses INPUT_PULLUP (was floating INPUT)

  0.5.0  2026-04-29  SSD1306 OLED status display
    - Custom framebuffer-free SSD1306 driver in OLED.h (text only,
      5x7 font in PROGMEM, ~150 B RAM total — vs ~1 KB for a full
      framebuffer driver).
    - 8-line status readout refreshed in the 1 s slow task: version,
      mode, setpoint, T_cold, T_hot, I, V, P, fan RPM, fault.
    - HAS_OLED set in both target pin maps (PROTO and Rev A).
    - ENABLE_OLED_DISPLAY flag (default 1) — auto-disables at runtime
      if the panel doesn't ACK at boot, so safe to leave on.
    - OLED_REFRESH_TICKS knob to slow the redraw cadence.
    - Cost when enabled: ~3.3 KB flash, ~10 B RAM.

  0.4.0  2026-04-27  Build-time feature flags + RAM trim + Rev A pin fix
    - ENABLE_* flags in Config.h gate every optional feature
      (debug regs, EEPROM, I2C scan, NTC cal, plotter, LED fade,
      fault LED, PD clamp, verbose boot, safety faults beyond OT)
    - MINIMAL_BUILD master switch: 14.7 KB flash / 664 B RAM
    - COMPACT_FAULT_MSGS prints "FLT[n]" instead of "FAULT[NAME]"
    - SerialCmd strncmp keys moved to PROGMEM (saves 134 B RAM)
    - Rev A: HW_LED_DAT corrected from D7 to D5 per the schematic
    - WS2812B.h derives port bit from HW_LED_DAT (was hard-coded)
    - Per-component memory report in MEMORY_REPORT.md

  0.3.0  2026-04-23  Bench feedback round
    - EEPROM save/load/defaults for persistent settings
    - Over-temp safety now checks all 3 NTCs (was hot-side only)
    - Deadband / damping / damping_pct user-modifiable via serial
    - D8 enable button INPUT_PULLUP (was floating on prototype)

  0.2.0  2026-04-22  Spec-complete + fault recovery
    - Fault LED slow-flash red per spec §24
    - Zero-deadband warning in Config.h per spec §19
    - README.md with implementation note per spec §30
    - PD re-initialization on every enable transition
    - HUSB238 PD fault debounced (5 ticks / 500 ms)
    - HUSB238 auto-renegotiate on first voltage drop
    - Fan tach fault grace period (3 s) after enable
    - INA226 clipping detection with ">" prefix and V*I_limit estimate
    - Plotter fields now include units + signed Drive_mA (cool/heat)
    - NTC calibration commands (cal, cal1, cal2, calshow)
    - Console mode to disable physical inputs
    - H-bridge polarity target-aware (inverted on PROTO)
    - Safe startup: H-bridge pin forced low before anything else
    - TPS I_limit decoded in register dump

  0.1.0  2026-04-21  First functional control pipeline
    - Bang-bang control with deadband (0.2 C) and damping (3.0 C / 50%)
    - NTC 3-channel read with 10-sample running average
    - Serial commands: help, status, enable, set, fan, imax, vmax, mode, maxhot, stream
    - Enable button debounce (D8 active-low) + mode switch (A6 analog)
    - Pot -> setpoint (-10 to 40 C range, always live)
    - WS2812B 23-LED bit-bang driver (AVR inline asm, zero deps)
    - Full LED mapping: temp bar (1-10), setpoint bar (11-20), mode/enable/HB indicators (21-23)
    - Fan 25 kHz hardware PWM (Timer2 OC2B) + polled tach with consistency filter
    - Fan passive cooling at 50% when TEC disabled
    - Hot-side over-temp safety (NTC2 > maxhot -> TEC off)
    - TPS55288 V/I/output control, current limiter ON
    - INA226 current/voltage/power read (100 mOhm shunt, 16A range, CAL=102)
    - HUSB238 USB-PD negotiation (20V/5A, PDO ladder walk, renegotiation on low current)
    - USB-PD power budget protection (90% clamp via INA226)
    - H-bridge direction control with safe direction-change sequence
    - I2C bus scanner with device annotation at boot
    - Register dump for TPS/INA/HUSB (behind DEBUG_DUMP_REGISTERS flag)
    - Dual build target: TARGET_PROTO (bench breadboard) / TARGET_REVA (production PCB)
    - Adafruit HUSB238 debug path (USE_ADAFRUIT_HUSB flag)

  0.0.1  2026-04-16  Skeleton + I2C bring-up
    - Project skeleton: Config.h, Pins.h, driver header stubs
    - I2C scanner confirming TPS55288 @ 0x75, INA226 @ 0x40, HUSB238 @ 0x08
*/

#endif // VERSION_H
