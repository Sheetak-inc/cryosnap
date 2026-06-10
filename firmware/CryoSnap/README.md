# CryoSnap — Firmware

Reference / teaching firmware for the CryoSnap thermoelectric (TEC)
development platform. Compiles in the stock Arduino IDE against the
ATmega328P (Arduino Nano Classic). No external libraries required
for the default build.

**Version:** see [`Version.h`](Version.h)

## Quick start

1. Open `CryoSnap.ino` in the Arduino IDE.
2. Select **Arduino Nano** / **ATmega328P** from Tools → Board.
3. Upload. Open the Serial Monitor at **115200 baud**.
4. You should see the version banner (e.g. `CryoSnap v0.7.11 REVA`)
   followed by a couple of HUSB238 PD-negotiation lines. The default
   build is intentionally terse — boot-time I2C scan, EEPROM
   blank/loaded chatter, status dump, and PD-budget echo are silent.
   Compile with `-DENABLE_VERBOSE_BOOT=1` and/or
   `-DENABLE_I2C_BOOT_SCAN=1` for the chatty bring-up build, or
   send `status` over serial at any time for the full snapshot.
5. Type `help` + Enter to see all serial commands.

Default control runs from physical inputs (pot = setpoint, A6 mode
switch, D8 button). Type `console` to switch to serial-only control.

## File layout

The sketch is split into focused header-only modules. Drivers are
named for the chip; render / scheduling helpers describe their job.
Everything is `inline` so the project still compiles as a single
translation unit — the "open and compile" story stays trivial.

### Top-level

| File | Purpose |
|---|---|
| `CryoSnap.ino` | Main story: setup, loop, fast/100 ms/1 s task buckets. Reads top-to-bottom. |
| `Config.h` | All tunables: pins, defaults, safety limits, feature flags (`ENABLE_*`). The maker edits this to customize behavior. |
| `Pins.h` | Board-target switch (`TARGET_PROTO` / `TARGET_REVA` / `TARGET_REVB`) exporting canonical `HW_*` pin names. |
| `Version.h` | Firmware version string + changelog. Bump on each release. |

### Control

| File | Purpose |
|---|---|
| `PID.h` | Closed-loop PID with anti-windup integrator clamp. Default control path (`DEFAULT_USE_PID = true`). Includes a tuning recipe in the file header. |
| `Controller.h` | One-shot `user_controller(elapsed_s)` hook fired by the `controller` console command — for demo-prep parameter sweeps. Empty by default; edit in place. |
| `Settings.h` | EEPROM persistence (magic + version + CRC). `save` / `load` / `reset` commands. Bump `SETTINGS_VERSION` on any layout change. |

### Drivers (one chip per header)

| File | Purpose |
|---|---|
| `TPS55288.h` | I2C buck-boost converter that drives the TEC. |
| `INA226.h` | I2C current/voltage/power sensing on the TEC rail. |
| `HUSB238.h` | I2C USB-PD sink controller (negotiates 20 V). |
| `HBridge.h` | DRV8701E direction control via single GPIO. |
| `FanControl.h` | Hardware-PWM fan speed + polled tach RPM. |
| `WS2812B.h` | Bit-banged driver for the 23-LED status chain. |
| `NTC.h` | 3-channel thermistor read with running average + calibration. |
| `OLED.h` | SSD1306 128x32 / 128x64 driver (zero-dep). Compiles to stubs when `ENABLE_OLED_DISPLAY = 0`. |
| `I2CScan.h` | Bus scanner — prints every device that ACKs at boot. |

### Rendering & UX

| File | Purpose |
|---|---|
| `LedRender.h` | Builds the WS2812B status frame (temp bar + setpoint + indicators + fault flash). |
| `OledRender.h` | Paints the 4-row OLED status frame. |
| `Diagnostics.h` | `task_diag()` — advisory cross-chip state-change reporter (logs INA/TPS/HUSB transitions on the slow task). |
| `SerialCmd.h` | Serial command parser + `status` print. Read-on-no-arg pattern: `set` prints the setpoint, `set 25.0` writes it. |

## Customising the firmware

Each header that's intended to be extended has a numbered recipe in
its top comment block. If you want to:

- **Add a serial command** — see the recipe at the top of `SerialCmd.h`.
- **Persist a new tunable across reboots** — see `Settings.h` (and bump `SETTINGS_VERSION`).
- **Add an `ENABLE_*` build-time feature flag** — see the block in `Config.h`.
- **Tune the PID for new hardware** — see the "HOW TO TUNE" block in `PID.h`.
- **Change task periods or add a periodic task** — see the comments above `loop()` in `CryoSnap.ino`.
- **Extend cross-chip diagnostics** — see `Diagnostics.h`.
- **Customise the LED frame or OLED layout** — see the top of `LedRender.h` / `OledRender.h`.
- **Run a one-shot demo profile** — edit `user_controller()` in `Controller.h` and trigger via the `controller` serial command.

## Execution model

Pure cooperative scheduling with `millis()`, no RTOS. Three task
buckets in `loop()`:

- **Fast tasks** (every iteration): serial command parser, enable
  button debounce, mode switch read, pot → setpoint.
- **100 ms control task** (`LOOP_INTERVAL_MS`): NTC read, INA226
  read, fault gate, PID compute (or bang-bang + damping fallback),
  Seebeck wait state machine (cooperative, holds OE off for up to
  3 s on a polarity flip without stalling the scheduler), TPS V/I
  limit, H-bridge direction, LED render, plotter line, `ctrl_tick()`
  for the one-shot controller hook.
- **1 s slow task**: fan tach polling (blocks ~100 ms while timing
  edges), HUSB238 PD status re-check, OLED render, `task_diag()`
  cross-chip advisory checks.

There is no ISR in the default build. The fan tach is polled in the
slow task so noise can be properly debounced — an ISR-based counter
is documented in `FanControl.h` as a future enhancement if the tach
moves to a Timer1 counter input.

## Control: PID (default) and bang-bang (fallback)

The 100 ms task runs one of two control laws based on `g_use_pid`:

- **PID** — `pid_compute()` returns a signed mA command; sign maps
  to H-bridge direction, magnitude to TPS current limit. Conditional
  integration plus an explicit mode-bound clamp keep the integrator
  from accumulating in a direction the actuator can't use. Derivative
  is taken on the measurement (not the error) so setpoint steps don't
  produce a kick. Conservative starter defaults are `Kp=200 Ki=5 Kd=0`
  (Config.h `DEFAULT_KP / DEFAULT_KI / DEFAULT_KD`); see `PID.h` for
  the bench-tuning recipe, then `save` to persist your numbers.
- **Bang-bang + damping** — simpler fallback control law, reachable
  via the `pid` console command for A/B comparison. Drives full
  current outside the damping band, scaled current inside, off
  inside the deadband. Note: bang-bang combined with Mode `Auto`
  produces limit-cycle oscillation around setpoint and will trigger
  the Seebeck wait state machine on every crossing. For steady-state
  regulation use Mode `Cool` or `Heat` (single-direction clamp) — or
  switch to PID, which is structurally immune to this interaction at
  the default tuning.

Switch live: `pid` toggles, `pid 1` forces PID, `pid 0` forces
bang-bang. `save` persists the choice to EEPROM.

## Persistence

`Settings.h` writes a CRC-checked struct to EEPROM (~60 B at
address 0). Console commands:

- `save` — write current runtime state to EEPROM.
- `load` — read EEPROM and apply.
- `reset` — wipe EEPROM and restore Config.h defaults.

The struct is wrapped with magic + version + CRC, so a blank or
corrupted EEPROM at boot falls back to Config.h defaults instead of
bricking the controller. Bump `SETTINGS_VERSION` whenever you change
the layout — old EEPROMs are then rejected at boot until the
operator runs `save` once.

## Design principles

- Every peripheral has its own header, named for the chip. A maker
  who wants to understand the INA226 opens `INA226.h` and sees the
  whole story in one file.
- `Config.h` is the single source of truth for pin numbers and
  tunables. Everything a maker might want to change is in one place.
- `Pins.h` handles the build-target board split via a single
  `BUILD_TARGET` define — no code duplication across boards.
- Plain C-style functions throughout. No class hierarchies, no
  inheritance, no polymorphism — a reader doesn't need to trace
  virtual dispatch to follow the control flow.
- No ISR in the default build. Fan tach is polled for robustness
  against PWM noise; an ISR or hardware-counter path is documented
  in `FanControl.h` for boards that need it.

## Open hardware assumptions (stubs or unconfirmed)

- **INA226 100 mOhm shunt** — the prototype has 100 mOhm, which
  saturates at 0.82 A. 10 mOhm resistors are on order. `INA_RSENSE`
  in Config.h must be updated to `0.010f` when swapped.
- **External AREF** — `USE_EXTERNAL_AREF = 1` calls
  `analogReference(EXTERNAL)`. Prototype boards expect AREF wired to
  3.3 V; production boards tie AREF to the 5 V rail. Verify the
  schematic matches your board before relying on NTC readings.
- **H-bridge polarity on prototype is inverted** (LOW = Heat). Rev A
  is expected to be correct (LOW = Cool). `HBridge.h` handles the
  swap via `BUILD_TARGET`; confirm on Rev A silicon.
- **NTC calibration** is empirical from the bench prototype, not yet
  from a traceable reference thermometer. Use `cal` / `cal1 / cal2`
  commands to recalibrate and update Config.h constants when ready.
- **Mode switch** on A6 may float if no switch is connected. Rev A
  schematic should pull A6 to a default zone.
- **USB-PD brownout recovery** — `_pd_reinit()` on enable is a
  software workaround for the prototype's 5 V regulator feedback.
  The hardware fix (removing the onboard 5 V regulator so the MCU
  is powered only by the computer USB) is tracked separately.

## Build targets

Set `BUILD_TARGET` in `Pins.h` (default is `TARGET_REVA`):

- **`TARGET_PROTO`** — bench breadboard prototype. Uses the pin
  values in Config.h, TPS55288 at I2C 0x75, inverted H-bridge
  polarity.
- **`TARGET_REVA`** — first-spin production PCB. Schematic-verified
  pinout, TPS55288 at I2C 0x74, correct H-bridge polarity.
- **`TARGET_REVB`** — next-spin production PCB. Same I2C address as
  Rev A but moves fan tach to D5 (Timer1 T1 input), INA Alert to D2,
  LED data to D4, and removes the discrete TPS fault pin (the chip
  handles its own shutdowns and firmware polls TPS STATUS over I2C).

## Footprint

Default build (PID + OLED + diagnostics + Seebeck SM; verbose-boot
and I2C-scan default OFF):

| Target | Flash | RAM |
|---|---|---|
| `TARGET_REVA` (default) | 30696 B / 99.9% (24 B free) | 805 B / 39% |
| `TARGET_PROTO` / `TARGET_REVB` | within ~100 B of REVA | — |
| `MINIMAL_BUILD = 1` | ~22 KB / 71% | ~600 B / 29% |

The default REVA build is tight against the Nano's 30720 B ceiling.
Adding a serial command or DIAG channel typically costs 50–200 B per
feature. If a new addition pushes over, reclaim space by setting
one of: `ENABLE_VERBOSE_BOOT=0` (already off by default; ~380 B),
`ENABLE_I2C_BOOT_SCAN=0` (already off; ~300 B),
`ENABLE_SEEBECK_TRACE=0` (drops the `DIAG: Seebeck` lines; ~150 B),
or `SEEBECK_HB_OFF_MAX_MS=0` (removes the polarity-flip mitigation
entirely; ~290 B). `MINIMAL_BUILD` is the all-at-once switch.
