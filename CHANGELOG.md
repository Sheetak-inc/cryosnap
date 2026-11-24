# Changelog

CryoSnap firmware release notes, newest first. The version prints at boot and
on the OLED status line.

## 2.1.0

**One download per hardware version.** Pick your version in `firmware/` or on
the release page; each is ready to upload, with nothing to select or edit.

- **Stock**: the CryoSnap dev kit as shipped. Same behavior as 2.0.0.
- **MicroCENTUM MC-18** (`-mc18`): drives a Sheetak MicroCENTUM micro cooler on
  the H-bridge bypass terminals. Cool mode only, 1.7 A / 2.28 V limits, PID
  tuned for the cooler's fast response, and a feedforward preload that starts
  each setpoint change at the measured hold current. See
  [`applications/microcentum/`](applications/microcentum/).
- **Microscope stage** (`-stage`): Amphenol TK95F probe calibration and the
  `pause` / `resume` commands, which freeze the cold-side reading while the
  probe is out of the sample. See
  [`applications/microscope-stage/`](applications/microscope-stage/).

Each version keeps its own saved settings. Switching versions on a board
starts from that version's defaults until you `save`.

## 2.0.0

- **Calibrated stock sensor.** A lookup table for the kit's MF55 thermistor,
  fitted against a Type K reference from −5 to 80 °C (0.6 °C max error). The
  previous linear conversion read several degrees off at the ends of the range.
- Readings just outside the table extend linearly instead of dropping out.
- **Better holds below ambient.** Higher default integral gain (Ki 15); the
  deadband now applies to bang-bang control only.
- **Fewer polarity reversals near ambient** in Auto mode: a direction guard
  waits before reversing unless the error is large.
- **Light-load converter switching**: the regulator follows the commanded
  current closely from 0 to 2 A.

## 0.8.0

- Non-linear thermistor calibration through a lookup table, with
  `NTC_SENSOR` selecting the table. `cal`, `cal1` and `cal2` now trim the
  table output.
- Saved settings from earlier versions are reset once (format change).

## 0.7.x

Robustness work on the power path and control loop:

- PID reworked: conditional integration (anti-windup), derivative on
  measurement, real time step.
- Soft-start current ramp at enable.
- Polarity reversal handled safely on both board revisions, including the
  thermoelectric module's Seebeck voltage after a reversal.
- Supply detection: insufficient or missing USB-C PD supply is detected and
  reported instead of faulting later.
- No false short-circuit faults at low output voltage.
- Recovery when the regulator drops off the I2C bus.

## 0.1.0 – 0.6.x

First control pipeline, fault handling and recovery, OLED status display
(128×64 and 128×32), runtime-selectable PID, serial command console, and
build-time feature flags.
