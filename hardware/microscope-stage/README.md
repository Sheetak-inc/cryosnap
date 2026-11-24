# Temperature-controlled microscope sample stage

> **Moved.** The microscope stage now lives on `main` in
> [`applications/microscope-stage/`](https://github.com/Sheetak-inc/cryosnap/tree/main/applications/microscope-stage),
> and its firmware is the `-stage` profile of CryoSnap v2.1.0 or later. Download
> prebuilt firmware from [the latest release](https://github.com/Sheetak-inc/cryosnap/releases/latest).
> This branch is kept so published links keep working. It now carries v2.1.0
> with the microscope-stage profile selected in `firmware/CryoSnap/Profile.h`.

A heated and cooled stage that holds a 35 mm petri dish under a microscope. One
thermoelectric module does both directions, the control loop reads a thermistor
in the water rather than on the plate, and the whole thing is built from a
CryoSnap development kit, three printed parts and commodity hardware. Nothing is
machined and no circuit board was designed.

It was built for hydrogel swelling and shrinking measurements, where every image
has to be paired with a water temperature measured at the moment of exposure.
The same approach applies to any small-vessel instrument that has to hold a
sample at a known temperature.

## What it does

| | |
|---|---|
| Range reached | −5 °C to 95 °C in the water |
| Agreement with a Type K reference | 2.7 °C maximum, 1.1 °C RMS, 95 % of points inside 1.67 °C |
| Conditions | 22 °C room, 100 W USB-C PD supply, `imax 3000`, `vmax 12000` |
| Vessel | 35 mm borosilicate dish, 2 to 3 mL of water, printed lid on |

The requirement it was built against was 10 °C to 80 °C at 1 °C setpoint
resolution, for runs of five minutes to an hour.

## What is on this branch

**`hardware/microscope-stage/`** holds the three printed parts, each as
SolidWorks, STEP and STL:

| Part | Role |
|---|---|
| `carrierv3Solo` | Dish carrier. Clamps the module and spreader to the cooler and locates the dish over the module. The dish itself is never clamped. |
| `lid2` | Printed petri lid. Closes the evaporation path and carries the control thermistor. |
| `sensorholder` | Holds the thermistor so the bead sits in the water clear of the glass floor. |

**`firmware/CryoSnap/`** is CryoSnap v2.1.0 with the microscope-stage profile
selected. Earlier copies of this branch carried v0.8.1; v2.1.0 uses the same
probe calibration table and adds the v2 control and converter fixes. Two
features matter here:

- `NTC.h` carries a 26-entry non-linear calibration table indexed on raw ADC
  counts, with a fitted correction baked in. The two-point linear conversion it
  replaces read about 23 °C low at the top of the range.
- `pause` and `resume` freeze the cold-side reading so the loop does not chase
  the probe while the lid is off for imaging. Hot-side and ambient channels stay
  live, so the over-temperature fault is never blinded.

`tools/ntc-calibration/fit_ntc_lut.py` regenerates the table for your own sensor
from its datasheet R-T curve and a logged reference run.

## Building one

The rest of the stack is off the shelf: a CryoSnap dev kit, a 127-couple
thermoelectric module, a low-profile CPU cooler with a slim PWM fan, a copper
spreader bar, metal-free thermal grease, a 100 W USB-C PD supply, a 10 kΩ curve F
thermistor and four 3 mm plastic screws.

Things worth knowing before you start:

- **Print in nylon**, or another material rated above the highest setpoint you
  will run. The first set was printed in a standard filament and softened during
  the characterization sweep at the top of the range.
- **Buy flat dishes and check them.** Nothing presses the dish onto the module,
  so the contact is whatever the two flat faces give you. A bowed floor or a
  raised rim leaves an air gap worth more thermal resistance than the glass.
- **Use plastic screws** for the carrier. They stay compliant and do not short
  the hot face to the cold one.
- **Calibrate in the running configuration**, probe at its working depth, dish
  filled, lid on, with the reference probe beside it in the same water.
- **Add a hot-side thermistor** on one of the spare NTC channels. This build
  populates only NTC1, so `maxhot` has nothing to trip on if the fan stalls.

## Limits

Stability at the probe, uniformity across the dish and contact repeatability
across dish reinstalls are not characterized. The thermistor has no continuous
immersion rating and an earlier one failed in the liquid path, so qualify it for
your own duty cycle. The calibration is tied to the thermal load it was fitted
under; change the dish, the fill depth or the duty cycle materially and part of
the correction no longer holds.

## Status

This branch is kept so published links keep working. Current development is on
`main` in `applications/microscope-stage/`.
