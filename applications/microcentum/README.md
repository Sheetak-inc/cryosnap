# CryoSnap with a MicroCENTUM micro cooler

Drive a Sheetak MicroCENTUM micro thermoelectric cooler from a stock CryoSnap
development kit: the same board, heat sink, fan and sensor, with a firmware
profile tuned for the cooler's low voltage and fast thermal response.

## What it does

Measured on one MC-18 with the stock CryoSnap heat sink, fan and sensor:

| | |
|---|---|
| Lowest cold-side temperature | −25 °C, nearly 50 °C below ambient |
| Hold at setpoint | within 0.1 °C on average; 0.3 °C peak to peak at cold setpoints |
| Setpoint change | overshoot typically under 0.5 °C; new setpoint reached in under 5 s |

Only the MC-18 is validated. The MC-08, MC-24 and MC-36 should run with a
different voltage limit set at startup (below).

## Wiring

Connect the MicroCENTUM to the **H-bridge bypass terminals** only. The profile
locks the controller in Cool mode with fixed polarity; Heat and Auto are
disabled.

## Firmware

Use the standard CryoSnap firmware in [`firmware/CryoSnap/`](../../firmware/CryoSnap/)
with the MicroCENTUM profile. In `Profile.h`, set:

```c
#define CRYOSNAP_PROFILE  PROFILE_MICROCENTUM_MC18
```

or download the prebuilt `-mc18` firmware from the
[latest release](https://github.com/Sheetak-inc/cryosnap/releases/latest).

The profile changes these defaults from the stock kit:

| Setting | Stock kit | MC-18 profile |
|---|---|---|
| Current limit | 2.0 A | 1.7 A |
| Voltage limit | 12 V | 2.28 V |
| PID Kp / Ki | 200 / 15 | 17 / 9 |
| Mode | Cool / Heat / Auto | Cool only |
| Pot range | −10 to 40 °C | −30 to 40 °C |

It also adds a **feedforward preload**: on every setpoint change the controller
starts from the measured current needed to hold the new setpoint, instead of
from zero, and the PID trims around it. The preload table in `Feedforward.h`
was measured on the stock CryoSnap heat sink and fan at about 23 °C ambient. A
different heat sink or a large heat load shifts the hold current; the PID
still converges, with more overshoot.

## Other MicroCENTUM models

At the serial console, type `vmax` followed by the limit in mV, then `save`:

| Model | `vmax` (mV) |
|---|---|
| MC-08 | 1000 |
| MC-18 | 2280 (default) |
| MC-24 | 3000 |
| MC-36 | 4600 |

## Limits

- One MC-18, one heat sink, one ambient. Results on other units and fixtures
  are not yet characterized.
- No long-duration or cycling data yet.
- Multistage MicroCENTUM (C3) units are not supported.
