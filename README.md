# CryoSnap

> **Microscope-stage branch.** This branch is kept for published links. The
> microscope stage build is in [`hardware/microscope-stage/`](hardware/microscope-stage/)
> here and in [`applications/microscope-stage/`](https://github.com/Sheetak-inc/cryosnap/tree/main/applications/microscope-stage) on `main`.

CryoSnap is a Sheetak thermoelectric validation platform for moving from bench testing to product integration faster. It is intended to help builders, engineers, researchers, and product teams start from a working TEC control system, collect useful thermal data earlier, and reuse validated architecture where appropriate.

> **Status:** Public reference design / community edition.
>
> CryoSnap is made available to encourage personal, educational, research, evaluation, and internal use. It is **source-available**, not an OSI-approved open source project. Modifications are allowed. Attribution is required. Commercial sale, paid distribution, or integration into a commercial product requires prior written authorization from Sheetak.

## Start here

- **Update your kit's firmware:** download the `.hex` for your setup from the
  [latest release](https://github.com/Sheetak-inc/cryosnap/releases/latest),
  or build from source with the Arduino IDE
  ([firmware/CryoSnap/](firmware/CryoSnap/README.md)).
- **First use:** [docs/getting-started.md](docs/getting-started.md) and
  [docs/safety.md](docs/safety.md).
- **Log and plot data on a PC:** [tools/cryosnap-monitor/](tools/cryosnap-monitor/README.md).
- **What changed:** [CHANGELOG.md](CHANGELOG.md).

## What is in this repository

| Folder | Contents |
|---|---|
| [`firmware/CryoSnap/`](firmware/CryoSnap/) | Arduino firmware. One source tree; `Profile.h` selects the hardware. |
| [`applications/`](applications/) | Worked builds on top of the kit: firmware profile, parts and results. |
| [`tools/`](tools/) | PC-side tools (CryoSnap Monitor). |
| [`docs/`](docs/) | Getting started and safety. |
| [`hardware/`](hardware/) | Hardware design files and their license terms. |

### Applications

| Build | What it does | Firmware profile |
|---|---|---|
| [MicroCENTUM micro cooler](applications/microcentum/) | Drives a Sheetak MicroCENTUM MC-18 to −25 °C from the stock kit | `PROFILE_MICROCENTUM_MC18` |
| [Microscope sample stage](applications/microscope-stage/) | Heats and cools a 35 mm petri dish under a microscope, −5 to 95 °C | `PROFILE_MICROSCOPE_STAGE` |


## License summary

This is a summary only. See [LICENSE.md](LICENSE.md), [COMMERCIAL.md](COMMERCIAL.md), [ATTRIBUTION.md](ATTRIBUTION.md), [NOTICE.md](NOTICE.md), and [TRADEMARKS.md](TRADEMARKS.md) for the controlling terms.

### Firmware

CryoSnap firmware is licensed under the **PolyForm Noncommercial License 1.0.0**, with an additional Sheetak permission for internal evaluation, research, development, prototyping, testing, education, and internal lab use.

Commercial use of CryoSnap firmware, including sale, paid distribution, integration into commercial products, or use in paid commercial offerings, requires prior written authorization from Sheetak.

### Hardware design files

CryoSnap hardware design files are made available under the CryoSnap Hardware Community Terms in [LICENSE.md](LICENSE.md).

You may copy, build, and modify CryoSnap-derived hardware for personal, educational, research, evaluation, and internal use, subject to attribution, notice preservation, safety responsibility, and the non-commercial restrictions in [LICENSE.md](LICENSE.md).

Commercial sale, paid distribution, contract manufacturing for third parties, or integration into a commercial product requires prior written authorization from Sheetak.

## Attribution

Attribution is required.

Suggested attribution:

```text
Based on CryoSnap by Sheetak, Inc.
Used under the CryoSnap community license terms.
```

For modified versions:

```text
Derived from CryoSnap by Sheetak, Inc.
Modified by [your name or organization].
Sheetak does not endorse or certify this derivative work.
```

See [ATTRIBUTION.md](ATTRIBUTION.md).

## Commercial use

Commercial use requires prior written authorization from Sheetak. Authorization may be free, paid, pilot-specific, production-specific, or partnership-based depending on the use case.

See [COMMERCIAL.md](COMMERCIAL.md).

## Safety

CryoSnap involves thermoelectric modules, driver electronics, temperature sensors, heat rejection, and powered hardware. You are responsible for safe assembly, operation, testing, electrical limits, thermal limits, regulatory compliance, and suitability for your application.

CryoSnap is provided without warranty. Do not use it in life-critical, safety-critical, medical, aerospace, automotive, or other high-risk applications without a separate written agreement with Sheetak and appropriate engineering validation.

## Contributing

Contributions are welcome for issues, documentation, examples, firmware improvements, and hardware design improvements. By contributing, you agree to the contribution terms in [CONTRIBUTING.md](CONTRIBUTING.md).

## Security and safety reports

Please report security vulnerabilities, unsafe behavior, or hardware/firmware issues that could cause overheating, overcurrent, electrical hazards, data exposure, or unsafe operation according to [SECURITY.md](SECURITY.md).

## Contact

- Commercial licensing: `info@sheetak.com`
- Security or safety reports: `info@sheetak.com`
- General CryoSnap questions: `info@sheetak.com`
