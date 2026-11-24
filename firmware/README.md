# CryoSnap firmware

Pick the version for your hardware. Each version is ready to use: there is
nothing to select or edit before you upload it.

| Version | Use it for | Source folder | Download |
|---|---|---|---|
| **Stock dev kit** | The CryoSnap kit as shipped (127-couple module, Cool / Heat / Auto) | [`stock/CryoSnap/`](stock/CryoSnap/) | [latest release](https://github.com/Sheetak-inc/cryosnap/releases/latest) |
| **MicroCENTUM MC-18** | A Sheetak MicroCENTUM micro cooler on the H-bridge bypass terminals ([application](../applications/microcentum/)) | [`microcentum-mc18/CryoSnap/`](microcentum-mc18/CryoSnap/) | [latest release](https://github.com/Sheetak-inc/cryosnap/releases/latest) |
| **Microscope stage** | The temperature-controlled petri-dish stage ([application](../applications/microscope-stage/)) | [`microscope-stage/CryoSnap/`](microscope-stage/CryoSnap/) | [latest release](https://github.com/Sheetak-inc/cryosnap/releases/latest) |

All versions are for the **Rev B** CryoSnap board.

## Two ways to install

**Upload the prebuilt firmware.** On the
[latest release](https://github.com/Sheetak-inc/cryosnap/releases/latest),
download `cryosnap-v<version>-<name>.hex` for your version and upload it to the
Arduino Nano (ATmega328P) with avrdude or a hex uploader. Check the file against
`SHA256SUMS.txt`.

**Build from source in the Arduino IDE.** Download
`cryosnap-v<version>-<name>.zip` from the release and unzip it, or copy the
version's folder from this repository. Open `CryoSnap/CryoSnap.ino`, select
**Arduino Nano** / **ATmega328P**, and upload. Open the Serial Monitor at
115200 baud; the boot banner shows the version, for example `CryoSnap v2.1.0-mc18`.

How the firmware is organized, and how to change it, is in each version's
`CryoSnap/README.md`. Release notes: [CHANGELOG.md](../CHANGELOG.md).
