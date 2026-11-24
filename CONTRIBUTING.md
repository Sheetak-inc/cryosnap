# Contributing to CryoSnap

Thank you for helping improve CryoSnap.

Contributions may include bug reports, documentation fixes, examples, firmware changes, PCB improvements, issue reproduction, test results, safety observations, and application notes.

## Before contributing

Please read:

- [LICENSE.md](LICENSE.md)
- [COMMERCIAL.md](COMMERCIAL.md)
- [ATTRIBUTION.md](ATTRIBUTION.md)
- [NOTICE.md](NOTICE.md)
- [TRADEMARKS.md](TRADEMARKS.md)
- [SECURITY.md](SECURITY.md)

## Contribution license terms

By submitting a contribution to this repository, you represent that you have the right to submit it and that it does not violate any third-party rights or confidentiality obligations.

You grant Sheetak, Inc. a perpetual, worldwide, non-exclusive, irrevocable, royalty-free license to use, copy, modify, publish, distribute, sublicense, relicense, make, have made, sell, offer for sale, import, and otherwise exploit your contribution, including in commercial products, services, internal projects, public releases, private releases, and separately licensed versions.

You also agree that your contribution may be made available to the public under the same license terms that apply to the relevant part of the repository, unless Sheetak states otherwise.

If Sheetak later uses your contribution in a commercially licensed version, product, service, or partnership, no additional permission or compensation is required unless Sheetak has separately agreed in writing.

## Developer Certificate of Origin

By contributing, you certify that at least one of the following is true:

- you created the contribution and have the right to submit it
- the contribution is based on prior work that is compatible with this repository's terms
- the contribution was provided to you by someone who certified one of the above
- you understand and agree that the contribution and project record are public

Use signed-off commits where practical:

```bash
git commit -s
```

## What not to contribute

Do not submit:

- confidential information
- trade secrets
- customer data
- export-controlled materials
- third-party proprietary code or designs
- code copied from incompatible licenses
- GPL, AGPL, LGPL, or other reciprocal-license code without prior discussion
- files from vendor SDKs unless their redistribution rights are clear
- PCB libraries, symbols, or footprints with unclear licensing
- images, fonts, icons, or diagrams with unclear licensing
- safety-critical changes without clear explanation and test evidence

## Pull request expectations

Good pull requests should include:

- a clear description of the change
- why the change is needed
- affected hardware revision, firmware version, or board variant
- test steps and results
- safety impact, if any
- known limitations
- updated documentation, if applicable

## Firmware changes

For firmware pull requests, include:

- target microcontroller or board revision
- build environment
- compiler/toolchain version
- test hardware
- test procedure
- expected behavior
- observed behavior
- safety limits affected by the change

## Hardware changes

For hardware pull requests, include:

- schematic diff or explanation
- PCB layout impact
- BOM impact
- thermal impact
- current, voltage, and connector rating impact
- manufacturability impact
- safety impact
- tested assembly photos or notes, if available

## Security and safety issues

Do not open a public issue for a vulnerability, overtemperature hazard, overcurrent hazard, unsafe default, or other issue that could reasonably cause damage or unsafe operation.

Report these issues according to [SECURITY.md](SECURITY.md).

## No support guarantee

Sheetak may review, modify, accept, reject, or close contributions at its discretion. Submitting a contribution does not create an obligation for Sheetak to provide support, merge the contribution, or maintain compatibility.
