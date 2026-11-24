# Security and Safety Policy

CryoSnap includes firmware, electronics, power control, temperature sensing, and thermal-control behavior. Security and safety issues can include software vulnerabilities as well as hardware or control-loop behavior that could cause unsafe operation.

## Reporting a vulnerability or safety issue

Please report security or safety issues privately.

Contact:

```text
info@sheetak.com
```

Include as much detail as possible:

- affected hardware revision
- affected firmware version or commit
- affected configuration
- test setup
- power supply used
- TEC/module/load used
- steps to reproduce
- expected behavior
- observed behavior
- logs, serial output, screenshots, or data files
- whether the issue could cause overheating, overcurrent, unexpected heating/cooling, unsafe output, data exposure, or device damage

## Please do not publicly disclose first

Do not open a public GitHub issue for a vulnerability or safety issue until Sheetak has had a reasonable opportunity to investigate and respond.

Examples of issues to report privately:

- bypass of firmware safety limits
- unsafe default parameters
- overtemperature behavior
- overcurrent behavior
- uncontrolled heating or cooling
- watchdog or fail-safe failure
- bootloader or update vulnerability
- command interface vulnerability
- unsafe sensor failure behavior
- incorrect polarity behavior
- PCB issue that could cause shorts, overheating, or unsafe current
- documentation error that could lead to unsafe operation

## Supported versions

Unless Sheetak states otherwise, only the latest public firmware release and latest public hardware revision are supported for security and safety review.

| Component | Supported |
| --- | --- |
| Latest public firmware release | Yes |
| Latest public hardware revision | Yes |
| Older firmware releases | Best effort |
| Older hardware revisions | Best effort |
| Modified forks | Best effort only |
| Commercial deployments | Governed by separate written agreement |

## Coordinated disclosure

Sheetak will attempt to:

- acknowledge reports within a reasonable time
- investigate reproducible reports
- request additional information when needed
- prioritize issues based on severity and safety impact
- publish fixes or mitigations when appropriate
- credit reporters when appropriate and requested

## Research safe harbor

Sheetak does not intend to pursue legal action for good-faith security or safety research that:

- avoids harm to people, property, systems, and data
- avoids privacy violations
- avoids disruption of Sheetak or third-party systems
- uses only devices, accounts, and systems you own or are authorized to test
- does not use findings for extortion, coercion, or unauthorized access
- is reported promptly and privately
- complies with applicable law

This safe harbor does not authorize commercial use, sale, distribution, or product integration of CryoSnap-derived work.

## Safety disclaimer

CryoSnap is provided as-is, without warranty. You are responsible for validating electrical, thermal, firmware, mechanical, and regulatory safety for your use case.

Do not use CryoSnap in life-critical, safety-critical, medical, aerospace, automotive, or other high-risk applications without a separate written agreement with Sheetak and appropriate engineering validation.
