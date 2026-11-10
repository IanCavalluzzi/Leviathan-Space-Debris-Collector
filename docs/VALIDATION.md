# Validation and limitations

The first public release was checked on **8 November 2026** on Windows with Python 3.13.13.

## Local software checks

| Check | Result |
| --- | --- |
| Install pinned requirements in a new isolated environment | Passed: NumPy 2.5.4, OpenCV 5.0.0.93, pySerial 3.5 |
| `python -m pip check` | Passed; no broken requirements |
| Python suite from `code/` | 245 tests passed |
| Production `grip_control` compile for UNO R4 WiFi | Passed; 60,032 program bytes, 7,284 bytes of globals |
| Motor-logic test sketch | Compile-time assertions passed with the publication headers |
| Actual-sketch sensor/controller simulation | 98 checks passed; zero failures |
| Production CSV formatting through Python parser | Eight fixtures accepted; interleaved motor comments ignored |

Arduino core 1.6.0 emitted warnings during compilation. The production sketch was not uploaded as part of publication. The C++ runtime checks simulate the sketch on a host using test Arduino/timer interfaces. Python camera, serial, and cloud paths use mocks and synthetic inputs.

## Reproduce checks

The [README](../README.md#test) gives the Python test command. For compile-time motor checks, use the supplied script with an installed Arduino CLI:

```powershell
.\code\tests\firmware\check_grip_logic.ps1 -ArduinoCli (Get-Command arduino-cli).Source
```

The sensor simulation uses Windows Clang to compile x86-64 Linux objects and WSL to link and run them with `ld` and libc. The script finds `clang.exe` on PATH by default. Pass `-Compiler` for a separate installation and `-WslDistribution` if your distribution differs from Ubuntu 22.04.

```powershell
.\code\tests\firmware\check_sensor_logic.ps1 -Compiler 'C:\path\to\clang.exe' -WslDistribution 'Ubuntu-22.04'
```

Create the root `.venv` first; the runner uses it for Python parser checks. These advanced tooling requirements are separate from running the application or Python tests.

## Physical evidence and remaining work

Earlier development established camera capture and USB communication, with user-reported local tracking, a Gemini handoff, and operation of a preceding manual-control build. Those observations do not validate every later revision or every scene.

The published 900-pulse defaults, signed runtime stroke settings, and latest controls have not been established as installed or physically tested by this publication check. Repaired Gemini handoff has offline regression evidence but still needs a live movement check in the final setup. Repeatable capture/lift/release, physical stop response, driver input compatibility, tendon calibration, and selected-target/range correspondence remain unverified.

Automatic capture starts off and remains blocked by the current physical-verification flags and absent measured configuration. A completed command is not evidence of contact or secure holding. Human motion supplies approach/lifting. Robot-arm integration, Raspberry Pi runtime, and space operation are future work.

Publication checks did not open a camera or serial connection, call Gemini, flash a controller, or move motors. Internal logs, saved credentials, private document copies, and generated packages are excluded from this repository. The GitHub workflow runs the Python checks on pushes and pull requests; hosted results are separate from these local checks.
