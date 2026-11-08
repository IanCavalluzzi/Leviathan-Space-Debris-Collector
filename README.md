# Leviathan Space Debris Collector

**A vision-guided soft robotic gripper for irregular debris.**

Developed for **UB Hacking 2026**, Leviathan explores a modular end effector inspired by the challenge of capturing irregular space debris. The prototype combines six flexible, logarithmic spiral fingers with camera-based target selection, local tracking, ultrasonic ranging, and coordinated motor control.

## The concept

Leviathan is designed to wrap flexible fingers around an object through tendon actuation. Two motor channels coordinate the finger groups, with the longer-term goal of integrating the assembly onto a robotic arm.

The current demonstration uses a human operator to position and lift the gripper.

## System architecture

| Component | Role |
| --- | --- |
| Six 3D-printed TPU spiral fingers | Compliant contact around the target |
| Two NEMA 23 stepper motors | Tendon actuation |
| Arduino UNO R4 WiFi | Local pulse generation, bounded movement, and sensor acquisition |
| USB camera | Scene input for target selection and tracking |
| Ultrasonic distance sensor | Range information for the capture window |
| Python, OpenCV, and Gemini API | Operator interface, requested-object selection, and local visual tracking |

Camera processing and AI requests run on the host computer. Motor timing runs locally on the Arduino, connected over USB serial.

## Demonstration workflow

1. **Select:** Describe the intended object. Gemini proposes a target from a camera snapshot, and the operator confirms the selection.
2. **Track:** OpenCV follows the confirmed target locally.
3. **Approach:** The operator positions the gripper using the camera view and available guidance.
4. **Capture:** The controller executes a coordinated Grab. The software also includes an explicitly enabled, one-shot capture mode gated by calibrated alignment, range, and controller readiness.
5. **Release:** A deliberate Prepare / Return Rest command restores the curled Rest configuration, which is distinct from the pre-tensioned physical zero.

The control software includes finite movement limits, command acknowledgments, Stop handling, and an optional communication watchdog. Manual motor controls support mechanism setup and testing.

## Project status

Leviathan is a hackathon prototype undergoing mechanical commissioning and repeatable capture-and-release testing. Automatic capture requires validated tendon travel and camera/sensor calibration. Robot-arm integration remains a future development goal.

The published source includes the latest 900-pulse defaults, signed runtime stroke settings, manual arm toggle, and explicit no-motion reference declarations. This revision has software validation; its installation and physical capture behavior still need verification. Motion completion does not confirm that an object is securely held. Raspberry Pi hosting and space operation are not demonstrated capabilities.

## Quick start

The primary development platform is **Windows with Python 3.13**. From the cloned repository root in PowerShell:

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r code\requirements.txt
.\.venv\Scripts\python.exe code\camera_preview.py --camera 0 --port off
```

This opens a camera-only preview without controller communication. Camera numbers depend on the devices connected to your computer. For the team's external Windows camera, try:

```powershell
.\.venv\Scripts\python.exe code\camera_preview.py --camera 1 --backend dshow --width 800 --height 600 --port off
```

Press **G** to outline an object, refine its foreground, then click to select it. **M** shows the mask; **C** or right-click clears selection; **Q** exits. Local selection works without a Gemini key.

For Gemini selection, create the ignored file `code/gemini_api_key.txt` containing only your own API key, or set `GEMINI_API_KEY` in the launching process. Press **A**, enter a target description, and inspect the saved-image proposal. **Enter** confirms it; **C** rejects it. Each explicit request sends one snapshot and the request text to Google's API. Normal startup sends no Gemini request. Availability and quota depend on your account; `GEMINI_MODEL` can override the source default.

See [setup and controls](docs/SETUP.md) before connecting a controller or using the manual motor launcher. Firmware source in this repository is not evidence that the connected board has that version installed.

## Test

From the repository root, run the hardware-free tests from `code/`:

```powershell
Push-Location code
try {
    ..\.venv\Scripts\python.exe -m unittest discover -s tests
} finally {
    Pop-Location
}
```

The tests use synthetic images and mocked camera, serial, and API interfaces. They do not operate hardware or establish capture reliability. See [validation and limitations](docs/VALIDATION.md) for the publication checks.

## Repository guide

| Path | Contents |
| --- | --- |
| [code/](code/) | Camera interface, local tracking, Gemini selection, serial communication, and manual controls |
| [code/firmware/grip_control/](code/firmware/grip_control/) | Operating two-axis UNO R4 WiFi firmware |
| [code/firmware/sensor_test/](code/firmware/sensor_test/) | Separate sensor-only diagnostic sketch |
| [code/tests/](code/tests/) | Python tests and firmware test fixtures |
| [docs/SETUP.md](docs/SETUP.md) | Installation, camera selection, controller setup, and recovery |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Current pin map and unresolved hardware assumptions |
| [docs/CREDITS.md](docs/CREDITS.md) | Research, software, and AI assistance credits |

Private credentials, virtual environments, generated build files, and internal working archives are excluded.

## Design and research

The mechanical assembly was modeled in **Autodesk Fusion 360**. The team adapted published spiral-finger geometry and developed the surrounding assembly, actuation integration, and vision/control software.

The finger design draws on **SpiRobs** by Zhanchi Wang, Nikolaos M. Freris, and Xi Wei:

[SpiRobs: Logarithmic Spiral-shaped Robots for Versatile Grasping Across Scales](https://arxiv.org/abs/2303.09861)
