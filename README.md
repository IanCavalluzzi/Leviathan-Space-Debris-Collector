# Leviathan Space Debris Collector

Leviathan is a soft gripper prototype for irregular debris, built for UB Hacking 2026. Two NEMA 23 steppers pull tendons on six 3D-printed TPU spiral fingers. The operator positions the gripper and lifts the object by hand.

A Windows laptop runs the OpenCV preview and tracks the selected object. The Arduino UNO R4 WiFi reads the ultrasonic sensor and controls the steppers over commands from the laptop's USB serial connection. Grab pays out the outer tendons while pulling the inner tendons. **Prepare / Return Rest** reverses that move.

## Quick start

Use **Windows and Python 3.13**. Run these commands from the repository root in PowerShell.

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r code\requirements.txt
.\.venv\Scripts\python.exe code\camera_preview.py --camera 0 --port off
```

The first camera is usually index 0. For the team's USB camera, try DirectShow at 800 × 600.

```powershell
.\.venv\Scripts\python.exe code\camera_preview.py --camera 1 --backend dshow --width 800 --height 600 --port off
```

Press **G** to outline and refine an object, then click it in the live preview. **M** shows the mask, **C** clears the selection, and **Q** quits.

For optional Gemini selection, set `GEMINI_API_KEY` or save your key in the ignored file `code/gemini_api_key.txt`. Press **A** and describe the target. The app sends one snapshot and the description to Google. Press **Enter** to confirm the saved-image proposal or **C** to reject it. OpenCV handles tracking after confirmation.

## Hardware and demo status

Use the combined grip controller sketch and follow the setup guide to establish the physical reference before moving the motors. The default strokes are 900 pulses per axis. Automatic capture requires driver and travel verification plus measured camera and range settings. Physical capture, Raspberry Pi hosting, and robot-arm integration are tracked in the validation notes. [Setup and controls](docs/SETUP.md) · [Hardware](docs/HARDWARE.md) · [Validation](docs/VALIDATION.md)

## Test

Run the Python tests from `code/`.

```powershell
Push-Location code
try {
    ..\.venv\Scripts\python.exe -m unittest discover -s tests
} finally {
    Pop-Location
}
```

The tests use synthetic images and simulated devices. See the validation notes for firmware compilation and simulation. [Validation](docs/VALIDATION.md#reproduce-checks)

## Repository guide

| Path | Contents |
| --- | --- |
| [code/](code/) | Host app and Windows launchers |
| [grip_control](code/firmware/grip_control/) | UNO R4 WiFi operating sketch and serial commands |
| [sensor_test](code/firmware/sensor_test/) | Sensor diagnostic sketch |
| [tests](code/tests/) | Python tests and firmware test runners |

## Design and research

The team modeled the assembly in Fusion 360 and adapted the spiral fingers from **SpiRobs** by Zhanchi Wang, Nikolaos M. Freris, and Xi Wei. [Paper](https://arxiv.org/abs/2303.09861) · [STEP file link](Link%20To%20Step%20File) · [Software and AI credits](docs/CREDITS.md)
