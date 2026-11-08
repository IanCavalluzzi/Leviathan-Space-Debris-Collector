# Setup and operation

## Windows environment

Use Python 3.13 and the commands in the [README](../README.md#quick-start). The pinned requirements are NumPy, desktop OpenCV, and pySerial. The project uses Python's standard HTTPS/JSON libraries for Gemini; no Gemini SDK is required.

After creating `.venv` at the repository root, `code/Start Camera.cmd` launches the camera application and accepts its command-line arguments. `code/Open Environment.cmd` opens PowerShell in `code/` with that environment activated. `code/Start Manual Motors.cmd` opens the camera-independent motor controls.

Useful commands from the repository root:

```powershell
.\.venv\Scripts\python.exe code\camera_preview.py --help
.\.venv\Scripts\python.exe code\camera_preview.py --list-ports
.\.venv\Scripts\python.exe code\camera_preview.py --camera 0 --port off
.\.venv\Scripts\python.exe code\manual_motors.py --port off
```

`--list-ports` enumerates serial identities without opening a camera or serial connection. `--port off` disables serial communication. Normal controller use defaults to discovery of one UNO R4 WiFi; an explicit `--port COMxx` selects the identified board. Avoid treating a remembered COM number or camera index as a permanent hardware identity.

## Vision controls

Use a steady camera, diffuse lighting, and a contrasting background. Keep the target fully visible and separated from other objects.

1. Press **G**, draw a rectangle around the target, and press Enter.
2. In foreground refinement, left-drag to keep missing target pixels and right-drag to remove unwanted pixels. Space recomputes; Enter accepts.
3. Click the target in the live preview. **M** toggles the mask. Right-click or **C** clears selection.
4. Move slowly. A lost or ambiguous target requires deliberate reselection. Re-outline after a large appearance or scale change.

**A** requests Gemini selection after you enter a description. Inspect and confirm the saved snapshot before local tracking begins. The displayed whole-object box can be estimated from a visible local anchor; it is not a complete measured silhouette or evidence of reach. **G/click is manual selection**, not semantic recognition. **Q/Escape** exits the normal camera preview; dialog keys depend on the active dialog.

## Controller firmware

The operating sketch is [grip_control.ino](../code/firmware/grip_control/grip_control.ino). Select **Arduino UNO R4 WiFi** in Arduino IDE and install the Arduino UNO R4 Boards package if needed. The development checks used core `arduino:renesas_uno` 1.6.0. No extra sketch libraries are required.

Compilation alone does not upload firmware:

```powershell
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi code\firmware\grip_control
```

Before a deliberate upload, turn motor power off and close the camera, Manual Motors, and Serial Monitor so the port is released. Select the actual board port in Arduino IDE. After upload, close Serial Monitor and restart the host. One program owns USB serial at a time.

The latest controls require `CONTROLS 1` metadata and a compatible live `PROFILE`. The default profile is `0 0 900 0 0 900`: physical zero, Rest, and Grasp A/B coordinates. Host controls reject missing/incompatible metadata. Installing Python source does not update the board.

## Manual sequence

Use the team's verified physical reference and [hardware assumptions](HARDWARE.md). The supplied 900-pulse strokes are selected demonstration settings, not a universal calibration.

1. At the marked pre-tensioned physical zero, confirm **Set physical neutral**. If the mechanism is already at its known retracted Rest posture, confirm **Set retracted** instead. Both actions assign coordinates without movement. They are declarations, not homing.
2. In the camera interface, switch **Manual arm ON**. This enables deliberate movement clicks without moving or sending ARM itself. Each movement handles its own acknowledged ARM. Manual Motors handles this per click without the camera toggle.
3. **Prepare / Return Rest** moves to Rest, default `{900,0}`. The human approaches the target.
4. **Manual Grab** exchanges A payout for B pull, default endpoint `{0,900}`. Completion reports `HOLD_UNCONFIRMED`; visually assess capture before a supported lift.
5. Lower the object and deliberately **Prepare / Return Rest**. This returns to curled Rest, not physical `{0,0}`.

**A pulses/B pulses** and **Apply pulses** change signed runtime strokes without moving or redefining the current position. Nonzero values must be within +/-2,500. Rest becomes `{A,0}` and Grasp `{0,B}`. A controller reset restores +900/+900 and requires reference declaration again. A/B +/-100 buttons are actual finite jogs. Custom jog counts remain within the installed per-job and cumulative position limits; typing alone does not move a motor.

Stop, USB/session loss, or lost reference disables the camera manual toggle. Stop requests a software stop; it is not a motor-power cutoff. Interrupted motion invalidates the reference. Restore the known physical posture before declaring a reference again. A completed hold never automatically unwinds because the object obscures the camera.

## Automatic capture and recovery

Auto starts off. It requires measured camera/range calibration, explicit echo-target association, validated dwell settings, compatible controller metadata, and physical verification flags. The published hardware flags remain false, so this is not a commissioned automatic-capture demonstration. Manual movement does not require a vision target or auto calibration.

If the camera fails, close other camera applications and check its device index/backend. If USB fails, close all other serial owners, enumerate ports again, and reconnect the identified controller. Restart after source updates. Never interpret a completed software job, plausible echo, or tracking box as confirmation of secure capture.
