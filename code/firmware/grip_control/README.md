# UNO R4 WiFi grip controller

This is the operating two-axis sketch. Use [setup and manual controls](../../../docs/SETUP.md) and the [hardware record](../../../docs/HARDWARE.md) before connecting or operating the mechanism.

The RA4M1 handles finite pulse generation and sensor acquisition. Pins are A D2/D3, B A5/A4, ultrasonic ECHO D6/TRIG D7. A is outer, B inner; positive pulls on both in the team's setup.

## Default coordinates

| Posture | A | B |
| --- | --- | --- |
| Pre-tensioned physical zero | 0 | 0 |
| Prepared curled Rest | 900 | 0 |
| Grasp / unconfirmed hold | 0 | 900 |

Prepare pulls A. Grab exchanges A -900/B +900 simultaneously. Deliberate return reverses the exchange to curled Rest. These are selected pulse settings, not measured object retention or a validated coupled path.

## Protocol

Use one USB serial owner at 115200 baud, with newline-terminated commands. Wait for acknowledgments and completion; do not queue motion.

| Command | Meaning |
| --- | --- |
| STATUS | State and LIMITS/PROFILE/SAFETY/CONTROLS metadata |
| REFERENCE_NEUTRAL | Idle/disarmed declaration of physical zero; no movement |
| REFERENCE_REST | Idle/disarmed declaration of already-retracted Rest; no movement |
| SET_STROKES A B | Idle/disarmed signed stroke settings; no motion; nonzero within +/-2,500; reset restores defaults |
| ARM | Permission for one finite job; expires after ten seconds |
| NEUTRAL / RELEASE | Prepare/return to configured curled Rest; ARM required |
| GRAB / CAPTURE | Coordinated move from Rest to Grasp; ARM required; completes HOLD_UNCONFIRMED |
| JOG A N / JOG B N | Finite signed commissioning jog; ARM required |
| GUARD | Explicit opt-in communication lease, 1,500 ms |
| STOP / DISARM | Stop new steps and disarm; no power cutoff or automatic unwind |

Boot is disarmed/unreferenced. Interrupted motion invalidates the reference. Guard expiration stops active motion; an existing completed hold does not unwind. STATUS renews an active guard, but does not keep ARM permission alive.

Current host controls require `CONTROLS 1` and a compatible live PROFILE. Default metadata is `# motor PROFILE 0 0 900 0 0 900`. Runtime edits change that profile. Source availability does not establish the installed board version.

## Compile and tests

From the repository root, with Arduino CLI/core installed:

```powershell
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi code\firmware\grip_control
.\code\tests\firmware\check_grip_logic.ps1 -ArduinoCli (Get-Command arduino-cli).Source
```

These compile without uploading. The full Python test command and validation limits are in [validation](../../../docs/VALIDATION.md). Physical verification flags remain false; the saved unloaded bench exception does not qualify the electrical interface or mechanism.
