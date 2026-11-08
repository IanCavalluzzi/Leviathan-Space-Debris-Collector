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
5. **Release:** A deliberate Return to neutral command restores the referenced starting configuration.

The control software includes finite movement limits, command acknowledgments, Stop handling, and an optional communication watchdog. Manual motor controls support mechanism setup and testing.

## Project status

Leviathan is a hackathon prototype undergoing mechanical commissioning and repeatable capture-and-release testing. Automatic capture requires validated tendon travel and camera/sensor calibration. Robot-arm integration remains a future development goal.

## Design and research

The mechanical assembly was modeled in **Autodesk Fusion 360**. The team adapted published spiral-finger geometry and developed the surrounding assembly, actuation integration, and vision/control software.

The finger design draws on **SpiRobs** by Zhanchi Wang, Nikolaos M. Freris, and Xi Wei:

[SpiRobs: Logarithmic Spiral-shaped Robots for Versatile Grasping Across Scales](https://arxiv.org/abs/2303.09861)
