# Prototype hardware

This records the team's current setup and source configuration. It is not a qualified electrical design or a universal wiring guide.

| Part | Recorded role/status |
| --- | --- |
| Arduino UNO R4 WiFi | RA4M1 runs pulse timing and sensor acquisition through normal Arduino headers |
| Two stepper motors and drivers | Two tendon groups; exact driver module/input compatibility remains unresolved |
| Hiwonder wide-angle USB camera | Selected board enumerates as icspring camera on the development laptop; device index can change |
| SR04-family ultrasonic sensor | Range from the sensor face; exact variant, ruler accuracy, and target correspondence need verification |
| Six compliant spiral fingers | Human supplies approach and lifting motion; no robot arm is integrated |

## Reported signal map

| Signal | UNO header |
| --- | --- |
| Driver A PUL+ / DIR+ | D2 / D3 |
| Driver B PUL+ / DIR+ | A5 / A4 |
| Ultrasonic ECHO / TRIG | D6 / D7 |
| Legacy IR inputs | D9-D13; retained in firmware/CSV compatibility, retired from the host UI |

A is the outer group, B the inner group; positive direction was reported to pull on both. Driver PUL-/DIR- returns were reported connected to UNO GND. No ENA connection is assigned. These are team-reported connections, not independent electrical qualification. The separate ESP32-S3 header and abandoned standalone ESP32 wiring are not this pin map.

## Limits and open work

The saved firmware has +/-2,500 commanded-position limits per axis, a 2,500-pulse jog cap, a 500-pulse/s ceiling, a ten-second job deadline, and no acceleration ramp. Default strokes are +900/+900. Counts are commanded pulses, not encoder measurements. Runtime stroke edits do not establish physical travel calibration.

`interfaceVerified` and `profileVerified` remain false. `unloadedTestAuthorized=true` preserves the team's explicit bench-test exception; it does not prove compatibility or attached-mechanism validation. Check the actual driver identity, GPIO/input-current interface, supply, current settings, tendon routing, travel, stop behavior, and release procedure before operating a new setup. Keep an accessible motor-power cutoff.

No force/grasp-confirmation sensor is selected. The ultrasonic sensor does not prove that its echo belongs to the selected camera target. Camera occlusion, job completion, contact, and secure holding are separate observations.

References: [Arduino UNO R4 WiFi](https://docs.arduino.cc/hardware/uno-r4-wifi/), [official pinout](https://docs.arduino.cc/resources/pinouts/ABX00087-full-pinout.pdf), [conditional TB6600-style module reference](https://wiki.dfrobot.com/dri0043), and [SpiRobs Figure 6](https://arxiv.org/pdf/2303.09861#page=10). A matching reference module does not identify the installed driver.
