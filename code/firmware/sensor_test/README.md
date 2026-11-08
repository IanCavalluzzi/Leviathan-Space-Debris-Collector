# Sensor-only diagnostic sketch

This is a separate diagnostic fallback, not the operating grip controller. Uploading it replaces motor-command firmware. Use [grip_control](../grip_control/) for integrated operation.

The sketch uses TRIG D7/ECHO D6 and does not configure or drive motor signals D2/D3. Legacy IR inputs remain D9-D13; the host interface no longer uses them. The SR04-family conversion assumes a standard HC-SR04; identify the actual module and interface before applying it to another setup. See [hardware assumptions](../../../docs/HARDWARE.md).

Select Arduino UNO R4 WiFi in Arduino IDE and Verify. Keep motor power off during a deliberate upload. Close other serial owners and select the actual board port. Open Serial Monitor at 115200 baud, then close it before using the host application.

CSV rows arrive about every 200 ms:

```text
ms,echo_us,ultrasonic_status,distance_cm,ir1_d9,ir2_d10,ir3_d11,ir4_d12,ir5_d13
```

Invalid range is `NA`, not zero. Comment diagnostics distinguish NO_RISE, NO_FALL, HIGH_BEFORE_TRIGGER, and PULSE. A valid echo does not identify its target or establish range accuracy or capture readiness. IR values are raw input levels, not calibrated distance or grip feedback.
