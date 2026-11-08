#pragma once
#include "motor_logic.h"

// User-confirmed tendon map: A = outer Group 1, B = inner Group 2 (Figure 6A).
// User-reported: A D2 PUL+ / D3 DIR+; B A5 PUL+ / A4 DIR+. No ENA assigned.
// Installed UNO R4 WiFi core maps A5 to 19 and A4 to 18.
// Physical reference: both sides just tensioned; positive pulls on both axes.
// User selected 900 pulling pulses per axis, increased from the reported 800.
// Derived paper sequence: prepare A, then exchange A payout for B pull.
// These endpoints and the coupled path still need an observed capture/return.
// Direct UNO drive was explicitly requested despite unresolved input-current
// compatibility. Verification flags stay false; the named bench override enables
// only the unloaded test envelope. Preserve D6/D7 and D9-D13. No automatic motion.
constexpr grip::Config MOTOR_CONFIG = {
    {
        {2, 3, false, 1, 1, -2500, 2500, 2500, 500, 1000, 1000, 2000, 1000},
        {19, 18, false, 1, 1, -2500, 2500, 2500, 500, 1000, 1000, 2000, 1000},
        // Per axis: stepPin, dirPin, interfaceVerified,
        // stepActiveLevel, positiveDirLevel, minPosition, maxPosition,
        // maxJogSteps (user-selected larger manual jogs), stepsPerSecond, minPulseActiveUs, minPulseIdleUs,
        // dirSetupUs, dirHoldUs
    },
    false,     // profileVerified: physical tendon calibration is still unverified
    {900, 0},  // neutral[A,B]: prepared curled Rest / deliberate return target
    {0, 900},  // grasp[A,B]: from Rest, A -900 and B +900 together
    10000,     // jobTimeoutMs: bounded local deadline
    true,      // unloadedTestAuthorized: keep these user-requested test presets saved
    {0, 0},    // reference[A,B]: user's pre-tensioned physical zero, NOT Rest
};
