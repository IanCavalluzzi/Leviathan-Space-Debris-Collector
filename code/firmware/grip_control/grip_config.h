#pragma once
#include "motor_logic.h"

// A drives the outer tendons. B drives the inner tendons. Positive pulls on both.
// UNO R4 WiFi maps A5/A4 to 19/18. See docs/HARDWARE.md for the bench setup.
constexpr grip::Config MOTOR_CONFIG = {
    {
        // stepPin, dirPin, interfaceVerified,
        // stepActiveLevel, positiveDirLevel, minPosition, maxPosition,
        // maxJogSteps, stepsPerSecond, minPulseActiveUs, minPulseIdleUs, dirSetupUs, dirHoldUs
        {2, 3, false, 1, 1, -2500, 2500, 2500, 500, 1000, 1000, 2000, 1000},
        {19, 18, false, 1, 1, -2500, 2500, 2500, 500, 1000, 1000, 2000, 1000},
    },
    false,     // profileVerified
    {900, 0},  // Rest
    {0, 900},  // Grasp
    10000,     // jobTimeoutMs
    true,      // unloadedTestAuthorized
    {0, 0},    // Physical reference with both tendons just tensioned
};
