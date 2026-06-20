#pragma once

#include <cstdint>

// Central place for board-specific constants. Pins are for the ESP32-C3 Super
// Mini. VERIFY against your board's silkscreen before wiring. Avoid the
// strapping pins (GPIO2, GPIO8, GPIO9) and the native-USB pins (GPIO18/19).
namespace config {

// --- I2C (MPU6050) ---
constexpr int kI2cSda = 5;
constexpr int kI2cScl = 6;
constexpr uint8_t kMpuAddress = 0x68;  // AD0 low

// --- ESC signal pins ---
constexpr int kEsc1Pin = 3;
constexpr int kEsc2Pin = 4;

// --- Tilt geometry ---
// The beam pivots about the IMU's X axis, so gravity in the Y/Z plane gives the
// tilt and the X gyro axis gives its rate. Flip this if bring-up shows the angle
// responding to the wrong tilt or with the wrong sign (mounting-dependent).
// (Used by main.cpp when it maps an ImuSample to (accel_angle, rate).)

// --- Complementary filter ---
// Gyro weight; the remaining (1 - alpha) trusts the accelerometer.
constexpr float kFilterAlpha = 0.98f;

// --- Control loop ---
constexpr uint32_t kLoopHz = 200;
constexpr uint32_t kLoopPeriodUs = 1000000UL / kLoopHz;

// --- PID (starting gains — tune live over serial) ---
constexpr float kKp = 0.010f;  // throttle fraction per degree of error
constexpr float kKi = 0.000f;
constexpr float kKd = 0.0008f;
// The PID output is a throttle *differential* around the base; cap it so one
// motor can't be driven far past the other. Kept within the 5% ceiling below.
constexpr float kPidOutLimit = 0.02f;

// --- Mixing & setpoint ---
constexpr float kBaseThrottle = 0.03f;  // common hover throttle (0..1)
constexpr float kSetpointDeg = 0.0f;    // target tilt: level
// Hard ceiling on per-motor throttle (safety). No propeller is ever commanded
// above this — enforced at the mixer, the single source of motor commands.
// base + kPidOutLimit must stay <= this, so the cap never starves control
// authority: here each motor lives in [0.01, 0.05].
//
// Set from .env via the LIBRA_THROTTLE_MAX build flag (mise injects it; see
// platformio.ini). Defaults to 0.05 (5%) if the flag isn't passed.
#ifndef LIBRA_THROTTLE_MAX
#define LIBRA_THROTTLE_MAX 0.05f
#endif
constexpr float kMaxThrottle = LIBRA_THROTTLE_MAX;

// --- Safety ---
// Past this tilt the beam is considered lost; cut both motors and disarm until
// re-enabled.
constexpr float kTiltLimitDeg = 45.0f;

}  // namespace config
