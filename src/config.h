#pragma once

#include <cstdint>

// Central place for board-specific constants. Pins are for the AI-Thinker
// ESP32-CAM. The camera + PSRAM consume most GPIOs; the pins below are the
// SD-card pins, which are free as long as no microSD card is used. VERIFY
// against your module before wiring. Do NOT use GPIO0 (camera XCLK + boot
// strap) or GPIO4 (onboard flash LED).
namespace config {

// --- I2C (MPU6050) ---
constexpr int kI2cSda = 14;
constexpr int kI2cScl = 15;
constexpr uint8_t kMpuAddress = 0x68;  // AD0 low

// --- ESC signal pins ---
constexpr int kEsc1Pin = 13;
constexpr int kEsc2Pin = 12;

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
// The control loop is pinned to this core so the camera + web server (on the
// other core, milestone M4) can never starve it. Arduino's loopTask runs on
// core 1; we keep control there and leave core 0 for WiFi/camera/web.
constexpr int kControlCore = 1;

// --- PID (starting gains — tune live from the web UI) ---
constexpr float kKp = 0.010f;  // throttle fraction per degree of error
constexpr float kKi = 0.000f;
constexpr float kKd = 0.0008f;
// The PID output is a throttle *differential* around the base; cap it so one
// motor can't be driven far past the other.
constexpr float kPidOutLimit = 0.30f;

// --- Mixing & setpoint ---
constexpr float kBaseThrottle = 0.30f;  // common hover throttle (0..1)
constexpr float kSetpointDeg = 0.0f;    // target tilt: level

// --- Safety ---
// Past this tilt the beam is considered lost; cut both motors and disarm until
// re-enabled.
constexpr float kTiltLimitDeg = 45.0f;

// --- WiFi access point (the board hosts its own network; connect directly) ---
constexpr char kApSsid[] = "libra";
constexpr char kApPassword[] = "balancebot";  // >= 8 chars, or "" for open

}  // namespace config
