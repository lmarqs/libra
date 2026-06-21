#pragma once

#include <cstdint>

// Central place for board-specific constants. Pins are for the ESP32-C3 Super
// Mini. VERIFY against your board's silkscreen before wiring. GPIO2 is a
// strapping pin used here for I2C SDA — fine because the bus idles high (the
// level GPIO2 wants at reset), but keep the other strapping pins (GPIO8/GPIO9)
// and the native-USB pins (GPIO18/GPIO19) free.
namespace config {

// --- I2C (MPU6050) ---
#ifndef LIBRA_I2C_SDA
#define LIBRA_I2C_SDA 2
#endif
#ifndef LIBRA_I2C_SCL
#define LIBRA_I2C_SCL 3
#endif
constexpr int kI2cSda = LIBRA_I2C_SDA;
constexpr int kI2cScl = LIBRA_I2C_SCL;
constexpr uint8_t kMpuAddress = 0x68;  // AD0 low

// --- ESC signal pins ---
#ifndef LIBRA_ESC_PIN1
#define LIBRA_ESC_PIN1 0
#endif
#ifndef LIBRA_ESC_PIN2
#define LIBRA_ESC_PIN2 1
#endif
constexpr int kEsc1Pin = LIBRA_ESC_PIN1;
constexpr int kEsc2Pin = LIBRA_ESC_PIN2;

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
// re-enabled. Overridable from .env (see platformio.ini).
#ifndef LIBRA_TILT_LIMIT_DEG
#define LIBRA_TILT_LIMIT_DEG 45.0f
#endif
constexpr float kTiltLimitDeg = LIBRA_TILT_LIMIT_DEG;

// --- WiFi access point (web UI) ---
// SoftAP credentials for the tuning web UI. Overridable from .env (see
// platformio.ini). WPA2 requires the password to be 8-63 chars; a shorter one
// makes WiFi.softAP() fail.
#ifndef LIBRA_AP_SSID
#define LIBRA_AP_SSID "libra"
#endif
#ifndef LIBRA_AP_PASSWORD
#define LIBRA_AP_PASSWORD "balancebot"
#endif
constexpr char kApSsid[] = LIBRA_AP_SSID;
constexpr char kApPassword[] = LIBRA_AP_PASSWORD;

}  // namespace config
