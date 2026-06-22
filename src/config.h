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
// The beam pivots about the IMU's Z axis: at rest gravity rests on +X, and
// rotating about Z swings it into Y, so gravity in the X/Y plane gives the tilt
// and the Z gyro axis (negated) gives its rate. Flip this if bring-up shows the
// angle responding to the wrong tilt or with the wrong sign (mounting-dependent).
// (Used by main.cpp when it maps an ImuSample to (accel_angle, rate).)
//
// Zero-offset (deg) subtracted from the measured tilt so a physically level beam
// reads 0. Mounting-specific — measure the resting angle and set it in .env.
// Defaults to 0 (no trim).
#ifndef LIBRA_ANGLE_OFFSET_DEG
#define LIBRA_ANGLE_OFFSET_DEG 0.0f
#endif
constexpr float kAngleOffsetDeg = LIBRA_ANGLE_OFFSET_DEG;

// --- Complementary filter ---
// Gyro weight; the remaining (1 - alpha) trusts the accelerometer.
constexpr float kFilterAlpha = 0.98f;

// --- Control loop ---
constexpr uint32_t kLoopHz = 200;
// The control task paces itself with xTaskDelayUntil at the FreeRTOS tick (1 ms on
// arduino-esp32, configTICK_RATE_HZ == 1000, on both archs). The period is
// pdMS_TO_TICKS(1000 / kLoopHz); if kLoopHz doesn't divide 1000 the rate is silently
// quantized to whole ticks. Keep it an even divisor so 200 Hz stays exactly 200 Hz.
static_assert(1000UL % kLoopHz == 0, "kLoopHz must evenly divide 1000 (1 ms FreeRTOS tick)");

// --- PID (starting gains — tune live over serial) ---
constexpr float kKp = 0.002f;  // throttle fraction per degree of error (full correction ~2.5 deg)
constexpr float kKi = 0.000f;
constexpr float kKd = 0.0008f;

// --- Mixing, throttle band & setpoint (safety) ---
constexpr float kSetpointDeg = 0.0f;  // target tilt: level

// The balancer keeps both propellers inside a throttle band [kMinThrottle, kMaxThrottle]
// (per-motor fraction, 0..1) — the single source of motor commands, enforced at the mixer.
// Both ends are the source of truth; the hover point and the control authority are DERIVED:
//   - hover (base) throttle = the band midpoint    — where both motors idle at zero error,
//   - PID authority (±limit) = half the band width  — so a full correction drives one motor
//     up to kMaxThrottle while the other drops to kMinThrottle (and vice versa).
// Thus base ± kPidOutLimit spans exactly [kMinThrottle, kMaxThrottle]: while armed the
// controller can command any per-motor throttle across the whole band and nothing outside it.
//
// Set from .env via LIBRA_THROTTLE_MIN / LIBRA_THROTTLE_MAX (mise injects them; see
// platformio.ini). Fractions map onto the ESC's ~1000-2000 us range (0.0 -> 1000 us idle,
// 1.0 -> 2000 us full). Defaults size the band to the reference 1404 4600KV/3S rig:
// [0.080, 0.090] (~1080-1090 us) -> hover 0.085, authority ±0.005; re-find the band for your
// motor (docs/testing.md). Widening the band raises both thrust and control authority — a
// change the safety rules guard (see CLAUDE.md). Props off / beam clamped while you tune.
#ifndef LIBRA_THROTTLE_MIN
#define LIBRA_THROTTLE_MIN 0.080f
#endif
#ifndef LIBRA_THROTTLE_MAX
#define LIBRA_THROTTLE_MAX 0.090f
#endif
constexpr float kMinThrottle = LIBRA_THROTTLE_MIN;
constexpr float kMaxThrottle = LIBRA_THROTTLE_MAX;
static_assert(kMinThrottle >= 0.0f, "LIBRA_THROTTLE_MIN must be >= 0");
static_assert(kMaxThrottle <= 1.0f, "LIBRA_THROTTLE_MAX must be <= 1 (full throttle)");
static_assert(kMinThrottle <= kMaxThrottle, "LIBRA_THROTTLE_MIN must not exceed LIBRA_THROTTLE_MAX");

// Hover at the band midpoint; full PID authority spans half the band (derivation above).
constexpr float kBaseThrottle = (kMinThrottle + kMaxThrottle) * 0.5f;
constexpr float kPidOutLimit = (kMaxThrottle - kMinThrottle) * 0.5f;

// --- Safety ---
// Past this tilt the beam is considered lost; cut both motors and disarm until
// re-enabled. Overridable from .env (see platformio.ini).
#ifndef LIBRA_TILT_LIMIT_DEG
#define LIBRA_TILT_LIMIT_DEG 45.0f
#endif
constexpr float kTiltLimitDeg = LIBRA_TILT_LIMIT_DEG;

// --- WiFi access point (web UI) ---
// SSID for the tuning web UI's SoftAP. The UI can arm/disarm (the software master-enable;
// boots disarmed), so on an OPEN AP any client could arm — set a WPA2 password via
// LIBRA_AP_PASS when props are on or others are in range. Defaults to open. Overridable
// from .env (see platformio.ini).
#ifndef LIBRA_AP_SSID
#define LIBRA_AP_SSID "libra"
#endif
constexpr char kApSsid[] = LIBRA_AP_SSID;

// Optional WPA2 password for the SoftAP. Empty (default) = open AP. WPA2 needs >= 8
// chars; a shorter non-empty value is rejected at boot and the AP stays open.
#ifndef LIBRA_AP_PASS
#define LIBRA_AP_PASS ""
#endif
constexpr char kApPass[] = LIBRA_AP_PASS;

// --- Bench / calibration gate ---
// Compile-time switch for the props-off bench commands (set motors_enabled / set motors_speed
// / run / x — raw per-motor drive for ESC throttle-range calibration and spin-start hunting).
// They bypass the kMaxThrottle cap and the tilt failsafe, so they are OFF by default; set
// LIBRA_BENCH_ENABLED=1 in .env for bench work, and back to 0 for a props-on build. Used as a
// preprocessor gate (#if) in main.cpp, so there is no constexpr binding.
#ifndef LIBRA_BENCH_ENABLED
#define LIBRA_BENCH_ENABLED 0
#endif

}  // namespace config
