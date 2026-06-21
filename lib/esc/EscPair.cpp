#include "EscPair.h"

#include <Arduino.h>

namespace {
// ESC PWM straight off LEDC. 200 Hz matches the 200 Hz control loop (every step lands a fresh
// frame, no stale 50 Hz holdover) and stays well inside the Afro/SimonK/BLHeli analog-PWM
// range. 14 bits is the LEDC resolution ceiling on the ESP32-C3 (the WROOM allows up to 20);
// at 200 Hz that is 1/(200·2^14) s ≈ 0.305 µs per step — ~3× finer than integer-µs
// writeMicroseconds, and identical on both archs. Channels 0/1 share LEDC timer 0 (same
// freq+resolution, which is required for shared timers); nothing else here uses LEDC.
constexpr uint32_t kRefreshHz = 200;
constexpr uint8_t kResBits = 14;
constexpr int kCh1 = 0;
constexpr int kCh2 = 1;
constexpr float kPeriodUs = 1000000.0f / kRefreshHz;  // 5000 µs at 200 Hz
constexpr uint32_t kDutyLevels = 1UL << kResBits;     // 16384
}  // namespace

EscPair::EscPair(int pin1, int pin2, int min_us, int max_us)
    : _pin1(pin1), _pin2(pin2), _min_us(min_us), _max_us(max_us) {}

void EscPair::writeChannel(int channel, float t) const {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  const float us = _min_us + t * (_max_us - _min_us);  // sub-µs precise
  uint32_t duty = static_cast<uint32_t>(us / kPeriodUs * kDutyLevels + 0.5f);
  if (duty >= kDutyLevels) duty = kDutyLevels - 1;
  ledcWrite(channel, duty);
}

void EscPair::begin(uint32_t arm_ms) {
  ledcSetup(kCh1, kRefreshHz, kResBits);
  ledcSetup(kCh2, kRefreshHz, kResBits);
  ledcAttachPin(_pin1, kCh1);
  ledcAttachPin(_pin2, kCh2);
  _ready = true;
  // Arm: hold minimum throttle so the ESCs accept the signal.
  disarm();
  delay(arm_ms);
}

void EscPair::writeThrottle(float t1, float t2) {
  if (!_ready) return;
  writeChannel(kCh1, t1);
  writeChannel(kCh2, t2);
}

void EscPair::disarm() { writeThrottle(0.0f, 0.0f); }
