#include "EscPair.h"

#include <Arduino.h>
#include <ESP32Servo.h>

namespace {
// ESP32Servo needs its LEDC timers reserved once before any attach(). Two ESCs
// need two timers; reserve a pair up front.
void ensureTimersAllocated() {
  static bool done = false;
  if (done) return;
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  done = true;
}
}  // namespace

EscPair::EscPair(int pin1, int pin2, int min_us, int max_us)
    : _pin1(pin1), _pin2(pin2), _min_us(min_us), _max_us(max_us) {}

int EscPair::throttleToUs(float t) const {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return _min_us + static_cast<int>(t * (_max_us - _min_us) + 0.5f);
}

void EscPair::begin(uint32_t arm_ms) {
  ensureTimersAllocated();
  auto* e1 = new Servo();
  auto* e2 = new Servo();
  e1->setPeriodHertz(50);
  e2->setPeriodHertz(50);
  e1->attach(_pin1, _min_us, _max_us);
  e2->attach(_pin2, _min_us, _max_us);
  _esc1 = e1;
  _esc2 = e2;
  // Arm: hold minimum throttle so the ESCs accept the signal.
  disarm();
  delay(arm_ms);
}

void EscPair::writeThrottle(float t1, float t2) {
  if (_esc1 == nullptr || _esc2 == nullptr) return;
  static_cast<Servo*>(_esc1)->writeMicroseconds(throttleToUs(t1));
  static_cast<Servo*>(_esc2)->writeMicroseconds(throttleToUs(t2));
}

void EscPair::disarm() { writeThrottle(0.0f, 0.0f); }
