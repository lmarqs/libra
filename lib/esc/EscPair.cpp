#include "EscPair.h"

#include <Arduino.h>
#include <ESP32Servo.h>

namespace {
// ESP32Servo needs its LEDC timers reserved once before any attach(). We claim
// only timers 2 and 3 (two ESCs need two timers) and deliberately leave timers
// 0/1 free for the camera's XCLK — initialize the camera before the ESCs so it
// takes timer 0. Grabbing all four here would break the camera.
void ensureTimersAllocated() {
  static bool done = false;
  if (done) return;
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  done = true;
}
}  // namespace

EscPair::EscPair(int pin1, int pin2, int min_us, int max_us)
    : pin1_(pin1), pin2_(pin2), min_us_(min_us), max_us_(max_us) {}

int EscPair::throttleToUs(float t) const {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return min_us_ + static_cast<int>(t * (max_us_ - min_us_) + 0.5f);
}

void EscPair::begin(uint32_t arm_ms) {
  ensureTimersAllocated();
  auto* e1 = new Servo();
  auto* e2 = new Servo();
  e1->setPeriodHertz(50);
  e2->setPeriodHertz(50);
  e1->attach(pin1_, min_us_, max_us_);
  e2->attach(pin2_, min_us_, max_us_);
  esc1_ = e1;
  esc2_ = e2;
  // Arm: hold minimum throttle so the ESCs accept the signal.
  disarm();
  delay(arm_ms);
}

void EscPair::writeThrottle(float t1, float t2) {
  if (esc1_ == nullptr || esc2_ == nullptr) return;
  static_cast<Servo*>(esc1_)->writeMicroseconds(throttleToUs(t1));
  static_cast<Servo*>(esc2_)->writeMicroseconds(throttleToUs(t2));
}

void EscPair::disarm() { writeThrottle(0.0f, 0.0f); }
