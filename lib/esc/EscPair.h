#pragma once

#include <cstdint>

// Drives the two propeller ESCs with 1000–2000 µs servo PWM generated directly off the
// ESP32 LEDC timer (not ESP32Servo), so the pulse width is sub-µs precise — see the .cpp for
// the refresh + resolution (≈0.3 µs/step), well below the 1 µs floor of integer-µs
// writeMicroseconds. Throttle is given as 0..1 and mapped into the µs range, clamped to the
// configured limits, so a runaway controller can't command past full throttle.
//
// Hardware-only (wraps LEDC), so it is never built by the native test env. The pure
// throttle-mixing math it consumes lives in Mixer.
class EscPair {
 public:
  EscPair(int pin1, int pin2, int min_us = 1000, int max_us = 2000);

  // Configure the LEDC channels and arm: hold minimum throttle for arm_ms so the ESCs
  // accept the signal. Blocks for arm_ms.
  void begin(uint32_t arm_ms = 3000);

  // Set each motor's throttle, 0..1, clamped to [min_us, max_us].
  void writeThrottle(float t1, float t2);

  // Cut both motors to minimum throttle (idle / not spinning).
  void disarm();

  int minUs() const { return _min_us; }
  int maxUs() const { return _max_us; }

 private:
  void writeChannel(int channel, float t) const;

  int _pin1, _pin2;
  int _min_us, _max_us;
  bool _ready = false;
};
