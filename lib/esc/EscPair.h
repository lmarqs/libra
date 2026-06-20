#pragma once

#include <cstdint>

// Drives the two propeller ESCs with standard 1000–2000 µs servo PWM at 50 Hz.
//
// ESCs arm by seeing minimum throttle held for a couple of seconds at power-up;
// begin() does that. Throttle is given as 0..1 and mapped into the µs range, so
// callers never deal with pulse widths directly. Everything is clamped to the
// configured limits — a runaway controller can't command past full throttle.
//
// Hardware-only (wraps ESP32Servo / LEDC), so it is never built by the native
// test env. The pure throttle-mixing math it consumes lives in Mixer.
class EscPair {
 public:
  EscPair(int pin1, int pin2, int min_us = 1000, int max_us = 2000);

  // Attach both channels and arm: hold minimum throttle for arm_ms so the ESCs
  // accept the signal. Blocks for arm_ms.
  void begin(uint32_t arm_ms = 3000);

  // Set each motor's throttle, 0..1, clamped to [min_us, max_us].
  void writeThrottle(float t1, float t2);

  // Cut both motors to minimum throttle (idle / not spinning).
  void disarm();

  int minUs() const { return _min_us; }
  int maxUs() const { return _max_us; }

 private:
  int throttleToUs(float t) const;

  int _pin1, _pin2;
  int _min_us, _max_us;
  // Servo objects are held in the .cpp via pimpl-free statics to keep ESP32Servo
  // out of this header (so the header stays cheap to include).
  void* _esc1 = nullptr;
  void* _esc2 = nullptr;
};
