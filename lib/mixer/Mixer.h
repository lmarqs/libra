#pragma once

// Per-motor throttle commands, 0..1.
struct MotorCommands {
  float m1;
  float m2;
};

// Turns the PID's single corrective output into two propeller throttles around
// a base (hover) throttle: one motor speeds up while the other slows down, which
// is what rotates the beam. Outputs are clamped to [min, max] so a large
// correction can't command negative or over-range throttle. Pure math —
// host-testable.
class Mixer {
 public:
  // base is the common throttle both motors idle at (0..1); the PID correction
  // is added to motor 1 and subtracted from motor 2.
  Mixer(float base, float min_out = 0.0f, float max_out = 1.0f);

  void setBase(float base) { _base = base; }
  float base() const { return _base; }

  MotorCommands mix(float correction) const;

 private:
  float _base;
  float _min_out;
  float _max_out;
};
