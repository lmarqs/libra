#pragma once

#include <ComplementaryFilter.h>
#include <Mixer.h>
#include <Pid.h>

// One control step's inputs (from the sensor + operator) ...
struct ControlInputs {
  float accel_angle;  // tilt from gravity, deg
  float rate;         // angular rate about the pivot axis, deg/s
  float setpoint;     // target tilt, deg
  float dt;           // time since previous step, s
  bool enabled;       // operator master-enable
};

// ... and its outputs (to the actuators + display).
struct ControlOutputs {
  float angle;   // fused tilt estimate, deg
  float output;  // PID correction (throttle differential)
  float m1;      // motor 1 throttle, 0..1
  float m2;      // motor 2 throttle, 0..1
  bool active;   // motors are being driven this step
  bool tripped;  // tilt failsafe fired this step
};

// The balancing policy, composed from the pure control blocks. Owns the filter,
// PID, and mixer, and decides — per step — whether to drive the motors:
//   - past the tilt limit it trips the failsafe and idles the motors,
//   - while disabled it idles and holds the integrator clear,
//   - on each (re-)arm it resets the integrator so stale wind-up can't kick.
// Pure C++ (no Arduino), so the whole loop is host-testable, not just the parts.
class Balancer {
 public:
  Balancer(ComplementaryFilter filter, Pid pid, Mixer mixer, float tilt_limit_deg);

  void setGains(Pid::Gains gains) { pid_.setGains(gains); }

  ControlOutputs step(const ControlInputs& in);

 private:
  ComplementaryFilter filter_;
  Pid pid_;
  Mixer mixer_;
  float tilt_limit_deg_;
  bool prev_active_ = false;
};
