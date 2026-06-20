#pragma once

// A discrete PID controller for the balance loop. Pure math, no Arduino
// dependencies — host-testable.
//
// Design choices that matter on a real plant:
//   - Derivative on measurement, not error, so a setpoint step doesn't produce
//     a "derivative kick" spike into the motors.
//   - Output clamped to [out_min, out_max].
//   - Anti-windup: the integral contribution is clamped to the output range, so
//     the integrator can't wind up while the actuator is saturated.
class Pid {
 public:
  struct Gains {
    float kp;
    float ki;
    float kd;
  };

  Pid(Gains gains, float out_min, float out_max);

  void setGains(Gains gains) { gains_ = gains; }
  Gains gains() const { return gains_; }

  void setOutputLimits(float out_min, float out_max);

  // Clear the integrator and derivative history (e.g. when re-arming).
  void reset();

  // Advance one step and return the clamped control output.
  //   dt must be > 0 (seconds). A non-positive dt returns the last output's
  //   proportional+integral response without updating the derivative.
  float update(float setpoint, float measurement, float dt);

  float integrator() const { return integral_; }

 private:
  Gains gains_;
  float out_min_;
  float out_max_;
  float integral_ = 0.0f;
  float prev_measurement_ = 0.0f;
  bool has_prev_ = false;
};
