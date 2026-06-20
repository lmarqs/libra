#include "Pid.h"

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

Pid::Pid(Gains gains, float out_min, float out_max) : gains_(gains), out_min_(out_min), out_max_(out_max) {}

void Pid::setOutputLimits(float out_min, float out_max) {
  out_min_ = out_min;
  out_max_ = out_max;
}

void Pid::reset() {
  integral_ = 0.0f;
  has_prev_ = false;
}

float Pid::update(float setpoint, float measurement, float dt) {
  const float error = setpoint - measurement;

  // Derivative on measurement (negated below) avoids a kick when the setpoint
  // jumps. Skipped on the first sample and on non-positive dt.
  float d_measurement = 0.0f;
  if (dt > 0.0f) {
    integral_ += error * dt;
    if (has_prev_) d_measurement = (measurement - prev_measurement_) / dt;
  }
  prev_measurement_ = measurement;
  has_prev_ = true;

  // Anti-windup: keep the integral *contribution* within the output range so a
  // saturated actuator can't let the integrator run away.
  if (gains_.ki != 0.0f) {
    const float i_lo = out_min_ / gains_.ki;
    const float i_hi = out_max_ / gains_.ki;
    integral_ = clampf(integral_, i_lo < i_hi ? i_lo : i_hi, i_lo < i_hi ? i_hi : i_lo);
  }

  const float output = gains_.kp * error + gains_.ki * integral_ - gains_.kd * d_measurement;
  return clampf(output, out_min_, out_max_);
}
