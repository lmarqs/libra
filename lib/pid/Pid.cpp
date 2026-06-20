#include "Pid.h"

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

Pid::Pid(Gains gains, float out_min, float out_max) : _gains(gains), _out_min(out_min), _out_max(out_max) {}

void Pid::setOutputLimits(float out_min, float out_max) {
  _out_min = out_min;
  _out_max = out_max;
}

void Pid::reset() {
  _integral = 0.0f;
  _has_prev = false;
}

float Pid::update(float setpoint, float measurement, float dt) {
  const float error = setpoint - measurement;

  // Derivative on measurement (negated below) avoids a kick when the setpoint
  // jumps. Skipped on the first sample and on non-positive dt.
  float d_measurement = 0.0f;
  if (dt > 0.0f) {
    _integral += error * dt;
    if (_has_prev) d_measurement = (measurement - _prev_measurement) / dt;
  }
  _prev_measurement = measurement;
  _has_prev = true;

  // Anti-windup: keep the integral *contribution* within the output range so a
  // saturated actuator can't let the integrator run away.
  if (_gains.ki != 0.0f) {
    const float i_lo = _out_min / _gains.ki;
    const float i_hi = _out_max / _gains.ki;
    _integral = clampf(_integral, i_lo < i_hi ? i_lo : i_hi, i_lo < i_hi ? i_hi : i_lo);
  }

  const float output = _gains.kp * error + _gains.ki * _integral - _gains.kd * d_measurement;
  return clampf(output, _out_min, _out_max);
}
