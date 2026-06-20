#include "Balancer.h"

#include <cmath>

Balancer::Balancer(ComplementaryFilter filter, Pid pid, Mixer mixer, float tilt_limit_deg)
    : _filter(filter), _pid(pid), _mixer(mixer), _tilt_limit_deg(tilt_limit_deg) {}

ControlOutputs Balancer::step(const ControlInputs& in) {
  const float angle = _filter.update(in.accel_angle, in.rate, in.dt);
  const bool tripped = fabsf(angle) > _tilt_limit_deg;
  const bool active = in.enabled && !tripped;

  float output = 0.0f, m1 = 0.0f, m2 = 0.0f;
  if (active) {
    if (!_prev_active) _pid.reset();  // fresh integrator on (re-)arm
    output = _pid.update(in.setpoint, angle, in.dt);
    const MotorCommands c = _mixer.mix(output);
    m1 = c.m1;
    m2 = c.m2;
  }
  _prev_active = active;

  return ControlOutputs{angle, output, m1, m2, active, tripped};
}
