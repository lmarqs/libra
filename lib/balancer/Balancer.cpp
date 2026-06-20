#include "Balancer.h"

#include <cmath>

Balancer::Balancer(ComplementaryFilter filter, Pid pid, Mixer mixer, float tilt_limit_deg)
    : filter_(filter), pid_(pid), mixer_(mixer), tilt_limit_deg_(tilt_limit_deg) {}

ControlOutputs Balancer::step(const ControlInputs& in) {
  const float angle = filter_.update(in.accel_angle, in.rate, in.dt);
  const bool tripped = fabsf(angle) > tilt_limit_deg_;
  const bool active = in.enabled && !tripped;

  float output = 0.0f, m1 = 0.0f, m2 = 0.0f;
  if (active) {
    if (!prev_active_) pid_.reset();  // fresh integrator on (re-)arm
    output = pid_.update(in.setpoint, angle, in.dt);
    const MotorCommands c = mixer_.mix(output);
    m1 = c.m1;
    m2 = c.m2;
  }
  prev_active_ = active;

  return ControlOutputs{angle, output, m1, m2, active, tripped};
}
