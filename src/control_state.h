#pragma once

#include <Balancer.h>  // ControlOutputs
#include <Pid.h>       // Pid::Gains

// Thread-safe state shared between the control task (core 1) and the web server
// / serial console (core 0). All access goes through these functions, which use
// a spinlock internally — never touch the state directly.

// Operator commands flowing into the control loop.
struct Commands {
  Pid::Gains gains;
  float setpoint;
  bool enabled;
};

// Live values flowing out of the control loop for display.
struct Telemetry {
  float angle;
  float output;
  float m1;
  float m2;
  bool enabled;
  bool tripped;  // tilt failsafe has fired since the last enable
};

namespace state {

// Commands (web/serial -> control loop).
Commands commands();
void setEnabled(bool on);
void setKp(float value);
void setKi(float value);
void setKd(float value);
void setSetpoint(float deg);

// Failsafe (control loop -> everywhere): disable and latch the tripped flag.
void tripFailsafe();

// Telemetry (control loop -> web/serial).
void publish(const ControlOutputs& out);
Telemetry telemetry();

}  // namespace state
