#include "control_state.h"

#include <Arduino.h>

#include "config.h"

namespace {
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

struct Shared {
  Pid::Gains gains{config::kKp, config::kKi, config::kKd};
  float setpoint = config::kSetpointDeg;
  bool enabled = false;  // boots disarmed
  float angle = 0.0f;
  float output = 0.0f;
  float m1 = 0.0f;
  float m2 = 0.0f;
  bool tripped = false;
};
Shared g;
}  // namespace

namespace state {

Commands commands() {
  taskENTER_CRITICAL(&mux);
  Commands c{g.gains, g.setpoint, g.enabled};
  taskEXIT_CRITICAL(&mux);
  return c;
}

void setEnabled(bool on) {
  taskENTER_CRITICAL(&mux);
  g.enabled = on;
  if (on) g.tripped = false;
  taskEXIT_CRITICAL(&mux);
}

void setKp(float value) {
  taskENTER_CRITICAL(&mux);
  g.gains.kp = value;
  taskEXIT_CRITICAL(&mux);
}

void setKi(float value) {
  taskENTER_CRITICAL(&mux);
  g.gains.ki = value;
  taskEXIT_CRITICAL(&mux);
}

void setKd(float value) {
  taskENTER_CRITICAL(&mux);
  g.gains.kd = value;
  taskEXIT_CRITICAL(&mux);
}

void setSetpoint(float deg) {
  taskENTER_CRITICAL(&mux);
  g.setpoint = deg;
  taskEXIT_CRITICAL(&mux);
}

void tripFailsafe() {
  taskENTER_CRITICAL(&mux);
  g.enabled = false;
  g.tripped = true;
  taskEXIT_CRITICAL(&mux);
}

void publish(const ControlOutputs& out) {
  taskENTER_CRITICAL(&mux);
  g.angle = out.angle;
  g.output = out.output;
  g.m1 = out.m1;
  g.m2 = out.m2;
  taskEXIT_CRITICAL(&mux);
}

Telemetry telemetry() {
  taskENTER_CRITICAL(&mux);
  Telemetry t{g.angle, g.output, g.m1, g.m2, g.enabled, g.tripped};
  taskEXIT_CRITICAL(&mux);
  return t;
}

}  // namespace state
