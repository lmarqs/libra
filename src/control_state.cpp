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

void setGain(char which, float value) {
  taskENTER_CRITICAL(&mux);
  if (which == 'p') g.gains.kp = value;
  if (which == 'i') g.gains.ki = value;
  if (which == 'd') g.gains.kd = value;
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

void publish(float angle, float output, float m1, float m2) {
  taskENTER_CRITICAL(&mux);
  g.angle = angle;
  g.output = output;
  g.m1 = m1;
  g.m2 = m2;
  taskEXIT_CRITICAL(&mux);
}

Telemetry telemetry() {
  taskENTER_CRITICAL(&mux);
  Telemetry t{g.angle, g.output, g.m1, g.m2, g.enabled, g.tripped};
  taskEXIT_CRITICAL(&mux);
  return t;
}

}  // namespace state
