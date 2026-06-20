// Libra — ESP32-CAM self-balancing beam.
//
// Full firmware. A control task pinned to core 1 runs the fixed-rate loop
// IMU -> complementary filter -> PID -> mixer -> ESCs, with a tilt failsafe and
// master enable. Core 0 hosts a WiFi AP + web UI (camera FPV stream + telemetry
// OSD + live gain tuning). Shared state crosses cores through control_state.
//
//   ⚠️  Boots DISARMED. Enable from the web UI (or serial 'e'). Props off /
//       beam clamped until you trust your gains. Past the tilt limit the
//       motors cut and you must re-enable.
//
// Serial commands (UART0, 115200): e | d | x | kp <v> | ki <v> | kd <v> | sp <v> | ?
// (d and x both disable; x is the emergency-stop alias.)

#include <Arduino.h>
#include <Balancer.h>
#include <EscPair.h>
#include <Imu.h>
#include <WiFi.h>
#include <Wire.h>

#include "camera.h"
#include "config.h"
#include "control_state.h"
#include "web.h"

// Control objects — owned and mutated only by the control task.
static Imu imu(config::kMpuAddress);
static EscPair escs(config::kEsc1Pin, config::kEsc2Pin);
static Balancer balancer(ComplementaryFilter(config::kFilterAlpha),
                         Pid({config::kKp, config::kKi, config::kKd}, -config::kPidOutLimit, config::kPidOutLimit),
                         Mixer(config::kBaseThrottle, 0.0f, config::kMaxThrottle), config::kTiltLimitDeg);

static void controlTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t last_us = micros();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / config::kLoopHz));
    const uint32_t now = micros();
    const float dt = (now - last_us) * 1e-6f;
    last_us = now;

    const Commands cmd = state::commands();

    ImuSample s;
    if (!imu.read(s)) continue;
    // s.gx is the rate about the pivot axis; flip its sign if bring-up shows the
    // gyro fighting the accelerometer instead of complementing it.
    balancer.setGains(cmd.gains);
    const ControlInputs in{Imu::accelAngleDeg(s), s.gx, cmd.setpoint, dt, cmd.enabled};
    const ControlOutputs out = balancer.step(in);

    if (out.active) {
      escs.writeThrottle(out.m1, out.m2);
    } else {
      escs.disarm();
    }
    if (out.tripped) state::tripFailsafe();  // latch disabled until re-enabled

    state::publish(out);
  }
}

static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line == "e") {
    state::setEnabled(true);
    Serial.println("balance: ENABLED");
  } else if (line == "d" || line == "x") {
    state::setEnabled(false);
    Serial.println("balance: disabled");
  } else if (line == "?") {
    const Telemetry t = state::telemetry();
    const Commands c = state::commands();
    Serial.printf("state: %s%s angle=%.2f sp=%.2f out=%.3f m1=%.2f m2=%.2f kp=%.4f ki=%.4f kd=%.4f\n",
                  t.enabled ? "ON " : "OFF", t.tripped ? " (TRIPPED)" : "", t.angle, c.setpoint, t.output, t.m1, t.m2,
                  c.gains.kp, c.gains.ki, c.gains.kd);
  } else if (line.startsWith("kp ")) {
    state::setKp(line.substring(3).toFloat());
  } else if (line.startsWith("ki ")) {
    state::setKi(line.substring(3).toFloat());
  } else if (line.startsWith("kd ")) {
    state::setKd(line.substring(3).toFloat());
  } else if (line.startsWith("sp ")) {
    state::setSetpoint(line.substring(3).toFloat());
  } else {
    Serial.println("?: e d x kp<v> ki<v> kd<v> sp<v> ?");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("libra: boot — DISARMED");

  Wire.begin(config::kI2cSda, config::kI2cScl);
  Wire.setClock(400000);
  Serial.println(imu.begin() ? "libra: MPU6050 ready" : "libra: MPU6050 not found");

  // Camera first so it claims LEDC timer 0 before the ESCs grab the others.
  Serial.println(cameraInit() ? "libra: camera ready" : "libra: camera init FAILED");

  Serial.println("libra: arming ESCs...");
  escs.begin();
  Serial.println("libra: armed (motors idle). PROPS OFF / BEAM CLAMPED.");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(config::kApSsid, config::kApPassword);
  Serial.printf("libra: AP '%s' -> http://%s/\n", config::kApSsid, WiFi.softAPIP().toString().c_str());
  Serial.println(webStart() ? "libra: web up" : "libra: web FAILED");

  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr,
                          config::kControlCore);
}

void loop() {
  static String line;
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      handleCommand(line);
      line = "";
    } else {
      line += c;
    }
  }
  delay(5);
}
