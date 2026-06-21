// Libra — ESP32-C3 self-balancing beam.
//
// Single-core firmware: one fixed-rate loop reads the IMU, runs the control
// policy (complementary filter -> PID -> mixer), and drives the two ESCs, with a
// tilt failsafe and a master enable. Tuning + telemetry are over USB serial and
// over an optional WiFi web UI (setpoint + gains only; arming stays serial).
//
//   ⚠️  Boots DISARMED. Enable over serial ('e'). Props off / beam clamped
//       until you trust your gains. Past the tilt limit the motors cut and you
//       must re-enable.
//
// Serial commands (USB-CDC, 115200): e | d | x | kp <v> | ki <v> | kd <v> | sp <v> | ?
// (d and x both disable; x is the emergency-stop alias.)
//
// Build with LIBRA_LOG_LEVEL>=4 (.env -> CORE_DEBUG_LEVEL) to stream raw IMU
// readings over serial via log_d() — used for IMU axis/bias bring-up.

#include <Arduino.h>
#include <Balancer.h>
#include <EscPair.h>
#include <Imu.h>
#include <Wire.h>

#include "config.h"
#include "web.h"

static Imu imu(config::kMpuAddress);
static EscPair escs(config::kEsc1Pin, config::kEsc2Pin);
static Balancer balancer(ComplementaryFilter(config::kFilterAlpha),
                         Pid({config::kKp, config::kKi, config::kKd}, -config::kPidOutLimit, config::kPidOutLimit),
                         Mixer(config::kBaseThrottle, 0.0f, config::kMaxThrottle), config::kTiltLimitDeg);

// Operator + control state — single execution context, so plain variables.
static Pid::Gains gains{config::kKp, config::kKi, config::kKd};
static float setpoint = config::kSetpointDeg;
static bool enabled = false;  // boots disarmed
static ControlOutputs last{};

static void step(float dt) {
  web::poll(gains, setpoint);  // apply any web-set gains/setpoint before computing

  ImuSample s;
  if (!imu.read(s)) return;
  // The beam pivots about the IMU's Z axis; -s.gz is its rate (negated to match
  // the sign of the accel-derived angle, verified during bring-up). Flip if the
  // gyro fights the accelerometer instead of complementing it.
  balancer.setGains(gains);
  const ControlInputs in{Imu::accelAngleDeg(s), -s.gz, setpoint, dt, enabled};
  last = balancer.step(in);

  if (last.active) {
    escs.writeThrottle(last.m1, last.m2);
  } else {
    escs.disarm();
  }
  if (last.tripped) enabled = false;  // latch disabled until re-enabled

  // Raw-IMU bring-up stream — compiled in only at debug verbosity
  // (LIBRA_LOG_LEVEL >= 4). Throttled to ~10 Hz at the 200 Hz loop.
#if CORE_DEBUG_LEVEL >= 4
  {
    static uint8_t n = 0;
    if (++n >= 20) {
      n = 0;
      log_d("ax=%+.3f ay=%+.3f az=%+.3f | gx=%+.2f gy=%+.2f gz=%+.2f | acc=%+.1f fused=%+.1f", s.ax, s.ay, s.az, s.gx,
            s.gy, s.gz, Imu::accelAngleDeg(s), last.angle);
    }
  }
#endif

  web::publish(last.angle, enabled, last.tripped, gains, setpoint);
}

static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line == "e") {
    enabled = true;
    Serial.println("balance: ENABLED");
  } else if (line == "d" || line == "x") {
    enabled = false;
    Serial.println("balance: disabled");
  } else if (line == "?") {
    Serial.printf("state: %s%s angle=%.2f sp=%.2f out=%.3f m1=%.2f m2=%.2f kp=%.4f ki=%.4f kd=%.4f\n",
                  enabled ? "ON " : "OFF", last.tripped ? " (TRIPPED)" : "", last.angle, setpoint, last.output, last.m1,
                  last.m2, gains.kp, gains.ki, gains.kd);
  } else if (line.startsWith("kp ")) {
    gains.kp = line.substring(3).toFloat();
  } else if (line.startsWith("ki ")) {
    gains.ki = line.substring(3).toFloat();
  } else if (line.startsWith("kd ")) {
    gains.kd = line.substring(3).toFloat();
  } else if (line.startsWith("sp ")) {
    setpoint = line.substring(3).toFloat();
  } else {
    Serial.println("?: e d x kp<v> ki<v> kd<v> sp<v> ?");
  }
}

static void pollSerial() {
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
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.setDebugOutput(true);  // route log_*() output to the USB-CDC serial
  Serial.println("libra: boot — DISARMED");

  Wire.begin(config::kI2cSda, config::kI2cScl);
  Wire.setClock(400000);
  Serial.println(imu.begin() ? "libra: MPU6050 ready" : "libra: MPU6050 not found");

  Serial.println("libra: arming ESCs...");
  escs.begin();
  Serial.println("libra: armed (motors idle). PROPS OFF / BEAM CLAMPED.");

  web::begin();  // WiFi SoftAP + web UI (setpoint/gains only; arming stays serial)
}

void loop() {
  pollSerial();

  static uint32_t last_us = micros();
  const uint32_t now = micros();
  if (now - last_us >= config::kLoopPeriodUs) {
    const float dt = (now - last_us) * 1e-6f;
    last_us = now;
    step(dt);
  }
}
