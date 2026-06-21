// Libra — self-balancing beam (ESP32-C3 single-core and ESP32 WROOM-32 dual-core).
//
// A dedicated fixed-rate control task reads the IMU, runs the control policy
// (complementary filter -> PID -> mixer), and drives the two ESCs. The only software
// safeguard is the tilt failsafe; arming is a HARDWARE switch on the ESC supply (the
// firmware always outputs the control signal). Tuning + telemetry are over USB serial
// and an optional WiFi web UI (setpoint + gains only).
//
// Concurrency model (see CLAUDE.md): the control task is pinned to
// ARDUINO_RUNNING_CORE — the APP core (1) on the dual-core WROOM, isolated from the
// WiFi/lwIP/httpd stack on the PRO core (0); the only core on the single-core C3,
// where priority + a per-period yield keep WiFi alive instead.
//
//   ⚠️  Arming is a PHYSICAL switch on the ESC supply — keep it OFF until ready
//       (props off / beam clamped while you trust your gains). The firmware does not
//       gate the motors: past the tilt limit the failsafe cuts output to idle and
//       auto-resumes once the beam is back within the limit.
//
// Serial commands (USB-CDC, 115200): kp <v> | ki <v> | kd <v> | sp <v> | ?
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

// Operator + control state — single execution context (controlTask), so plain variables.
static Pid::Gains gains{config::kKp, config::kKi, config::kKd};
static float setpoint = config::kSetpointDeg;
static ControlOutputs last{};

// --- Control task placement (timers/tasks/loops; dual-arch) ---
#ifndef ARDUINO_RUNNING_CORE
#define ARDUINO_RUNNING_CORE 0  // defensive; the framework defines it (1 on dual, 0 on C3)
#endif
// Run control on ARDUINO_RUNNING_CORE: the APP core (1) on the dual-core WROOM —
// physically isolated from WiFi/lwIP/httpd on the PRO core (0) — and the only core on
// the single-core C3, where the priority below + the per-period yield keep WiFi alive.
static constexpr BaseType_t kControlCore = ARDUINO_RUNNING_CORE;
// Above httpd (tskIDLE_PRIORITY+5 = 5) so control preempts the web server on the C3;
// below lwIP (18) / WiFi (23) so the WiFi stack is never starved. MUST stay < 18.
static constexpr UBaseType_t kControlPrio = 10;
// Matches the Arduino loopTask budget; the step path uses String/snprintf/atan2f.
static constexpr uint32_t kControlStackBytes = 8192;

static void step(float dt) {
  web::poll(gains, setpoint);  // apply any web-set gains/setpoint before computing

  ImuSample s;
  if (!imu.read(s)) return;
  // The beam pivots about the IMU's Z axis; -s.gz is its rate (negated to match
  // the sign of the accel-derived angle, verified during bring-up). Flip if the
  // gyro fights the accelerometer instead of complementing it.
  balancer.setGains(gains);
  const float angle = Imu::accelAngleDeg(s) - config::kAngleOffsetDeg;  // zero-trim to level
  // No software master-enable: the control loop always runs (arming is a hardware switch
  // on the ESC supply). The tilt failsafe is the only software cutoff — past the limit
  // balancer.step() reports !active and we idle the motors, auto-resuming with a fresh PID
  // integrator once the beam is back within the limit.
  const ControlInputs in{angle, -s.gz, setpoint, dt, /*enabled=*/true};
  last = balancer.step(in);

  if (last.active) {
    escs.writeThrottle(last.m1, last.m2);
  } else {
    escs.disarm();
  }

  // Raw-IMU bring-up stream — compiled in only at debug verbosity
  // (LIBRA_LOG_LEVEL >= 4). Throttled to ~10 Hz at the 200 Hz loop.
#if CORE_DEBUG_LEVEL >= 4
  {
    static uint8_t n = 0;
    if (++n >= 20) {
      n = 0;
      log_d("ax=%+.3f ay=%+.3f az=%+.3f | gx=%+.2f gy=%+.2f gz=%+.2f | acc=%+.1f fused=%+.1f", s.ax, s.ay, s.az, s.gx,
            s.gy, s.gz, angle, last.angle);
      // Confirm the control task's stack budget is comfortable during bring-up.
      log_d("control: core=%d stack_free=%u words", xPortGetCoreID(), uxTaskGetStackHighWaterMark(nullptr));
    }
  }
#endif

  web::publish(last.angle, last.tripped, gains, setpoint);
}

static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line == "?") {
    log_i("state:%s angle=%.2f sp=%.2f out=%.3f m1=%.2f m2=%.2f kp=%.4f ki=%.4f kd=%.4f",
          last.tripped ? " TRIPPED" : "", last.angle, setpoint, last.output, last.m1, last.m2, gains.kp, gains.ki,
          gains.kd);
  } else if (line.startsWith("kp ")) {
    gains.kp = line.substring(3).toFloat();
  } else if (line.startsWith("ki ")) {
    gains.ki = line.substring(3).toFloat();
  } else if (line.startsWith("kd ")) {
    gains.kd = line.substring(3).toFloat();
  } else if (line.startsWith("sp ")) {
    setpoint = line.substring(3).toFloat();
  } else {
    log_w("commands: kp<v> ki<v> kd<v> sp<v> ?");
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

// The fixed-rate control loop, in its own task (see the concurrency note at the top).
static void controlTask(void*) {
  TickType_t next_wake = xTaskGetTickCount();
  uint32_t last_us = micros();
  log_i("control task on core %d (prio %u)", xPortGetCoreID(), uxTaskPriorityGet(nullptr));
  for (;;) {
    // Sleep until the next control period — yields the core to WiFi/idle housekeeping
    // instead of busy-spinning. This yield is load-bearing: dropping it starves the
    // FreeRTOS idle task (the C3's shared core, or the WROOM's dedicated core-1 idle),
    // tripping the task watchdog. On an overrun xTaskDelayUntil returns pdFALSE without
    // sleeping, so yield explicitly to keep idle fed.
    if (xTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1000UL / config::kLoopHz)) == pdFALSE) {
      taskYIELD();
    }

    pollSerial();  // polled once per period (200 Hz) — ample for typed commands

    const uint32_t now = micros();
    const float dt = (now - last_us) * 1e-6f;
    last_us = now;
    step(dt);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.setDebugOutput(true);  // route log_*() output to the USB-CDC serial
  log_i("boot — arming is the hardware ESC switch; keep it OFF until ready");

  Wire.begin(config::kI2cSda, config::kI2cScl);
  Wire.setClock(400000);
  // Bound a glitched/contended bus: a stalled read fails fast so step() just skips a
  // cycle instead of hanging the (now higher-priority) control task — which on the
  // WROOM would starve the core-1 idle task and trip its watchdog.
  Wire.setTimeOut(10);  // ms
  if (imu.begin()) {
    log_i("MPU6050 ready");
  } else {
    log_e("MPU6050 not found");
  }

  log_i("ESC arm signal (min throttle held ~3 s)...");
  escs.begin();  // blocks ~3 s arming; must complete before controlTask drives the ESCs
  log_i("ESCs ready (idle). Control loop runs; motors gated by the hardware switch.");

  web::begin();  // WiFi SoftAP + web UI (setpoint/gains only; arming stays serial)

  // Hand the loop off to a dedicated, core-pinned task and reclaim loopTask's stack.
  // Created last: the IMU, ESCs, and WiFi are all up before control starts driving.
  xTaskCreatePinnedToCore(controlTask, "control", kControlStackBytes, nullptr, kControlPrio, nullptr, kControlCore);
  vTaskDelete(nullptr);  // setup() runs in loopTask; controlTask now owns the control loop
}

// Never runs after setup() deletes loopTask, but Arduino requires the symbol.
void loop() {}
