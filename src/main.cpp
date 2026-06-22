// Libra — self-balancing beam (ESP32-C3 single-core and ESP32 WROOM-32 dual-core).
//
// A dedicated fixed-rate control task reads the IMU, runs the control policy
// (complementary filter -> PID -> mixer), and drives the two ESCs — gated by a software
// master-enable (armed from the web UI; boots DISARMED) and the tilt failsafe. The ESC
// supply is gated by a separate HARDWARE switch. Tuning + telemetry are over USB serial
// and the WiFi web UI (setpoint, gains, arm/disarm).
//
// Concurrency model (see CLAUDE.md): the control task is pinned to
// ARDUINO_RUNNING_CORE — the APP core (1) on the dual-core WROOM, isolated from the
// WiFi/lwIP/httpd stack on the PRO core (0); the only core on the single-core C3,
// where priority + a per-period yield keep WiFi alive instead.
//
//   ⚠️  Boots DISARMED. Arm/disarm from the web UI (the software master-enable); the ESC
//       supply is a separate HARDWARE switch — keep it OFF until ready. Past the tilt limit
//       the failsafe cuts the motors and latches DISARMED — re-arm from the web. Props off
//       / beam clamped while you trust your gains.
//
// Serial commands (USB-CDC, 115200): kp <v> | ki <v> | kd <v> | sp <v> | ?
//   Bench / ESC calibration (PROPS OFF): set motors_enabled on|off | set motors_speed <m1> <m2> | run | x
//   (motors default to 0 and only move when enabled; raise to 1.0 then 0.0 by hand to calibrate.)
//
// Build with LIBRA_LOG_LEVEL>=4 (.env -> CORE_DEBUG_LEVEL) to stream raw IMU
// readings over serial via log_d() — used for IMU axis/bias bring-up.

#include <Arduino.h>
#include <Balancer.h>
#include <EscPair.h>
#include <Imu.h>
#include <Throttle.h>
#include <Wire.h>

#include "config.h"
#include "web.h"

static Imu imu(config::kMpuAddress);
static EscPair escs(config::kEsc1Pin, config::kEsc2Pin);
static Balancer balancer(ComplementaryFilter(config::kFilterAlpha),
                         Pid({config::kKp, config::kKi, config::kKd}, -config::kPidOutLimit, config::kPidOutLimit),
                         Mixer(config::kBaseThrottle, config::kMinThrottle, config::kMaxThrottle),
                         config::kTiltLimitDeg);

// Operator + control state — single execution context (controlTask), so plain variables.
static Pid::Gains gains{config::kKp, config::kKi, config::kKd};
static float setpoint = config::kSetpointDeg;
static bool enabled = false;  // software master-enable; boots DISARMED, armed from the web UI
static ControlOutputs last{};

// Bench / manual-drive override (controlTask-owned), compiled in only with LIBRA_BENCH_ENABLED.
// While `benchActive`, the control loop skips balancing and the operator drives each ESC
// directly: `motorsOn` gates whether the per-motor speeds are applied or both held at idle
// (min). Speeds default to 0, so nothing ever jumps to throttle on its own — for ESC
// calibration you raise them to 1.0 yourself. Props-off bench only; it bypasses the
// kMaxThrottle cap and the tilt failsafe, which is why it is gated off by default.
#if LIBRA_BENCH_ENABLED
static bool benchActive = false;              // manual mode engaged (balancing suspended)
static bool motorsOn = false;                 // drive the speeds, vs. hold both at idle
static float m1Speed = 0.0f, m2Speed = 0.0f;  // per-motor bench throttle, 0..1
#endif

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
#if LIBRA_BENCH_ENABLED
  // Bench override: drive each ESC at its manual speed (or hold idle) and skip balancing.
  // Still publish (with bench=true) so the web UI never shows a safe-looking state while
  // the motors are live.
  if (benchActive) {
    escs.writeThrottle(motorsOn ? m1Speed : 0.0f, motorsOn ? m2Speed : 0.0f);
    web::publish(last.angle, enabled, /*tripped=*/false, /*bench=*/true, gains, setpoint);
    return;
  }
#endif

  web::poll(gains, setpoint);  // apply any web-set gains/setpoint before computing
  const int arm = web::pollArm();
  if (arm >= 0) enabled = (arm == 1);  // web arm/disarm; control loop owns `enabled`

  ImuSample s;
  if (!imu.read(s)) return;
  // The beam pivots about the IMU's Z axis; -s.gz is its rate (negated to match
  // the sign of the accel-derived angle, verified during bring-up). Flip if the
  // gyro fights the accelerometer instead of complementing it.
  balancer.setGains(gains);
  const float angle = Imu::accelAngleDeg(s) - config::kAngleOffsetDeg;  // zero-trim to level
  const ControlInputs in{angle, -s.gz, setpoint, dt, enabled};
  last = balancer.step(in);

  if (last.active) {
    escs.writeThrottle(last.m1, last.m2);
  } else {
    escs.disarm();
  }
  if (last.tripped) enabled = false;  // tilt failsafe latches DISARMED — re-arm from the web

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

  web::publish(last.angle, enabled, last.tripped, /*bench=*/false, gains, setpoint);
}

static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line == "?") {
#if LIBRA_BENCH_ENABLED
    if (benchActive) {
      log_i("state: BENCH motors=%s m1=%.3f m2=%.3f sp=%.2f kp=%.4f ki=%.4f kd=%.4f", motorsOn ? "ON" : "off", m1Speed,
            m2Speed, setpoint, gains.kp, gains.ki, gains.kd);
    } else
#endif
    {
      log_i("state: %s%s angle=%.2f sp=%.2f out=%.3f m1=%.2f m2=%.2f kp=%.4f ki=%.4f kd=%.4f",
            enabled ? "ARMED" : "disarmed", last.tripped ? " TRIPPED" : "", last.angle, setpoint, last.output, last.m1,
            last.m2, gains.kp, gains.ki, gains.kd);
    }
  }
#if LIBRA_BENCH_ENABLED
  else if (line.startsWith("set motors_enabled ")) {
    // Master switch for manual bench drive. ON applies the per-motor speeds (which default to
    // 0, so enabling never spins on its own); OFF / x holds both at idle. Both suspend
    // balancing — 'run' hands control back to the balancer.
    String v = line.substring(19);
    v.trim();
    v.toLowerCase();
    if (v == "on" || v == "1" || v == "true") {
      benchActive = true;
      motorsOn = true;
      log_w("motors ENABLED (PROPS OFF!) — m1=%.3f m2=%.3f", m1Speed, m2Speed);
      if (m1Speed > config::kMaxThrottle || m2Speed > config::kMaxThrottle) {
        log_w("  note: above the balancing cap kMaxThrottle=%.3f (calibration level).", config::kMaxThrottle);
      }
    } else if (v == "off" || v == "0" || v == "false") {
      benchActive = true;
      motorsOn = false;
      log_i("motors OFF (idle). 'run' resumes balancing.");
    } else {
      log_w("usage: set motors_enabled on|off");
    }
  } else if (line.startsWith("set motors_speed ")) {
    // Per-motor bench throttle: set motors_speed <m1> <m2>, each 0..1. Stored at once and
    // applied while enabled; full range so you can teach the ESC max/min (1.0 then 0.0) by hand.
    String rest = line.substring(17);
    rest.trim();
    const int gap = rest.indexOf(' ');
    if (gap < 0) {
      log_w("usage: set motors_speed <m1> <m2>  (each 0..1)");
    } else {
      m1Speed = throttle::clamp01(rest.substring(0, gap).toFloat());
      m2Speed = throttle::clamp01(rest.substring(gap + 1).toFloat());
      const float u1 = escs.minUs() + m1Speed * (escs.maxUs() - escs.minUs());
      const float u2 = escs.minUs() + m2Speed * (escs.maxUs() - escs.minUs());
      log_i("motors_speed m1=%.4f (%.2f us) m2=%.4f (%.2f us)%s", m1Speed, u1, m2Speed, u2,
            motorsOn ? "" : " — enable with 'set motors_enabled on'");
    }
  } else if (line == "run") {
    benchActive = false;
    log_i("bench off — balancing");
  } else if (line == "x") {
    // Emergency idle (alias for 'set motors_enabled off'): hold both at min. The hardware ESC
    // switch is the real kill; this just parks the signal until 'run'.
    benchActive = true;
    motorsOn = false;
    log_w("STOP: motors idle. 'run' resumes balancing.");
  }
#endif
  else if (line.startsWith("kp ")) {
    gains.kp = line.substring(3).toFloat();
  } else if (line.startsWith("ki ")) {
    gains.ki = line.substring(3).toFloat();
  } else if (line.startsWith("kd ")) {
    gains.kd = line.substring(3).toFloat();
  } else if (line.startsWith("sp ")) {
    setpoint = line.substring(3).toFloat();
  } else {
#if LIBRA_BENCH_ENABLED
    log_w(
        "commands: kp<v> ki<v> kd<v> sp<v> ? | bench(PROPS OFF): set motors_enabled on|off | set motors_speed <m1> "
        "<m2> | run | x");
#else
    log_w("commands: kp<v> ki<v> kd<v> sp<v> ?");
#endif
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
  log_i("boot — DISARMED. Arm from the web UI; the ESC supply has its own hardware switch.");

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
  log_i("ESCs ready (idle). DISARMED — arm from the web UI to balance.");

  web::begin();  // WiFi SoftAP + web UI (setpoint/gains only; arming stays serial)

  // Hand the loop off to a dedicated, core-pinned task and reclaim loopTask's stack.
  // Created last: the IMU, ESCs, and WiFi are all up before control starts driving.
  xTaskCreatePinnedToCore(controlTask, "control", kControlStackBytes, nullptr, kControlPrio, nullptr, kControlCore);
  vTaskDelete(nullptr);  // setup() runs in loopTask; controlTask now owns the control loop
}

// Never runs after setup() deletes loopTask, but Arduino requires the symbol.
void loop() {}
