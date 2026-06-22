# CLAUDE.md

Working guide for this repo. See `README.md` for the project overview + architecture diagrams.

## ⚠️ Safety — this firmware spins propellers (read first)

Physical-harm risk. These are hard guardrails, not suggestions:

- **Arming is two-layer** (explicit operator decision, 2026-06-21): a **HARDWARE switch on the ESC power supply** (the energy gate — keep it OFF at boot and when handling the beam) *plus* a **software master-enable** that boots **DISARMED** and is armed/disarmed from the web UI. Both must be on for the motors to drive; the firmware idles the ESCs while software-disarmed.
- **NEVER** weaken, bypass, or remove the **tilt failsafe** — past `kTiltLimitDeg` the balancer cuts the motors and the control loop latches **DISARMED** (re-arm from the web). Treat it as sacrosanct.
- **NEVER** widen the throttle band or raise the ESC µs range without explicit user confirmation. The band is `[kMinThrottle, kMaxThrottle]`, set from `.env` via `LIBRA_THROTTLE_MIN` / `LIBRA_THROTTLE_MAX`; the hover throttle (`kBaseThrottle` = band midpoint) and the PID authority (`kPidOutLimit` = half the band) are **derived** from it, so widening the band raises both thrust and control authority. The committed default is a **zero band `[0.0, 0.0]`** (2026-06-22): hover 0, authority 0 — a fresh build is **inert** (never drives the motors) until the operator sets a real band for their motor. The fractions map onto the ESC's ~1000–2000 µs range (0.0 → 1000 µs idle, 1.0 → 2000 µs full). The props-off bench commands (`set motors_enabled` / `set motors_speed`, compiled in only with `LIBRA_BENCH_ENABLED=1`) drive the ESCs directly and **skip both the `kMaxThrottle` cap and the tilt failsafe** — but never change the band. Autonomous balancing keeps both safeguards (clamped to the band; tilt failsafe live).
- The web UI sets the setpoint, gains, **and arm/disarm** (the software master-enable; boots disarmed). It must **NEVER** change a limit, and a web-set setpoint is clamped to the tilt limit. Because arming is exposed, set `LIBRA_AP_PASS` (WPA2) whenever props are on or anyone could be in range — on an open AP any client can arm.
- Assume props are on: any change that could command thrust must keep the safe-by-default path intact.

## Toolchain — always go through mise

PlatformIO lives in a Python venv managed by [mise](https://mise.jdx.dev/), which
also loads `.env`. Do **not** call `pio` directly — run the mise tasks:

- `mise run build` — compile (default env `esp32-wroom32`; `-e esp32-c3-super-mini` for the C3)
- `mise run upload` / `mise run monitor` / `mise run run`
- `mise run test` — host unit tests (`pio test -e native`)
- `mise run format` / `mise run format-check` — clang-format
- `mise run probe` / `banner` / `stream` — non-interactive serial debug (`tools/serial_dbg.py`)

## Verification — gate, not optional

Before declaring any change done, these MUST pass:

- `mise run test` — host unit tests for the pure libs
- `mise run format-check` — clang-format clean
- `mise run build:all` — both boards still compile (the dual-arch gate)

Add/extend Unity tests in `test/` when you touch a pure lib. The pure libs are the
fast loop — do logic work there, not on hardware.

For hardware bring-up + on-board debugging (talking to the board, the debug log
level, IMU axis/offset bring-up, WSL serial gotchas), see
**[docs/testing.md](docs/testing.md)**.

## Build-time config

Two tiers of `-D` build flags:

- **Board pin map** (I2C SDA/SCL, ESC pins) — board hardware, so literal flags per
  `[env]` in `platformio.ini` (C3 and WROOM-32 differ), **not** from `.env`.
- **Deployment tunables** (throttle, tilt, IMU offset, AP SSID, log level) — from
  `.env` (gitignored; `mise run setup` copies `.env.example`), shared across boards
  via the `[tunables]` section and `${tunables.build_flags}`.

Adding a **tunable** knob:

- **Name** it `LIBRA_<AREA>_<NAME>` (e.g. `LIBRA_THROTTLE_MAX`).
- **Default + document** it in `.env.example`; add it to the `[tunables]` block —
  numbers `-DX=${sysenv.X}`, strings quote-wrapped `-DX='"${sysenv.X}"'`.
- **src/config.h**: guard with `#ifndef` and bind a `constexpr`. config.h is the
  flag-less default — the source of truth for tunables, and the fallback for pins
  when an `[env]` omits the literal:
  ```cpp
  #ifndef LIBRA_THROTTLE_MAX
  #define LIBRA_THROTTLE_MAX 0.05f
  #endif
  constexpr float kMaxThrottle = LIBRA_THROTTLE_MAX;
  ```
- **Never commit real values** — `.env` is gitignored.

## Layout & the host/hardware split

Pure control logic in `lib/` is host-tested via the `native` env; hardware drivers are not.

- `lib/pid`, `lib/filter`, `lib/mixer` — **pure C++, no Arduino deps, host-tested. Keep them that way.**
- `lib/balancer` — composes filter+pid+mixer into the balancing policy (failsafe, re-arm reset). The control loop's logic lives here, not in `main`.
- `lib/imu`, `lib/esc` — hardware drivers (I2C / LEDC PWM). Arduino-only.
- `src/web` — WiFi SoftAP + HTTP server (ESP-IDF `httpd`) for the tuning web UI. Arduino-only. The `httpd` handlers run in their own task, so this is the **one** cross-context shared-state point: it's guarded by a small spinlock (`taskENTER_CRITICAL`), exchanged with the loop via `web::poll()` / `web::publish()`. The control loop stays the single owner of `gains`/`setpoint`.
- `src/main.cpp` — setup + a dedicated fixed-rate `controlTask` (pinned, see Gotchas): poll serial (non-blocking), then each control step applies any pending web command → reads IMU → `balancer.step()` → drives ESCs → publishes telemetry. The control task is the only thing that drives the ESCs. `loop()` is an empty stub — `setup()` spawns `controlTask` and `vTaskDelete`s the Arduino loopTask.
- `src/config.h` — pin defaults (flag-less fallback), limits, loop rate, gains, AP SSID (open AP).

The `native` env compiles only the libs a test includes — so **never `#include <Arduino.h>`** (or `Wire.h`, `esp32-hal-ledc.h`, …) from `lib/pid`, `lib/filter`, `lib/mixer`.

## Conventions

- clang-format (Google base, 2-space, 120 cols); private members use a **leading** underscore (`_member`).
- The fixed-rate `controlTask` sleeps to its period via `xTaskDelayUntil` (keep that yield — see Gotchas); serial handling stays non-blocking so it never stalls a step.

## Gotchas (append as you discover them)

- _(2026-06-20)_ The C3 Super Mini uses native USB-CDC for serial. The build flags `-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1` route `Serial` to the USB port; without them there's no serial over USB. (The WROOM-32 uses a USB-UART bridge instead — no native-USB flags, flashed over `/dev/ttyUSB*`.)
- _(2026-06-20)_ Tuning + telemetry are over serial **and** an optional WiFi web UI (`src/web`): a SoftAP (open by default — set `LIBRA_AP_PASS`) + `httpd` on :80 serving setpoint/gain sliders + ARM/DISARM buttons (arming model: see Safety, above). There is still no camera. The WiFi stack roughly doubles flash use (~60%) and adds RAM; it preempts the loop on the single core, but the loop measures `dt` so it absorbs the jitter.
- _(2026-06-21)_ `mise run monitor` looking dead is **normal**: the firmware only prints on boot + on command, and native USB-CDC doesn't reset the chip on attach (so the banner's already gone). Type `?`, press RST, or use `mise run probe`/`banner`/`stream`. Debug verbosity is `LIBRA_LOG_LEVEL` → `CORE_DEBUG_LEVEL`; build with `>=4` to compile in the ~10 Hz raw-IMU `log_d` stream.
- _(2026-06-21)_ Upload failing with `Could not exclusively lock port … Resource temporarily unavailable` means a monitor holds `/dev/ttyACM0`. Close `mise run monitor`/`run` (or find the holder via `lsof`/`fuser`/`/proc/*/fd`), then re-upload.
- _(2026-06-21)_ `controlTask` sleeps to the next control period via `xTaskDelayUntil` instead of busy-spinning — a tight no-yield loop starves the FreeRTOS IDLE task, tripping the task watchdog and squeezing WiFi/lwIP housekeeping (closed sockets, DHCP/TCP timers) → the AP/web UI gets unstable. Any future edit to `controlTask` MUST keep that yield (on a deadline overrun `xTaskDelayUntil` returns `pdFALSE` without sleeping, so we `taskYIELD()` to still feed idle). Same reason `web::setHandler` parses query args *before* taking the `taskENTER_CRITICAL` spinlock (no string/float parsing with interrupts disabled on the WiFi core).
- _(2026-06-21)_ **Dual-arch core map.** The control loop runs in a dedicated `controlTask` pinned to `ARDUINO_RUNNING_CORE`: the **APP core (1)** on the dual-core WROOM-32 — physically isolated from the WiFi/lwIP/`httpd` stack on the **PRO core (0)** — and the **only core (0)** on the single-core C3. Two mechanisms enforce the same isolation, **keep both**: (1) core *pinning* (carries the WROOM), and (2) `controlTask` priority `10` > `httpd` (5) so control preempts the web server on the C3, while staying < lwIP (18)/WiFi (23) so the WiFi stack is never starved. `httpd` is pinned to core 0 (`web.cpp`), correct on both. Consequence on the WROOM: `controlTask` is the only user task on core 1, so its `xTaskDelayUntil` yield is what feeds **core-1's** idle task → a control step that overruns the 5 ms period can trip the core-1 idle WDT. `Wire.setTimeOut(10)` bounds the one blocking call (the I2C IMU read) so a glitched bus skips a step instead of hanging. `controlTask` gets an 8 KB stack (matches loopTask; it uses `String`/`snprintf`/`atan2f`) — a `LIBRA_LOG_LEVEL>=4` build logs `uxTaskGetStackHighWaterMark` to confirm headroom.
- _(2026-06-21)_ **All serial output goes through the Arduino `log_*` macros** (`log_e`/`log_w`/`log_i`/`log_d`), not raw `Serial.print*` — uniform `[time][level][file:line] func():` prefix, no `libra:`/`balance:`/`state:` text prefixes (file:line identifies the source). Levels: failures (`MPU6050 not found`, WiFi/httpd start) = `log_e`; invalid-command help + bench/calibration prompts = `log_w`; boot banner + `state:` replies + AP join/leave = `log_i`; IMU bring-up stream = `log_d` (`>=4`). So `LIBRA_LOG_LEVEL` gates **everything** — below 3 the banner and command replies vanish (so `probe`/`banner` need `>=3`, the default); errors survive to 1. `Serial.setDebugOutput(true)` (set in `setup()`) is what routes `log_*` to USB-CDC and is now load-bearing for all output. `Serial` itself is input-only now (`pollSerial`). Don't reintroduce `Serial.print*` for diagnostics — `grep -rn 'Serial\.print' src` should stay empty.
- _(2026-06-21)_ **Bench / ESC-calibration commands (props off).** Compiled in only with `LIBRA_BENCH_ENABLED=1` (default 0 — a props-on build has neither the commands nor the safeguard bypass; `tools/sweep.py` / `mise run sweep` needs this build). `set motors_enabled on|off` is the manual-drive master switch and `set motors_speed <m1> <m2>` sets each motor's throttle (0..1); `run` hands control back to the balancer, `x` forces idle. They set a `benchActive` override that `step()` checks **first** — it writes the per-motor speeds (or 0 when `motorsOn` is false) to the ESCs and returns *before* `balancer.step()`, so in bench mode **both the `kMaxThrottle` cap and the tilt failsafe are skipped** (required: calibration holds true MAX/MIN regardless of tilt). It still calls `web::publish(..., bench=true, ...)` so the web UI never shows a safe-looking state while motors are live. Speeds default to 0 and only apply once enabled, so nothing ever jumps to throttle on its own (the earlier auto-MAX `cal` command was dropped for being unsafe). Values are clamped to the full ESC range by the pure host-tested `throttle::clamp01` (`lib/throttle`). Calibration needs MAX present *at power-up*, but `escs.begin()` holds MIN for ~3 s at boot, so the ESC must be on a switch **separate from the board**: `set motors_speed 1 1` + `set motors_enabled on` (streams MAX) → switch ESC on → `set motors_speed 0 0` → `run`. Full procedure in docs/testing.md §7.
