# CLAUDE.md

Working guide for this repo. See `README.md` for the project overview + architecture diagrams.

## ⚠️ Safety — this firmware spins propellers (read first)

Physical-harm risk. These are hard guardrails, not suggestions:

- **NEVER** weaken, bypass, or remove the tilt failsafe, the master-enable gate, or the disarmed-on-boot default.
- **NEVER** raise thrust/output limits (`kMaxThrottle`, `kPidOutLimit`, `kBaseThrottle`, ESC µs range) in `config.h` without explicit user confirmation.
- The web UI may set the setpoint + gains only; **NEVER** expose arm/disarm (or limits) over WiFi without explicit confirmation. Arming stays serial-only, and a web-set setpoint is clamped to the tilt limit.
- Assume props are on: any change that could command thrust must keep the safe-by-default path intact.

## Toolchain — always go through mise

PlatformIO lives in a Python venv managed by [mise](https://mise.jdx.dev/), which
also loads `.env`. Do **not** call `pio` directly — run the mise tasks:

- `mise run build` — compile (default env `esp32-c3-super-mini`)
- `mise run upload` / `mise run monitor` / `mise run run`
- `mise run test` — host unit tests (`pio test -e native`)
- `mise run format` / `mise run format-check` — clang-format
- `mise run probe` / `banner` / `stream` — non-interactive serial debug (`tools/serial_dbg.py`)

## Verification — gate, not optional

Before declaring any change done, these MUST pass:

- `mise run test` — host unit tests for the pure libs
- `mise run format-check` — clang-format clean

Add/extend Unity tests in `test/` when you touch a pure lib. The pure libs are the
fast loop — do logic work there, not on hardware.

For hardware bring-up + on-board debugging (talking to the board, the debug log
level, IMU axis/offset bring-up, WSL serial gotchas), see
**[docs/testing.md](docs/testing.md)**.

## Build-time config (.env → build flags)

Tunable constants come from `.env` (gitignored; `mise run setup` copies
`.env.example`). mise loads `.env`; PlatformIO injects each var as a `-D` flag.
Standard for adding a knob:

- **Name** it `LIBRA_<AREA>_<NAME>` (e.g. `LIBRA_THROTTLE_MAX`).
- **Default + document** it in `.env.example` with its value.
- **platformio.ini** build_flags: numbers `-DX=${sysenv.X}`; strings must be
  quote-wrapped `-DX='"${sysenv.X}"'` so they reach the compiler as literals.
- **src/config.h**: guard the default so a flag-less build still works, then bind
  a `constexpr` — this makes config.h the single source of the default:
  ```cpp
  #ifndef LIBRA_THROTTLE_MAX
  #define LIBRA_THROTTLE_MAX 0.05f
  #endif
  constexpr float kMaxThrottle = LIBRA_THROTTLE_MAX;
  ```
- **Never commit real values** — `.env` is gitignored; defaults live only in
  `config.h` + `.env.example`.

## Layout & the host/hardware split

Pure control logic in `lib/` is host-tested via the `native` env; hardware drivers are not.

- `lib/pid`, `lib/filter`, `lib/mixer` — **pure C++, no Arduino deps, host-tested. Keep them that way.**
- `lib/balancer` — composes filter+pid+mixer into the balancing policy (failsafe, re-arm reset). The control loop's logic lives here, not in `main`.
- `lib/imu`, `lib/esc` — hardware drivers (I2C / Servo). Arduino-only.
- `src/web` — WiFi SoftAP + HTTP server (ESP-IDF `httpd`) for the tuning web UI. Arduino-only. The `httpd` handlers run in their own task, so this is the **one** cross-context shared-state point: it's guarded by a small spinlock (`taskENTER_CRITICAL`), exchanged with the loop via `web::poll()` / `web::publish()`. The control loop stays the single owner of `gains`/`setpoint`.
- `src/main.cpp` — setup + a single fixed-rate `loop()`: poll serial (non-blocking), then each control step applies any pending web command → reads IMU → `balancer.step()` → drives ESCs → publishes telemetry. The control loop is the only thing that drives the ESCs.
- `src/config.h` — pins, limits, loop rate, gains, AP credentials.

The `native` env compiles only the libs a test includes — so **never `#include <Arduino.h>`** (or `Wire.h`, `ESP32Servo.h`, …) from `lib/pid`, `lib/filter`, `lib/mixer`.

## Conventions

- clang-format (Google base, 2-space, 120 cols); private members use a **leading** underscore (`_member`).
- The fixed-rate `loop()` must stay responsive — serial handling is non-blocking and must never delay a control step.

## Gotchas (append as you discover them)

- _(2026-06-20)_ The C3 Super Mini uses native USB-CDC for serial. The build flags `-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1` route `Serial` to the USB port; without them there's no serial over USB.
- _(2026-06-20)_ Tuning + telemetry are over serial **and** an optional WiFi web UI (`src/web`): an open SoftAP (no password) + `httpd` on :80 serving setpoint/gain sliders. The web UI never arms — arm/disarm is serial-only. There is still no camera. The WiFi stack roughly doubles flash use (~60%) and adds RAM; it preempts the loop on the single core, but the loop measures `dt` so it absorbs the jitter.
- _(2026-06-21)_ `mise run monitor` looking dead is **normal**: the firmware only prints on boot + on command, and native USB-CDC doesn't reset the chip on attach (so the banner's already gone). Type `?`, press RST, or use `mise run probe`/`banner`/`stream`. Debug verbosity is `LIBRA_LOG_LEVEL` → `CORE_DEBUG_LEVEL`; build with `>=4` to compile in the ~10 Hz raw-IMU `log_d` stream.
- _(2026-06-21)_ Upload failing with `Could not exclusively lock port … Resource temporarily unavailable` means a monitor holds `/dev/ttyACM0`. Close `mise run monitor`/`run` (or find the holder via `lsof`/`fuser`/`/proc/*/fd`), then re-upload.
- _(2026-06-21)_ `loop()` sleeps to the next control period via `vTaskDelayUntil` instead of busy-spinning — on the single core a tight no-yield loop starves the FreeRTOS IDLE task, tripping the task watchdog and squeezing WiFi/lwIP housekeeping (closed sockets, DHCP/TCP timers) → the AP/web UI gets unstable. Any future edit to `loop()` MUST keep a yield. Same reason `web::setHandler` parses query args *before* taking the `taskENTER_CRITICAL` spinlock (no string/float parsing with interrupts disabled on the WiFi core).
