# CLAUDE.md

Working guide for this repo. See `README.md` for the project overview + architecture diagrams.

## ⚠️ Safety — this firmware spins propellers (read first)

Physical-harm risk. These are hard guardrails, not suggestions:

- **NEVER** weaken, bypass, or remove the tilt failsafe, the master-enable gate, or the disarmed-on-boot default.
- **NEVER** raise thrust/output limits (`kPidOutLimit`, `kBaseThrottle`, ESC µs range) in `config.h` without explicit user confirmation.
- Assume props are on: any change that could command thrust must keep the safe-by-default path intact.

## Toolchain — always go through mise

PlatformIO lives in a Python venv managed by [mise](https://mise.jdx.dev/), which
also loads `.env`. Do **not** call `pio` directly — run the mise tasks:

- `mise run build` — compile (default env `esp32cam`)
- `mise run upload` / `mise run monitor` / `mise run run`
- `mise run test` — host unit tests (`pio test -e native`)
- `mise run format` / `mise run format-check` — clang-format
- `mise run hexdump` — embed `assets/*` as PROGMEM (web UI)

## Verification — gate, not optional

Before declaring any change done, these MUST pass:

- `mise run test` — host unit tests for the pure libs
- `mise run format-check` — clang-format clean

Add/extend Unity tests in `test/` when you touch a pure lib.

## Build-time config (.env → build flags)

Tunable constants come from `.env` (gitignored; `mise run setup` copies
`.env.example`). mise loads `.env`; PlatformIO injects each var as a `-D` flag.
Standard for adding a knob:

- **Name** it `LIBRA_<AREA>_<NAME>` (e.g. `LIBRA_THROTTLE_MAX`, `LIBRA_AP_SSID`).
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
- `src/main.cpp` — setup + the core-1 control task: read IMU → `balancer.step()` → drive ESCs.
- `src/control_state.{h,cpp}` — spinlock-guarded cross-core state. **All cross-core access goes through `state::` functions.**
- `src/camera.{h,cpp}`, `src/web.{h,cpp}` — core-0 camera + two `esp_http_server` instances (`:80` control, `:81` MJPEG).
- `src/config.h` — pins, limits, loop rate, gains, AP credentials.

The `native` env compiles only the libs a test includes — so **never `#include <Arduino.h>`** (or `Wire.h`, `ESP32Servo.h`, …) from `lib/pid`, `lib/filter`, `lib/mixer`.

## Conventions

- clang-format (Google base, 2-space, 120 cols); private members use a **leading** underscore (`_member`).
- The control loop must **never block** on WiFi/serial — the network path is tuning/telemetry only.

## Gotchas (append as you discover them)

- **LEDC timer split (don't break this):** camera XCLK and ESP32Servo share LEDC. Camera owns timer 0; `EscPair` allocates only timers 2 & 3; `cameraInit()` must run **before** `escs.begin()` so the camera claims timer 0 first. Reordering breaks the camera.
- _(2026-06-20)_ ESP32-CAM has no native USB — flash/log via USB-TTL/FTDI (GPIO0→GND for the bootloader).
