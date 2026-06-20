# CLAUDE.md

Working guide for this repo. See `README.md` for the project overview + architecture diagrams.

## ⚠️ Safety — this firmware spins propellers (read first)

Physical-harm risk. These are hard guardrails, not suggestions:

- **NEVER** weaken, bypass, or remove the tilt failsafe, the master-enable gate, or the disarmed-on-boot default.
- **NEVER** raise thrust/output limits (`kMaxThrottle`, `kPidOutLimit`, `kBaseThrottle`, ESC µs range) in `config.h` without explicit user confirmation.
- Assume props are on: any change that could command thrust must keep the safe-by-default path intact.

## Toolchain — always go through mise

PlatformIO lives in a Python venv managed by [mise](https://mise.jdx.dev/), which
also loads `.env`. Do **not** call `pio` directly — run the mise tasks:

- `mise run build` — compile (default env `esp32-c3-super-mini`)
- `mise run upload` / `mise run monitor` / `mise run run`
- `mise run test` — host unit tests (`pio test -e native`)
- `mise run format` / `mise run format-check` — clang-format

## Verification — gate, not optional

Before declaring any change done, these MUST pass:

- `mise run test` — host unit tests for the pure libs
- `mise run format-check` — clang-format clean

Add/extend Unity tests in `test/` when you touch a pure lib.

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
- `src/main.cpp` — setup + a single fixed-rate `loop()`: poll serial, then read IMU → `balancer.step()` → drive ESCs. The C3 is single-core, so there's one execution context — no tasks, no shared-state locking.
- `src/config.h` — pins, limits, loop rate, gains.

The `native` env compiles only the libs a test includes — so **never `#include <Arduino.h>`** (or `Wire.h`, `ESP32Servo.h`, …) from `lib/pid`, `lib/filter`, `lib/mixer`.

## Conventions

- clang-format (Google base, 2-space, 120 cols); private members use a **leading** underscore (`_member`).
- The fixed-rate `loop()` must stay responsive — serial handling is non-blocking and must never delay a control step.

## Gotchas (append as you discover them)

- _(2026-06-20)_ The C3 Super Mini uses native USB-CDC for serial. The build flags `-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1` route `Serial` to the USB port; without them there's no serial over USB.
- _(2026-06-20)_ Tuning + telemetry are serial-only — there is no WiFi, web server, or camera. Don't reach for those; the control path is local.
