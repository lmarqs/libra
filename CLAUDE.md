# CLAUDE.md

Guidance for working in this repo. See `README.md` for the project overview.

## What this is

`libra` is firmware for a single-axis self-balancing beam (ESP32-CAM + MPU6050 +
two propeller-driven ESCs), built to experiment with PID control. The MCU sits
at the pivot and streams an FPV camera view with a telemetry OSD to the web UI.

The ESP32-CAM is dual-core: the PID control loop is pinned to one core so the
camera stream + web server (on the other core) can never starve it. It has no
native USB — flash + log via an external USB-TTL/FTDI adapter (GPIO0→GND for the
bootloader).

## Toolchain — always go through mise

PlatformIO lives in a Python venv managed by [mise](https://mise.jdx.dev/),
which also loads `.env`. Do **not** call `pio` directly — run the mise tasks so
the venv and env vars are present:

- `mise run build` — compile (default env `esp32-c3-super-mini`)
- `mise run upload` / `mise run monitor` / `mise run run`
- `mise run test` — host unit tests (`pio test -e native`)
- `mise run format` / `mise run format-check` — clang-format
- `mise run hexdump` — embed `assets/*` as PROGMEM (web UI; no-op until `assets/` exists)

## Layout & the host/hardware split

Pure control logic lives in `lib/` and is unit-tested on the host via the
`native` env; hardware drivers and wiring are not host-testable.

- `lib/pid`, `lib/filter`, `lib/mixer` — **pure C++, no Arduino deps, host-tested.** Keep them that way.
- `lib/imu`, `lib/esc` — hardware drivers (I2C / Servo). Arduino-only.
- `src/main.cpp` — setup + the core-1 control task (loop + failsafe + master enable).
- `src/control_state.{h,cpp}` — spinlock-guarded state shared between the control task (core 1) and web/serial (core 0). All cross-core access goes through `state::` functions.
- `src/camera.{h,cpp}` — ESP32-CAM (OV2640) init for the MJPEG stream.
- `src/web.{h,cpp}` — two `esp_http_server` instances on core 0: `:80` page/telemetry/set, `:81` MJPEG stream.
- `src/config.h` — pins, limits, loop rate, gains, AP credentials.
- `assets/index.html` — the web UI; embedded as PROGMEM by `mise run hexdump` and `#include`d by web.cpp.
- `test/` — Unity tests for the pure libs.

The `native` env excludes `src/` and, via the library dependency finder, only
compiles the libs a test actually includes — so never `#include <Arduino.h>`
(or `Wire.h`, `ESP32Servo.h`, …) from `lib/pid`, `lib/filter`, or `lib/mixer`.

### LEDC timer split (don't break this)

Both the camera's XCLK and ESP32Servo use the LEDC peripheral. The camera owns
timer 0; `EscPair` allocates only timers 2 & 3; and `cameraInit()` must run
before `escs.begin()` in setup so the camera claims timer 0 first. Reordering or
re-allocating all four timers breaks the camera.

## Conventions

- C++ formatted with clang-format (Google base, 2-space indent, 120 cols). Run `mise run format` before committing.
- The control loop must never block on WiFi/serial; the network path is for tuning/telemetry only.
- Safety first: boots disarmed, tilt failsafe cuts thrust, master-enable gates the motors.
