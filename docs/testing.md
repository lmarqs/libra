# Testing & debugging

How to test and debug Libra effectively — host tests, build/flash, talking to the
board, debug logging, and sensor bring-up. The fast loop is host unit tests; the
board is reached over serial, non-interactively.

## TL;DR — the loop

```sh
mise run test          # host unit tests (pure libs) — fast, no hardware
mise run build         # compile the default env (esp32-wroom32)
mise run build:all     # compile BOTH boards — the dual-arch gate
mise run upload        # flash over USB
mise run probe         # ask the board its state ('?') and print the reply
mise run banner        # reset + capture the boot banner
mise run stream        # capture serial for 30s (e.g. the debug IMU stream)
mise run format-check  # clang-format clean
```

Gate before declaring a change done: **`mise run test` + `mise run format-check`**.

## 1. Host unit tests — the primary correctness loop

The pure control libs (`lib/{pid,filter,mixer,balancer}`) have **no Arduino deps**
and are unit-tested on the host via the PlatformIO `native` env:

```sh
mise run test          # Unity tests; currently 21 cases
```

This needs no board and runs in seconds — do most logic work here. **Add or extend
Unity tests in `test/` whenever you touch a pure lib.** Hardware drivers (`imu`,
`esc`, `web`) are deliberately *not* host-compiled, so keep Arduino/`Wire`/WiFi
includes out of the pure libs (the `native` env would fail to build them).

## 2. Build & flash

Two boards build from one tree (per-board pin map + flags live in `platformio.ini`):
the **ESP32-C3 Super Mini** (single-core, native USB) and the **ESP32 WROOM-32**
(dual-core, USB-UART bridge). `mise run build` compiles the default env
(`esp32-wroom32`); target a specific board by passing the env through mise:

```sh
mise run build                            # default env (esp32-wroom32)
mise run build:all                        # both boards — dual-arch compile gate
mise x -- pio run -e esp32-c3-super-mini  # just the C3
mise run upload                           # build + flash (default env)
mise run run                              # build + upload + monitor, in order
```

If `upload` can't connect on the C3, enter download mode once: hold **BOOT**, tap
**RST**, release BOOT, then upload. USB-CDC auto-reset handles it afterwards.

### Concurrency / core model (both archs)

The control loop runs in a dedicated `controlTask` pinned to `ARDUINO_RUNNING_CORE`:
the **APP core (1)** on the dual-core WROOM-32 (isolated from WiFi/`httpd` on the PRO
core 0), and the **only core (0)** on the single-core C3 (where its priority `10` —
above `httpd`, below lwIP/WiFi — plus the per-period `xTaskDelayUntil` yield keep WiFi
alive). A `LIBRA_LOG_LEVEL>=4` build logs the task's core and stack high-water-mark, so
you can confirm `core=1` on the WROOM / `core=0` on the C3 and that the 8 KB stack has
headroom. See CLAUDE.md → Gotchas for the full rationale and the watchdog invariant.

## 3. Talking to the board over serial

### Why the monitor looks dead

The firmware prints a boot banner **once** and then only replies to commands — it
does not stream by default. And the C3's native USB-CDC does **not** reset the chip
when a monitor attaches (unlike a UART bridge), so the banner has already scrolled
past. Result: `mise run monitor` shows a blank screen until you type. That's
expected, not a fault. Type `?` for a state line, or press **RST** to re-emit the
banner.

### Non-interactive helpers (preferred)

`tools/serial_dbg.py` (wrapped by mise) talks to the board without a live terminal —
scriptable, CI/agent-friendly, and it resets the board cleanly (pulses EN via RTS
into the **app**, not the bootloader):

```sh
mise run probe                                           # send '?', print the state line
mise x -- python tools/serial_dbg.py probe --cmd "kp 0.02"   # send any command
mise run banner                                          # reset + capture the boot banner
mise run stream                                          # capture 30s of serial
mise x -- python tools/serial_dbg.py stream --seconds 20 --grep "gx="  # filter lines
```

A typical `probe` reply (all serial output now goes through the ESP32 `log_*` API, so each
line carries a `[time][level][file:line] func():` prefix — the `state:` fields follow it):

```
[ 12345][I][main.cpp:106] handleCommand(): state: OFF angle=-0.24 sp=0.00 out=0.000 m1=0.00 m2=0.00 kp=0.0100 ki=0.0000 kd=0.0008
```

The state line is `log_i`, so `probe`/`banner` need `LIBRA_LOG_LEVEL >= 3` (the default) to
see it; below 3 only errors/warnings print.

### Serial command reference

| Command | Effect |
|---|---|
| `e` | enable (arm the control loop) |
| `d` / `x` | disable (`x` is the emergency-stop alias) |
| `kp <v>` / `ki <v>` / `kd <v>` | set a PID gain live |
| `sp <v>` | set the target tilt, degrees |
| `?` | print state (angle, output, throttles, gains) |

Setpoint and gains are also settable from the WiFi web UI; **arming is serial-only.**

## 4. Debug logging — env-var log level

**All** serial output goes through the ESP32 `log_*` API (`CORE_DEBUG_LEVEL`), driven by
`LIBRA_LOG_LEVEL` in `.env` (0=none, 1=error, 2=warn, 3=info, 4=debug, 5=verbose). So the
level gates everything, not just the IMU stream — each line is prefixed
`[time][level][file:line] func():`:

- **≥ 1 (error)** failures: `MPU6050 not found`, `WiFi SoftAP failed`, `HTTP server failed`.
- **≥ 2 (warn)** invalid-command help.
- **≥ 3 (info)** the boot banner, arm/disarm + `state:` replies, and AP join/leave events.
  This is why `probe`/`banner` need level ≥ 3 (the default).
- **≥ 4 (debug)** compiles in a ~10 Hz raw-IMU stream via `log_d()` — the bring-up tool.
  Below 4 it is compiled out entirely (zero runtime cost).
- `Serial.setDebugOutput(true)` routes `log_*()` to the USB-CDC — load-bearing for *all*
  output now, not just debug.
- Default is **3**; bump to 4, rebuild, and watch with `mise run stream` only while
  bringing up the IMU.

```sh
# enable the IMU stream
sed -i 's/^LIBRA_LOG_LEVEL=.*/LIBRA_LOG_LEVEL=4/' .env && mise run upload
mise run stream                       # lines look like: ... step(): ax=.. gx=.. acc=.. fused=..
```

Each line carries the raw accel (`ax/ay/az`, g), gyro (`gx/gy/gz`, °/s), the
accel-derived angle (`acc`), and the filtered angle (`fused`).

## 5. Sensor bring-up — IMU axis & zero-offset

The IMU mapping is mounting-specific. To (re-)derive it after a remount:

1. Build with `LIBRA_LOG_LEVEL=4`, flash, and `mise run stream`.
2. Move the beam: **hold level → rotate CW and hold → level → rotate CCW → level**,
   pausing between phases.
3. Read off the trace:
   - **Accel angle axis** — at level, see which accel axis carries ~+1 g. Gravity
     resting on +X with tilt swinging into Y means the angle is `atan2(ay, ax)`
     (set in `Imu::accelAngleDeg`).
   - **Pivot gyro axis + sign** — the gyro axis that spikes during rotation is the
     rate. Use it negated/un-negated so the **fused** angle tracks **acc** during
     motion (here `-s.gz`, in `main.cpp`). If fused lags or diverges from acc, flip
     the sign.
   - **Zero-offset** — read the steady angle at physical level and put it in
     `LIBRA_ANGLE_OFFSET_DEG` (`.env`) so level reads 0.

This rig's result: `atan2(ay, ax)`, rate `-s.gz`, offset `-3.4°`.

## 6. WSL2 serial notes

- Device: **`/dev/ttyACM0`** — the C3's built-in USB-Serial/JTAG (VID:PID
  `303A:1001`), shared into WSL via `usbipd-win`.
- **Port busy** (`Could not exclusively lock port … Resource temporarily
  unavailable`): a monitor holds it. Close `mise run monitor`/`run`, or find the
  holder: `lsof /dev/ttyACM0`, `fuser -v /dev/ttyACM0`, or scan
  `/proc/*/fd` for the symlink to `ttyACM0`.
- A reset re-enumerates USB; `usbipd` can detach the device — re-attach from
  Windows (`usbipd attach --wsl --busid <id>`) if `/dev/ttyACM0` disappears.

## 7. Safety while bench-testing

**Props off or beam clamped.** The firmware boots disarmed; arm only over serial
(`e`), and `x` is the emergency stop. The web UI can never arm. When first arming,
keep gains low and confirm the correction sign — if the beam drives *away* from
level, hit `x` and flip the output sign / motor wiring. Never raise `kMaxThrottle`,
`kPidOutLimit`, or the tilt limit without intent (see [CLAUDE.md](../CLAUDE.md)).
