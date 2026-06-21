#!/usr/bin/env python3
"""Fine ESC throttle sweep for bench characterization (PROPS OFF).

Drives the firmware's bench commands over serial, stepping the throttle and holding each
level so you can watch the motor and read off the printed throttle / pulse width. Use it to
find a motor's spin-start, check the usable band, and confirm both motors respond.

Requires a firmware build with the bench commands compiled in (LIBRA_BENCH=1; see .env). It
opens the serial port (which resets the board), waits for the ESC to re-arm, sweeps, then
returns the motors to idle.

Examples:
  python tools/sweep.py                               # 0.00->0.10, 0.005 step, both motors
  python tools/sweep.py 0.07 0.10 0.001 2             # narrow + fine, 2 s holds, both
  python tools/sweep.py 0.07 0.10 0.001 2 --motor 1   # left (m1 / ESC1) only
  python tools/sweep.py 0.07 0.10 0.001 2 --motor 2   # right (m2 / ESC2) only
  python tools/sweep.py 0.07 0.10 0.001 2 --motor seq # left, then right (test both)
  python tools/sweep.py --port /dev/ttyACM0           # C3 native-USB port
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser(description="Fine ESC throttle sweep (props off).")
    ap.add_argument("start", nargs="?", type=float, default=0.0, help="start throttle 0..1 (default 0.0)")
    ap.add_argument("stop", nargs="?", type=float, default=0.10, help="stop throttle 0..1 (default 0.10)")
    ap.add_argument("step", nargs="?", type=float, default=0.005, help="throttle step (default 0.005)")
    ap.add_argument("hold", nargs="?", type=float, default=3.0, help="seconds held per step (default 3)")
    ap.add_argument("--motor", choices=["both", "1", "2", "seq"], default="both",
                    help="both (default) | 1 (left/m1) | 2 (right/m2) | seq (1 then 2)")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="serial port (default /dev/ttyUSB0; C3 = /dev/ttyACM0)")
    args = ap.parse_args()

    s = serial.Serial(args.port, 115200, timeout=0.2)
    s.dtr = False  # don't hold the board in reset
    s.rts = False
    print(f"opened {args.port}; sweep {args.start:.4f}->{args.stop:.4f} step {args.step:.4f}, "
          f"hold {args.hold:.0f}s, motor={args.motor}", flush=True)
    print("waiting 5 s for boot + ESC arm...", flush=True)
    time.sleep(5.0)

    def send(cmd, wait):
        s.reset_input_buffer()
        s.write((cmd + "\n").encode())
        buf = b""
        end = time.time() + wait
        while time.time() < end:
            chunk = s.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                txt = line.decode(errors="replace").rstrip("\r")
                if txt:
                    print(f"  >> {txt}", flush=True)

    def sweep_one(label, drive_m1, drive_m2):
        v = args.start
        while v <= args.stop + 1e-9:
            m1 = v if drive_m1 else 0.0
            m2 = v if drive_m2 else 0.0
            us = 1000 + v * 1000
            print(f"\n>>> {label}: throttle {v:.4f} (~{us:.2f} us) — hold {args.hold:.0f}s — WATCH THE MOTOR",
                  flush=True)
            send(f"set motors_speed {m1:.4f} {m2:.4f}", args.hold)
            v += args.step

    send("set motors_enabled on", 1.5)
    if args.motor in ("both",):
        sweep_one("BOTH", True, True)
    elif args.motor == "1":
        sweep_one("M1 (left)", True, False)
    elif args.motor == "2":
        sweep_one("M2 (right)", False, True)
    elif args.motor == "seq":
        sweep_one("M1 (left)", True, False)
        send("set motors_speed 0 0", 1.0)
        sweep_one("M2 (right)", False, True)

    print("\n=== back to idle ===", flush=True)
    send("set motors_speed 0 0", 1.0)
    send("set motors_enabled off", 1.0)
    s.close()
    print("done — motors idle.", flush=True)


if __name__ == "__main__":
    main()
