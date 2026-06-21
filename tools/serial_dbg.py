#!/usr/bin/env python3
"""Non-interactive serial debugging for the Libra firmware (ESP32-C3).

The firmware prints a boot banner once and otherwise only replies to commands,
and the C3's native USB-CDC does not reset the chip when a monitor attaches — so
`pio device monitor` usually shows a blank screen. These subcommands talk to the
board non-interactively instead, which is scriptable and friendly to CI, an agent,
or a quick `mise run probe`.

Subcommands:
  probe    send one command (default '?') and print the reply
  banner   reset the board and capture its boot banner
  stream   capture serial output for N seconds (e.g. the debug IMU stream)

Run through mise so the PlatformIO venv's pyserial is used:
  mise run probe
  mise run banner
  mise run stream
  mise x -- python tools/serial_dbg.py probe --cmd "kp 0.02"
  mise x -- python tools/serial_dbg.py stream --seconds 20 --grep "gx="

See docs/testing.md for the full workflow.
"""
import argparse
import sys
import time

try:
    import serial  # pyserial — bundled with PlatformIO
except ImportError:
    sys.exit("pyserial not found — run through mise (e.g. `mise run probe`).")

DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200


def open_port(port):
    try:
        return serial.Serial(port, BAUD, timeout=0.3)
    except serial.SerialException as e:
        sys.exit(
            f"could not open {port}: {e}\n"
            f"If it is busy, a monitor likely holds it — close `mise run monitor`/`run`, "
            f"or find the holder: `lsof {port}`, `fuser {port}`, or scan /proc/*/fd."
        )


def reset_into_app(s):
    """Pulse EN (RTS) with BOOT (DTR) released so the C3 reboots into the app,
    not the bootloader. Works over the built-in USB-Serial/JTAG."""
    s.dtr = False
    s.rts = False
    time.sleep(0.1)
    s.rts = True  # EN low -> reset
    time.sleep(0.15)
    s.rts = False  # release -> run the app


def read_lines(s, seconds, on_line):
    end = time.time() + seconds
    buf = b""
    while time.time() < end:
        chunk = s.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            on_line(line.decode(errors="replace").rstrip("\r"))


def cmd_probe(args):
    s = open_port(args.port)
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write((args.cmd + "\n").encode())
    seen = []
    read_lines(s, args.seconds, lambda ln: (print(ln), seen.append(ln)))
    s.close()
    if not seen:
        print("(no reply — board may be mid-arm, in the bootloader, or not running the app)")


def cmd_banner(args):
    s = open_port(args.port)
    reset_into_app(s)
    read_lines(s, args.seconds, print)
    s.close()


def cmd_stream(args):
    s = open_port(args.port)
    t0 = time.time()

    def show(ln):
        if args.grep and args.grep not in ln:
            return
        print(f"[t={time.time() - t0:5.1f}] {ln}")

    read_lines(s, args.seconds, show)
    s.close()


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", default=DEFAULT_PORT, help=f"serial port (default {DEFAULT_PORT})")
    sub = p.add_subparsers(dest="sub", required=True)

    pp = sub.add_parser("probe", help="send a command and print the reply")
    pp.add_argument("--cmd", default="?", help="command to send (default '?')")
    pp.add_argument("--seconds", type=float, default=2.0, help="how long to read the reply")
    pp.set_defaults(func=cmd_probe)

    pb = sub.add_parser("banner", help="reset the board and capture the boot banner")
    pb.add_argument("--seconds", type=float, default=7.0, help="capture window (covers the ~3s ESC arm)")
    pb.set_defaults(func=cmd_banner)

    ps = sub.add_parser("stream", help="capture serial output for N seconds")
    ps.add_argument("--seconds", type=float, default=30.0, help="capture window")
    ps.add_argument("--grep", default=None, help="only print lines containing this substring (e.g. 'gx=')")
    ps.set_defaults(func=cmd_stream)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
