#!/usr/bin/env python3
"""
telemetry.py — read the obstacle-avoiding robot's UART telemetry stream.

The firmware (src/main.c, v3) emits one line every ~100 ms on RC6 at 9600 8N1:

    t=12345 st=0 pz=0 d=37 L=0 R=0 vL=90 vR=90

    t   milliseconds since boot        st  state 0..3 (see STATE_NAMES)
    pz  1 = paused                     d   ultrasonic distance, cm
    L/R left / right IR blocked (1)    vL/vR commanded wheel speed, %

Usage:
    python telemetry.py --port COM5
    python telemetry.py --port /dev/ttyUSB0 --csv run.csv
    python telemetry.py --port COM5 --plot

While running, type a line and press Enter to send it to the robot:
    p   toggle pause        ?   force an immediate status frame        q   quit
"""

import argparse
import csv
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed.  ->  pip install pyserial")

STATE_NAMES = {0: "CRUISE", 1: "BACK", 2: "TURN", 3: "FAULT"}
FIELDS = ("t", "st", "pz", "d", "L", "R", "vL", "vR")


def parse_line(line):
    """'t=1 st=0 d=37 ...' -> dict of ints, or None if it doesn't look like a frame."""
    out = {}
    for token in line.split():
        key, _, val = token.partition("=")
        if key in FIELDS and val.lstrip("-").isdigit():
            out[key] = int(val)
    return out if "t" in out and "st" in out else None


def reader(port, csv_writer, plot_buf, stop):
    last_t = None
    while not stop.is_set():
        raw = port.readline()
        if not raw:
            continue
        frame = parse_line(raw.decode("ascii", "replace").strip())
        if frame is None:
            continue

        st = frame.get("st", -1)
        gap = "" if last_t is None else f"{frame['t'] - last_t:+d}ms"
        last_t = frame["t"]
        print(
            f"{frame['t']:>8}  {STATE_NAMES.get(st, st):<7}"
            f"{'PAUSED ' if frame.get('pz') else '       '}"
            f"d={frame.get('d', '?'):>4}cm  "
            f"IR L{frame.get('L', '?')} R{frame.get('R', '?')}  "
            f"v {frame.get('vL', '?'):>3}/{frame.get('vR', '?'):<3}  {gap}"
        )

        if csv_writer:
            csv_writer.writerow([frame.get(k, "") for k in FIELDS])

        if plot_buf is not None:
            plot_buf.append((frame["t"] / 1000.0, frame.get("d", 0), st))
            del plot_buf[:-600]  # keep ~60 s


def live_plot(plot_buf, stop):
    import matplotlib.pyplot as plt

    fig, ax_d = plt.subplots(figsize=(9, 4))
    ax_s = ax_d.twinx()
    ax_d.set_xlabel("t (s)")
    ax_d.set_ylabel("distance (cm)")
    ax_s.set_ylabel("state")
    ax_s.set_yticks(list(STATE_NAMES))
    ax_s.set_yticklabels(list(STATE_NAMES.values()))
    ax_d.axhline(20, ls="--", lw=0.8, color="grey")  # threshold

    while not stop.is_set():
        if plot_buf:
            ts, ds, ss = zip(*plot_buf)
            ax_d.lines.clear()
            ax_s.lines.clear()
            ax_d.axhline(20, ls="--", lw=0.8, color="grey")
            ax_d.plot(ts, ds, color="tab:blue")
            ax_s.step(ts, ss, color="tab:red", where="post", alpha=0.6)
            ax_d.relim(); ax_d.autoscale_view()
            ax_d.set_xlim(max(0, ts[-1] - 60), ts[-1] + 1)
        plt.pause(0.2)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--csv", metavar="PATH", help="append frames to this CSV file")
    ap.add_argument("--plot", action="store_true", help="live matplotlib plot")
    args = ap.parse_args()

    port = serial.Serial(args.port, args.baud, timeout=1)
    print(f"listening on {args.port} @ {args.baud}   (Ctrl+C to stop)\n")

    csv_file = csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            csv_writer.writerow(FIELDS)

    plot_buf = [] if args.plot else None
    stop = threading.Event()
    th = threading.Thread(target=reader, args=(port, csv_writer, plot_buf, stop), daemon=True)
    th.start()

    try:
        if args.plot:
            live_plot(plot_buf, stop)
        else:
            for cmd in iter(sys.stdin.readline, ""):
                cmd = cmd.strip()
                if cmd == "q":
                    break
                if cmd:
                    port.write(cmd[:1].encode("ascii"))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        th.join(timeout=2)
        port.close()
        if csv_file:
            csv_file.close()
        print("\nstopped.")


if __name__ == "__main__":
    main()
