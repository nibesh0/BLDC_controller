#!/usr/bin/env python3
"""Record the BLDC UART log stream (see log.c) to CSV, optionally live-plot it.

    pip install pyserial==3.5
    python tools/pi_logger.py COM8
    python tools/pi_logger.py COM8 --out logs/test3.csv --live
    python tools/pi_logger.py COM8 --live --window 5 --fc 5 --fill-ms 20
    python tools/pi_logger.py COM8 --seconds 30 --out logs/step_300rpm.csv

Columns come from the target's descriptor frame, so channels added to
chTable[] in log.c show up here (and in the live plot) without edits.

--live shows the same plot as plot_log.py (same groups, low-pass, speed
sign, time axis from 0 s) as a scrolling window of the last --window
seconds. Recording runs in its own thread, so the CSV is complete even if
the plot redraws slowly. Stop with Ctrl+C or by closing the plot window.
"""

import argparse
import collections
import csv
import datetime
import os
import signal
import struct
import sys
import threading
import time

import serial

SYNC = b"\xA5\x5A"
FRAME_DESC = 0x44
FRAME_DATA = 0x53
MAX_LEN = 2048

# type code -> (struct format, csv formatter); must match log.c
TYPES = {
    0: ("H", str),
    1: ("h", str),
    2: ("I", str),
    3: ("i", str),
    4: ("f", lambda v: "%.7g" % v),
}


class FrameParser:
    """Splits the byte stream into checked (type, payload) frames."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data

    def frames(self):
        buf = self.buf
        while True:
            i = buf.find(SYNC)
            if i < 0:
                del buf[:max(0, len(buf) - 1)]  # keep a possible A5
                return
            if i:
                del buf[:i]
            if len(buf) < 5:
                return
            ftype = buf[2]
            length = buf[3] | (buf[4] << 8)
            if ftype not in (FRAME_DESC, FRAME_DATA) or length > MAX_LEN:
                self.bad += 1
                del buf[:1]
                continue
            end = 5 + length
            if len(buf) < end + 1:
                return
            if (sum(buf[2:end]) & 0xFF) != buf[end]:
                self.bad += 1
                del buf[:1]
                continue
            payload = bytes(buf[5:end])
            del buf[:end + 1]
            yield ftype, payload


def parse_desc(p):
    ver, nch = p[0], p[1]
    tick_hz, div = struct.unpack_from("<IH", p, 2)
    pos = 8
    chans = []
    for _ in range(nch):
        ctype, nlen = p[pos], p[pos + 1]
        pos += 2
        name = p[pos:pos + nlen].decode("ascii", "replace")
        pos += nlen
        if ctype not in TYPES:
            raise ValueError("unknown channel type %d for %s" % (ctype, name))
        chans.append((name, ctype))
    return ver, tick_hz, div, chans


class CsvLog:
    """One CSV file per channel layout."""

    def __init__(self, path, tick_hz, chans):
        self.tick_hz = tick_hz
        self.chans = chans
        self.fmt = struct.Struct("<HI" + "".join(TYPES[t][0] for _, t in chans))
        self.conv = [TYPES[t][1] for _, t in chans]
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        self.path = path
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.columns = ["time_s", "seq", "tick"] + [n for n, _ in chans]
        self.w.writerow(self.columns)
        self.rows = 0
        self.lost = 0
        self.tick0 = None
        self.tick_wrap = 0
        self.last_tick = None
        self.last_seq = None

    def add(self, payload):
        """Write one sample. Returns (time_s, seq, tick, values...) or None."""
        if len(payload) != self.fmt.size:
            return None
        vals = self.fmt.unpack(payload)
        seq, tick = vals[0], vals[1]

        if self.last_seq is not None:
            self.lost += (seq - self.last_seq - 1) & 0xFFFF
        self.last_seq = seq

        if self.last_tick is not None and tick < self.last_tick:
            self.tick_wrap += 1 << 32
        self.last_tick = tick
        tick_abs = tick + self.tick_wrap
        if self.tick0 is None:
            self.tick0 = tick_abs
        t = (tick_abs - self.tick0) / self.tick_hz   # 0 s at first sample

        row = ["%.5f" % t, seq, tick]
        row += [c(v) for c, v in zip(self.conv, vals[2:])]
        self.w.writerow(row)
        self.rows += 1
        return (t,) + vals

    def close(self):
        self.f.flush()
        self.f.close()


def default_path():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join("logs", "pi_%s.csv" % stamp)


class Recorder(threading.Thread):
    """Serial -> CSV. Also keeps the latest samples for the live plot."""

    def __init__(self, ser, base, seconds, live_rows):
        super().__init__(daemon=True)
        self.ser = ser
        self.base = base
        self.seconds = seconds
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.samples = collections.deque(maxlen=max(1, live_rows))
        self.columns = None         # live plot column names
        self.layout_id = 0          # bumps when the channel layout changes
        self.log = None
        self.parser = FrameParser()
        self.bad_size = 0
        self.error = None

    def run(self):
        try:
            self._loop()
        except Exception as e:      # report in main thread
            self.error = e
        finally:
            self.stop.set()
            self.ser.close()
            if self.log:
                self.log.close()
                print("Saved %d rows to %s (lost %d, bad %d)"
                      % (self.log.rows, self.log.path, self.log.lost,
                         self.parser.bad + self.bad_size))

    def _new_log(self, tick_hz, div, chans, part):
        path = self.base
        if part:
            root, ext = os.path.splitext(self.base)
            path = "%s_%d%s" % (root, part, ext or ".csv")
        self.log = CsvLog(path, tick_hz, chans)
        with self.lock:
            self.samples.clear()
            self.columns = list(self.log.columns)
            self.layout_id += 1
        print("Writing %s: %d channels, %.0f Hz sample rate"
              % (path, len(chans), tick_hz / max(1, div)))

    def _loop(self):
        ser, parser = self.ser, self.parser
        layout = None
        part = 0
        t_start = time.monotonic()
        t_status = t_start
        rows_status = 0

        while not self.stop.is_set():
            parser.feed(ser.read(max(1, ser.in_waiting)))

            for ftype, payload in parser.frames():
                if ftype == FRAME_DESC:
                    try:
                        _, tick_hz, div, chans = parse_desc(payload)
                    except (ValueError, IndexError, struct.error) as e:
                        print("Bad descriptor: %s" % e)
                        continue
                    if (tick_hz, chans) != layout:
                        # New layout (first start or new firmware): new file
                        if self.log:
                            self.log.close()
                            print("Layout changed, closed %s" % self.log.path)
                        layout = (tick_hz, chans)
                        self._new_log(tick_hz, div, chans, part)
                        part += 1
                elif self.log is not None:
                    row = self.log.add(payload)
                    if row is None:
                        self.bad_size += 1
                    else:
                        with self.lock:
                            self.samples.append(row)

            now = time.monotonic()
            if now - t_status >= 1.0:
                if self.log:
                    rate = (self.log.rows - rows_status) / (now - t_status)
                    rows_status = self.log.rows
                    self.log.f.flush()
                    print("rows %d  %.0f/s  lost %d  bad %d"
                          % (self.log.rows, rate, self.log.lost,
                             parser.bad + self.bad_size))
                else:
                    print("no descriptor yet (bytes in buffer: %d)"
                          % len(parser.buf))
                t_status = now

            if self.seconds > 0 and now - t_start >= self.seconds:
                break

    def snapshot(self):
        with self.lock:
            return self.layout_id, self.columns, list(self.samples)


class LivePlot:
    """Scrolling plot with the same processing as plot_log.py."""

    def __init__(self, rec, args):
        import matplotlib.pyplot as plt
        import numpy as np
        import pandas as pd
        import plot_log as P

        self.plt, self.np, self.pd, self.P = plt, np, pd, P
        self.rec = rec
        self.args = args
        self.layout_id = -1
        self.lines = []
        self.axes = []
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.canvas.mpl_connect("close_event", lambda e: rec.stop.set())
        self.title = self.fig.suptitle("waiting for descriptor...")

    def _build(self, columns):
        P, args = self.P, self.args
        filt = (P.FILTERED & set(columns)) if args.fc > 0 else set()
        names = list(columns) + [c + " lpf" for c in sorted(filt)]
        if "piCount" in columns:
            names.append("piCount diff")
        groups = P.build_groups(names)

        self.fig.clf()
        self.title = self.fig.suptitle("")
        axes = self.fig.subplots(len(groups), 1, sharex=True, squeeze=False)
        self.axes = axes[:, 0]
        self.lines = []
        for ax, (title, cols) in zip(self.axes, groups):
            for c in cols:
                if c in P.STEP:
                    ln, = ax.plot([], [], label=c, lw=1, drawstyle="steps-post")
                    self.lines.append((ln, c))
                elif c in filt:
                    ln, = ax.plot([], [], label=c + " (lpf)", lw=1.3)
                    self.lines.append((ln, c + " lpf"))
                    if not args.no_raw  and 0:
                        raw, = ax.plot([], [], color=ln.get_color(), lw=0.5,
                                       alpha=0.25, label="_nolegend_")
                        self.lines.append((raw, c))
                else:
                    ln, = ax.plot([], [], label=c, lw=1.0)
                    self.lines.append((ln, c))
            P.style_axis(ax, title)
        P.style_time_axis(self.axes[-1], 0.0, args.window)
        self.fig.subplots_adjust(left=0.07, right=0.99, top=0.95, bottom=0.05,
                                 hspace=0.15)

    def update(self, _frame=None):
        rec, args, np, pd = self.rec, self.args, self.np, self.pd
        if rec.stop.is_set():
            self.plt.close(self.fig)
            return

        layout_id, columns, rows = rec.snapshot()
        if columns is None:
            return
        if layout_id != self.layout_id:
            self._build(columns)
            self.layout_id = layout_id
        if len(rows) < 2:
            self.title.set_text("%s   waiting for data..." % rec.log.path)
            return

        d = pd.DataFrame(np.asarray(rows, dtype=float), columns=columns)
        t_last = float(d.time_s.iloc[-1])
        x0 = max(0.0, t_last - args.window)          # axis starts at 0 s
        x1 = x0 + args.window
        d = d[d.time_s >= x0]
        try:
            d, _, notes = self.P.process(d, args.fc, args.fill_ms, args.dir)
        except ValueError as e:
            self.title.set_text(str(e))
            return

        t = d.time_s.to_numpy()
        for ln, c in self.lines:
            if c in d:
                ln.set_data(t, d[c].to_numpy())
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self.axes[-1].set_xlim(x0, x1)

        log = rec.log
        self.title.set_text(
            "LIVE  %s   t %.1f s   rows %d   lost %d   bad %d   %s"
            % (os.path.basename(log.path), t_last, log.rows, log.lost,
               rec.parser.bad + rec.bad_size, ", ".join(notes)))

    def run(self):
        from matplotlib.animation import FuncAnimation
        self._anim = FuncAnimation(self.fig, self.update,
                                   interval=int(self.args.refresh * 1000),
                                   cache_frame_data=False)
        self.plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="'XDS100 Class USB Serial Port', e.g. COM8")
    ap.add_argument("--baud", type=int, default=625000, help="must match LOG_BAUD")
    ap.add_argument("--out", default=None, help="CSV path (default logs/pi_<time>.csv)")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after N s (0 = Ctrl+C)")
    live = ap.add_argument_group("live plot (same options as plot_log.py)")
    live.add_argument("--live", action="store_true", help="show scrolling live plot")
    live.add_argument("--window", type=float, default=10.0, help="visible time [s] (default 10)")
    live.add_argument("--refresh", type=float, default=0.5, help="redraw period [s] (default 0.5)")
    live.add_argument("--fc", type=float, default=10.0, help="low-pass cutoff [Hz], 0 = off (default 10)")
    live.add_argument("--fill-ms", type=float, default=0.0,
                      help="bridge speed dropouts (exact 0) up to this long, 0 = off")
    live.add_argument("--dir", choices=["pos", "neg", "raw"], default="pos",
                      help="pos = all speed lines positive (default), neg, raw = as logged")
    live.add_argument("--no-raw", action="store_true", help="hide raw trace of filtered channels")
    args = ap.parse_args()

    if args.live:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        try:
            import matplotlib  # noqa: F401
            import plot_log    # noqa: F401
        except ImportError as e:
            sys.exit("--live needs matplotlib, pandas, scipy: %s" % e)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit("Cannot open %s: %s" % (args.port, e))
    try:
        ser.set_buffer_size(rx_size=1 << 20)   # Windows: ride out slow redraws
    except (AttributeError, serial.SerialException):
        pass

    print("Listening on %s @ %d baud, waiting for descriptor..." % (args.port, args.baud))
    ser.reset_input_buffer()

    # Enough rows for the window at up to 2 kHz sample rate
    rec = Recorder(ser, args.out or default_path(), args.seconds,
                   int(args.window * 2000) + 100)
    # Ctrl+C just requests a clean stop (also works while the plot is open)
    signal.signal(signal.SIGINT, lambda *_: rec.stop.set())
    rec.start()

    if args.live:
        LivePlot(rec, args).run()   # returns when the window is closed
        rec.stop.set()
    else:
        while rec.is_alive():
            rec.join(0.2)

    rec.stop.set()
    rec.join()
    if rec.error:
        sys.exit("Logger error: %s" % rec.error)


if __name__ == "__main__":
    main()
