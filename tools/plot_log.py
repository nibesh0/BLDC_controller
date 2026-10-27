#!/usr/bin/env python3
"""Plot every channel of a pi_logger.py CSV.

    python tools/plot_log.py tools/logs/test2.csv
    python tools/plot_log.py tools/logs/test2.csv --fc 5            # stronger LPF
    python tools/plot_log.py tools/logs/test2.csv --fc 0            # raw only
    python tools/plot_log.py tools/logs/test2.csv --t0 20 --t1 22   # zoom
    python tools/plot_log.py tools/logs/test2.csv --t0 20 --t1 22 --rel  # zoom, axis from 0 s
    python tools/plot_log.py tools/logs/test2.csv --fill-ms 20      # bridge hall dropouts
    python tools/plot_log.py tools/logs/test2.csv --save            # PNG only

Noisy analog channels (FILTERED) get a zero-phase low-pass (Butterworth,
forward-backward, so no time lag). Raw data stays visible as a faint line.
The filter runs on the whole log before zooming and restarts at lost-sample
gaps, so it never smears across missing data. Setpoints, gains and states
are never filtered.

Known channels are grouped; any new channel added to chTable[] gets its own
subplot automatically. Grey bands = samples lost on the link (seq gaps).

The processing helpers here (process, build_groups, style_*) are also used
by pi_logger.py --live, so both plots look the same.
"""

import argparse
import os
import time

import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator, MaxNLocator
import numpy as np
import pandas as pd
from scipy.signal import butter, sosfiltfilt

GROUPS = [
    ("Speed [rpm]",        ["rpmRef", "rpmRefNow", "piFb", "hallRpm"]),
    ("Error [rpm]",        ["rpmErr"]),
    ("PI terms [duty]",    ["piProp", "piInteg", "piOutRaw", "dutyNow"]),
    ("Kp",                 ["piKp"]),
    ("Ki",                 ["piKi"]),
    ("State",              ["motorState", "faultCode", "speedMode", "speedIdle"]),
    ("Comm step",          ["commStep"]),
]
STEP = {"motorState", "faultCode", "speedMode", "speedIdle", "commStep",
        "piKp", "piKi"}
SKIP = {"time_s", "seq", "tick", "piCount"}

# Channels that get the low-pass. Add new noisy channels here.
FILTERED = {"piFb", "hallRpm", "rpmErr", "piProp", "piOutRaw", "dutyNow",
            "piInteg"}
# Filtered channels drawn without their faint raw trace (filtered line only)
NO_RAW = {"rpmErr"}
# Speed feedback channels where an exact 0 can be a hall dropout
SPEED_FB = {"piFb", "hallRpm"}
# Magnitude-only speed channels; negated for --dir neg
SPEED_MAG = {"rpmRef", "rpmRefNow", "piFb"}

FILTER_ORDER = 2


def segments(d):
    """Index ranges of continuous data (split at seq gaps)."""
    if "seq" not in d:
        return [(0, len(d))]
    cut = np.flatnonzero(d.seq.diff().fillna(1).to_numpy() > 1)
    edges = [0] + cut.tolist() + [len(d)]
    return list(zip(edges[:-1], edges[1:]))


def lowpass(x, fc, fs, segs):
    """Zero-phase Butterworth low-pass, applied per continuous segment."""
    sos = butter(FILTER_ORDER, fc, btype="low", fs=fs, output="sos")
    pad = 3 * (2 * len(sos) + 1)
    y = x.astype(float).copy()
    for a, b in segs:
        if b - a > pad:
            y[a:b] = sosfiltfilt(sos, x[a:b])
    return y


def fill_dropouts(x, max_len):
    """Hold the last value through runs of exact 0 up to max_len samples.

    Longer zero runs are treated as real standstill and kept.
    """
    s = pd.Series(x, dtype=float)
    zero = s == 0
    run_id = (zero != zero.shift()).cumsum()
    run_len = zero.groupby(run_id).transform("size")
    short = zero & (run_len <= max_len)
    return s.mask(short).ffill().fillna(0.0).to_numpy()


def process(d, fc=10.0, fill_ms=0.0, direction="pos"):
    """Sign, dropout bridging, low-pass and derived columns.

    d needs time_s (and seq for gap handling), positional index.
    Returns (new frame, set of filtered channel names, notes for the title).
    Filtered values go to '<name> lpf'; raw columns keep their name.
    """
    d = d.reset_index(drop=True).copy()
    notes = []
    dt = float(d.time_s.diff().median()) if len(d) > 1 else 0.002
    if not dt > 0:
        dt = 0.002
    fs = 1.0 / dt

    # Plot-only sign: give all speed lines the same sign so they overlay
    if direction != "raw":
        want = -1.0 if direction == "neg" else 1.0
        if "hallRpm" in d:
            nz = d.hallRpm[d.hallRpm != 0]
            if len(nz) and np.sign(nz.median()) == -want:
                d["hallRpm"] = -d["hallRpm"]
        if want < 0:
            for c in SPEED_MAG & set(d.columns):
                d[c] = -d[c]
        notes.append("speed shown %s" % ("negative" if want < 0 else "positive"))

    if fill_ms > 0:
        n = max(1, int(round(fill_ms / (dt * 1000))))
        for c in SPEED_FB & set(d.columns):
            d[c] = fill_dropouts(d[c].to_numpy(), n)
        notes.append("dropouts <= %g ms bridged" % fill_ms)

    filt = set()
    if fc > 0:
        if fc >= fs / 2:
            raise ValueError("--fc must be below %.0f Hz (Nyquist)" % (fs / 2))
        segs = segments(d)
        for c in FILTERED & set(d.columns):
            d[c + " lpf"] = lowpass(d[c].to_numpy(), fc, fs, segs)
            filt.add(c)
        notes.append("LPF %g Hz" % fc)

    if "piCount" in d:
        d["piCount diff"] = d.piCount.diff()
    return d, filt, notes


def build_groups(columns):
    """Subplot list [(title, [columns])]; unknown channels get their own."""
    cols = set(columns)
    groups = [(title, [c for c in cs if c in cols]) for title, cs in GROUPS]
    groups = [g for g in groups if g[1]]
    used = {c for _, cs in groups for c in cs}
    derived = {c for c in columns if c.endswith(" lpf")} | {"piCount diff"}
    groups += [(c, [c]) for c in columns if c not in used | SKIP | derived]
    if "piCount diff" in cols:
        groups.append(("PI updates / sample", ["piCount diff"]))
    return groups


def style_axis(ax, title):
    ax.set_ylabel(title, fontsize=8)
    ax.grid(True, which="major", lw=0.4)
    ax.grid(True, which="minor", lw=0.2, alpha=0.5)
    ax.legend(loc="upper right", fontsize=7, ncol=4)


def style_time_axis(ax, x0, x1, label="time [s]"):
    """Exact range (no margin before 0), round-number ticks.

    Call on the bottom subplot; with sharex the others only show grid lines.
    """
    ax.set_xlim(x0, x1)
    ax.xaxis.set_major_locator(MaxNLocator(nbins=16, steps=[1, 2, 2.5, 5, 10]))
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.ticklabel_format(axis="x", useOffset=False, style="plain")
    ax.set_xlabel(label)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--t0", type=float, default=None, help="zoom start [s]")
    ap.add_argument("--t1", type=float, default=None, help="zoom end [s]")
    ap.add_argument("--fc", type=float, default=50.0,
                    help="low-pass cutoff [Hz], 0 = off (default 10)")
    ap.add_argument("--fill-ms", type=float, default=0.0,
                    help="bridge speed dropouts (exact 0) up to this long "
                         "before filtering, 0 = off")
    ap.add_argument("--dir", choices=["pos", "neg", "raw"], default="pos",
                    help="pos = all speed lines positive (default), "
                         "neg = all negative, raw = as logged")
    ap.add_argument("--no-raw", action="store_true",
                    help="hide raw trace of filtered channels")
    ap.add_argument("--rel", action="store_true",
                    help="zoomed time axis starts at 0 s instead of --t0")
    ap.add_argument("--save", action="store_true", help="write PNG, don't show")
    args = ap.parse_args()

    d = pd.read_csv(args.csv)
    if d.empty:
        raise SystemExit("empty log")
    d["time_s"] = d.time_s - d.time_s.iloc[0]      # log always starts at 0 s

    # Process the full log so zooming has no filter edge effects
    try:
        d, filt, notes = process(d, args.fc, args.fill_ms, args.dir)
    except ValueError as e:
        raise SystemExit(str(e))

    if args.t0 is not None:
        d = d[d.time_s >= args.t0]
    if args.t1 is not None:
        d = d[d.time_s <= args.t1]
    if d.empty:
        raise SystemExit("no rows in selected time range")
    t_abs = d.time_s                                # time in the full log
    t0_abs = float(t_abs.iloc[0])
    if args.rel:
        d = d.assign(time_s=d.time_s - t0_abs)      # zoom starts at 0 s
    t = d.time_s

    groups = build_groups(list(d.columns))
    fig, axes = plt.subplots(len(groups), 1, sharex=True, squeeze=False,
                             figsize=(14, 2.0 * len(groups) + 1))
    axes = axes[:, 0]
    gaps = d.index[d.seq.diff() > 1] if "seq" in d else []

    for ax, (title, cols) in zip(axes, groups):
        for c in cols:
            if c in STEP:
                ax.step(t, d[c], where="post", label=c, lw=1)
            elif c in filt:
                line, = ax.plot(t, d[c + " lpf"], label=c + " (lpf)", lw=1.3)
                if not args.no_raw and c not in NO_RAW:
                    ax.plot(t, d[c], color=line.get_color(), lw=0.5,
                            alpha=0.25, label="_nolegend_")
            else:
                ax.plot(t, d[c], label=c, lw=1.0)
        for g in gaps:
            ax.axvspan(d.time_s.shift()[g], d.time_s[g], color="0.85", lw=0)
        style_axis(ax, title)

    label = "time [s] from %.3f s" % t0_abs if args.rel and t0_abs > 0 \
        else "time [s]"
    style_time_axis(axes[-1], float(t.iloc[0]), float(t.iloc[-1]), label)

    name = os.path.splitext(os.path.basename(args.csv))[0]
    rng = "" if args.t0 is None and args.t1 is None else \
        "  [%.3f .. %.3f s]" % (t_abs.iloc[0], t_abs.iloc[-1])
    lost = int((d.seq.diff() - 1).clip(lower=0).sum()) if "seq" in d else 0
    extra = ("   " + ", ".join(notes)) if notes else ""
    fig.suptitle("%s%s   rows %d   lost %d%s" % (name, rng, len(d), lost, extra))
    fig.tight_layout()

    if args.save:
        suffix = "" if not rng else "_zoom_%dms_%dms" % (
            round(t_abs.iloc[0] * 1000), round(t_abs.iloc[-1] * 1000))
        if rng and args.rel:
            suffix += "_rel"
        out = os.path.splitext(args.csv)[0] + suffix + ".png"
        # Windows: an open image preview can briefly block the file
        for attempt in range(5):
            try:
                fig.savefig(out, dpi=110)
                break
            except OSError:
                if attempt == 4:
                    raise
                time.sleep(0.5)
        print("saved", out)
    else:
        plt.show()


if __name__ == "__main__":
    main()
