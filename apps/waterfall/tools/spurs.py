#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The receiver's own spurs at every whole-MHz LO, measured and written out as src/spur_tables.h.

Besides its fixed clock harmonics the receiver makes narrow lines that move with the LO, at
offsets that change from one tuning to the next (2.3 to 13 MHz below it around 1840 MHz,
some there in only a few bursts in ten). The steady-line mask learns them in a view that
stays put; a sweep hops the LO every step, too fast to learn, so it bridges the bins this
table lists for each step's LO instead (x).

  spurs.py measure -o spurs.npz [--from 1800 --to 2800]
      raw bins (before any correction) at 80 MS/s at every whole-MHz LO: per bin the median
      and the 90th percentile over FRAMES captures; about 6 minutes for 1001 LOs
  spurs.py header spurs.npz [more.npz ...] [-o src/spur_tables.h] [--png lines.png]
      the lines: bins standing above their neighbours in most captures (or strongly in a
      few), and not at one radio frequency over the neighbouring LOs (those are real signals
      or the clock harmonics, which stay put while the LO moves)

Below about 1842 MHz the lines sit in one of two places, about 1.7 MHz apart, depending on the
boot: after a reset (flashing included) always the higher, after power-on usually the lower.
Measure both (after a reset, measure all; replug until the other shows, measure 1795-1860) and
merge them in one header; above 1842 MHz they agree to the bin.

The table is this board's; measure another board's and rebuild.
"""

import argparse
import sys
import time

NFFT = 1024
RATE_MHZ = 80
FRAMES = 16
USE_BINS = 215  # +-16.8 MHz about the LO: what a sweep step uses (+-16), with room
NEAR, FAR = 3, 12  # the neighbours a line is measured against, bins either side
STEADY_DB = 2.5  # above the neighbours in the median capture
BURSTY_DB = 5.0  # or in the 90th percentile one
FIXED_LOS = 3  # at one radio frequency at this many of the 4 nearest LOs: not ours
TRACK_BINS = 6  # ours move smoothly: a hit near this offset at the next or previous LO
BRIDGE_BINS = 2  # lo_bridge()'s


def measure(link, los, out=print):
    """{"med", "p90": (LOs, NFFT) dB fftshifted, "los": MHz}."""
    import numpy as np
    from wf_serial import FRAME_BINS, bins_db

    med = np.full((len(los), NFFT), np.nan, np.float32)
    p90 = np.full((len(los), NFFT), np.nan, np.float32)
    link.command("$peak 1")
    link.command("$span 40")
    t0 = time.time()
    for i, lo in enumerate(los):
        link.command(f"$lo {lo}")
        link.command(f"$bins {FRAMES + 3}")
        rows = [bins_db(f) for f in link.frames(FRAME_BINS, FRAMES + 3) if f.lo_khz == lo * 1000]
        if rows:
            p = np.vstack(rows[-FRAMES:])
            med[i] = np.median(p, axis=0)
            p90[i] = np.percentile(p, 90, axis=0)
        if i % 100 == 99:
            out(f"{lo} MHz, {time.time() - t0:.0f} s")
    return {"med": med, "p90": p90, "los": np.array(los)}


def excess(p):
    """Each bin over the median of its neighbours NEAR to FAR bins either side, dB."""
    import numpy as np

    off = [d for d in range(-FAR, FAR + 1) if abs(d) >= NEAR]
    nb = np.stack([np.roll(p, -d, axis=-1) for d in off])
    return p - np.median(nb, axis=0)


def find_lines(d):
    """{LO MHz: sorted bins about the LO (-USE_BINS to USE_BINS)} of the receiver's own lines."""
    import numpy as np

    los = list(d["los"])
    k = np.arange(NFFT) - NFFT // 2
    use = (np.abs(k) <= USE_BINS) & (np.abs(k) > BRIDGE_BINS)
    hit = (excess(d["med"]) > STEADY_DB) | (excess(d["p90"]) > BURSTY_DB)
    hit &= use
    hit[np.isnan(d["med"]).any(axis=1)] = False
    bin_mhz = RATE_MHZ / NFFT
    # a hit's radio frequency in bins of a grid common to every LO
    rf = {lo: set(np.round((lo + k[hit[i]] * bin_mhz) / bin_mhz).astype(int))
          for i, lo in enumerate(los)}
    moving = {}
    for i, lo in enumerate(los):
        keep = []
        for kk in k[hit[i]]:
            r = round((lo + kk * bin_mhz) / bin_mhz)
            fixed = sum(any(r + j in rf.get(lo + dl, ()) for j in (-1, 0, 1))
                        for dl in (-2, -1, 1, 2))
            if fixed < FIXED_LOS:
                keep.append(int(kk))
        moving[lo] = np.array(keep, dtype=int)
    # a lone hit is noise or a passing signal; the receiver's lines track the LO
    lines = {}
    for lo, ks in moving.items():
        near = [moving.get(lo + dl, np.array([], int)) for dl in (-1, 1)]
        lines[lo] = [int(kk) for kk in ks
                     if any(len(n) and np.min(np.abs(n - kk)) <= TRACK_BINS for n in near)]
    return lines


def ranges(bins, grow=2):
    """Sorted bins as (first, last) runs, each widened by @p grow bins either side."""
    out = []
    for b in bins:
        if out and b - grow <= out[-1][1] + 1:
            out[-1][1] = b + grow
        else:
            out.append([b - grow, b + grow])
    return [tuple(r) for r in out]


def header(lines, source):
    los = sorted(lines)
    idx, runs = [], []
    for lo in los:
        idx.append(len(runs))
        runs.extend(ranges(lines[lo]))
    idx.append(len(runs))
    out = [
        "/* SPDX-License-Identifier: GPL-3.0-or-later */",
        "/*",
        f" * Generated by tools/spurs.py from {source}: do not edit.",
        " * The receiver's own lines at 80 MS/s for each whole-MHz LO from SPUR_LO_MIN to",
        " * SPUR_LO_MAX: spur_run[spur_idx[lo - SPUR_LO_MIN]] up to the next LO's, each the",
        " * first and last bin about the LO (main.c, sweep_patch()).",
        " */",
        "",
        f"#define SPUR_LO_MIN {los[0]}",
        f"#define SPUR_LO_MAX {los[-1]}",
        "",
        f"static const uint16_t spur_idx[{len(idx)}] = {{",
    ]
    for i in range(0, len(idx), 12):
        out.append("\t" + ", ".join(str(v) for v in idx[i:i + 12]) + ",")
    out += ["};", "", f"static const int16_t spur_run[{max(len(runs), 1)}][2] = {{"]
    for i in range(0, len(runs), 6):
        out.append("\t" + ", ".join(f"{{{a}, {b}}}" for a, b in runs[i:i + 6]) + ",")
    if not runs:
        out.append("\t{0, -1},")
    out.append("};")
    return "\n".join(out) + "\n"


def save_png(path, lines):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 8))
    for lo, ks in lines.items():
        ax.plot([k * RATE_MHZ / NFFT for k in ks], [lo] * len(ks), "k.", ms=1.5)
    ax.set_xlabel("MHz from the LO")
    ax.set_ylabel("LO, MHz")
    ax.invert_yaxis()
    ax.grid()
    fig.tight_layout()
    fig.savefig(path, dpi=80)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("measure")
    p.add_argument("--port")
    p.add_argument("--from", dest="lo_from", type=int, default=1800)
    p.add_argument("--to", dest="lo_to", type=int, default=2800)
    p.add_argument("-o", required=True, help=".npz")
    p = sub.add_parser("header")
    p.add_argument("npz", nargs="+", help="measurements, merged: each LO's lines in any")
    p.add_argument("-o", default="-", help="the header (default: stdout)")
    p.add_argument("--png")
    args = ap.parse_args()

    import numpy as np

    if args.cmd == "measure":
        import wf_serial

        link = wf_serial.Link(args.port or wf_serial.find_port())
        status = None
        try:
            status = link.sync()
            link.pause_screen()
            res = measure(link, list(range(args.lo_from, args.lo_to + 1)),
                          out=lambda s: print(s, file=sys.stderr))
        finally:
            if status:
                st = dict(zip(status.split()[1::2], status.split()[2::2]))
                for line in (f"$peak {st['peak']}", f"$span {int(st['span']) // 1000}",
                             f"$tune {st['view']}"):
                    try:
                        link.command(line)
                    except TimeoutError:
                        pass
            link.close()
        np.savez_compressed(args.o, **res)
        return

    lines = {}
    for f in args.npz:
        for lo, ks in find_lines(np.load(f)).items():
            lines[lo] = sorted(set(lines.get(lo, [])) | set(ks))
    text = header(lines, ", ".join(f.split("/")[-1] for f in args.npz))
    if args.o == "-":
        sys.stdout.write(text)
    else:
        with open(args.o, "w") as f:
            f.write(text)
    if args.png:
        save_png(args.png, lines)


if __name__ == "__main__":
    main()
