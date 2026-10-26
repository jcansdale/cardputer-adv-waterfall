#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""The receive gain indices, characterised from what is on the air (receive only).

  gaincal.py measure -o gain.npz [--lo 2437] [--gains 0:83:2]
      at each gain: the raw bins of single bursts (floor, DC), rows peak held over many bursts
      (bursty signals) and raw I/Q (median RMS, peak, clipping); about 3.5 s a gain, 5 minutes for all; puts the
      settings back after
  gaincal.py plot gain.npz [more.npz ...] [--png gains.png]
      per gain: the floor, a clock line's level (the board's own harmonic, steady) and its step
      against the previous gain, the DC bin, the clipped fraction and the ambient signals'
      margin over the floor; prints the table
"""

import argparse
import time

SPAN = 16  # MHz: 40 MS/s, 1024 bins of 39 kHz
FRAMES = 12
IQ_BURSTS = 9
CLIP = 511


def measure(link, lo, gains, out=print):
    import numpy as np
    from wf_serial import FRAME_BINS, FRAME_IQ, bins_db, freqs_mhz

    def bins(n, g):
        link.command(f"$bins {n + 2}")
        rows = [bins_db(f) for f in link.frames(FRAME_BINS, n + 2)
                if f.lo_khz == lo * 1000 and f.gain == g]
        return np.vstack(rows[-n:])

    for c in ("$post off", f"$span {SPAN}", f"$lo {lo}"):
        link.command(c)
    res = {"gains": np.array(gains), "lo": lo}
    single, held, iq_rms, iq_peak, clip, dc = [], [], [], [], [], []
    for g in gains:
        t0 = time.time()
        link.command(f"$gain {g}")
        link.command("$speed -4")  # one burst a row: a plain Welch average
        single.append(np.median(bins(FRAMES, g), axis=0))  # the median: a Wi-Fi burst drops out
        link.command("$speed 2")  # tens of bursts a row, peak held
        held.append(np.median(bins(FRAMES, g), axis=0))
        link.command(f"$iq {IQ_BURSTS}")
        iq = np.vstack([np.frombuffer(f.payload, dtype="<i2").reshape(-1, 2)
                        for f in link.frames(FRAME_IQ, IQ_BURSTS)]).astype(np.float32)
        z = (iq[:, 0] + 1j * iq[:, 1]).reshape(IQ_BURSTS, -1)
        dc.append(np.median(np.mean(z, axis=1)))
        iq_rms.append(np.median(np.std(z, axis=1)))  # the quietest half of the bursts
        iq_peak.append(np.max(np.abs(iq)))
        clip.append(np.mean(np.any(np.abs(iq) >= CLIP, axis=1)))
        out(f"gain {g}: floor {np.nanmedian(single[-1]):.1f} dBFS, rms {iq_rms[-1]:.1f}, "
            f"peak {iq_peak[-1]:.0f}, clipped {clip[-1] * 100:.3f}%, {time.time() - t0:.1f} s")
    link.command("$speed -4")
    link.command("$bins 1")
    f = next(iter(link.frames(FRAME_BINS, 1)))
    res.update(freqs=freqs_mhz(f), single=np.array(single), held=np.array(held),
               iq_rms=np.array(iq_rms), iq_peak=np.array(iq_peak), clip=np.array(clip),
               dc=np.array(dc))
    return res


def analyse(d, ref=None):
    """Per gain: floor, DC bin, the reference line, ambient signals' margin over the floor."""
    import numpy as np

    f = d["freqs"]
    lo = float(d["lo"])
    inner = (np.abs(f - lo) < 12) & (np.abs(f - lo) > 0.3)  # inside the filters, off the LO
    if ref is None:  # the steadiest of the board's clock harmonics (40 and 48 MHz: 8 MHz apart)
        best = None
        for c in np.unique(np.round(f[inner] / 8) * 8):
            lv = np.nanmax(d["held"][:, np.abs(f - c) < 0.1], axis=1)
            scatter = np.nanmedian(np.abs(np.diff(lv, 2)))
            if best is None or scatter < best[0]:
                best = (scatter, float(c))
        ref = best[1]
    centre = np.abs(f - lo) < 0.05
    near = np.abs(f - ref) < 0.1
    floor = np.nanmedian(d["single"][:, inner], axis=1)
    dcbin = np.nanmax(d["single"][:, centre], axis=1)
    line = np.nanmax(d["held"][:, near], axis=1)
    band = inner & (np.abs(f - lo) < 9) & (np.abs(f - ref) > 0.3)
    margin = np.nanpercentile(d["held"][:, band], 90, axis=1) - floor
    return ref, floor, dcbin, line, margin


def plot(paths, png):
    import numpy as np

    runs = []
    for path in paths:
        d = np.load(path)
        ref, floor, dcbin, line, margin = analyse(d)
        g = d["gains"]
        rms_db = 20 * np.log10(d["iq_rms"] / CLIP)
        runs.append((d, g, ref, floor, line, margin, rms_db))
        print(f"{path}: LO {int(d['lo'])} MHz, reference line {ref:.2f} MHz")
        print(" gain  floor  line  step   rms dBFS  peak  clip%   DC  margin")
        for i, gi in enumerate(g):
            step = "" if i == 0 else f"{line[i] - line[i - 1]:+5.1f}"
            print(f"{gi:5d} {floor[i]:6.1f} {line[i]:5.1f} {step:>5} {rms_db[i]:8.1f} "
                  f"{d['iq_peak'][i]:5.0f} {d['clip'][i] * 100:6.3f} {abs(d['dc'][i]):5.1f} "
                  f"{margin[i]:6.1f}")
    if not png:
        return
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    for n, (d, g, ref, floor, line, margin, rms_db) in enumerate(runs):
        c = f"C{n}"
        lo = int(d["lo"])
        ax[0].plot(g, line, "o-", ms=3, color=c, label=f"LO {lo}: clock line {ref:.0f} MHz")
        ax[0].plot(g, floor, "s--", ms=3, color=c, alpha=0.7, label=f"LO {lo}: floor (median bin)")
        ax[1].step(g[1:], np.diff(line), where="mid", color=c, label=f"LO {lo}")
        ax[2].plot(g, margin, "o-", ms=3, color=c, label=f"LO {lo}")
        clipped = g[d["clip"] > 0]
        ax[2].plot(clipped, margin[d["clip"] > 0], "x", color="red", ms=8,
                   label="bursts clipped" if n == 0 else None)
    ax[0].set_ylabel("dBFS")
    ax[0].set_title("Receive gain index (peak-held line, single-burst floor)")
    ax[1].set_ylabel("line step (dB)")
    ax[1].axhline(1, color="grey", lw=0.5)
    ax[2].set_ylabel("ambient Wi-Fi over floor (dB)")
    ax[2].set_xlabel("gain index")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(png, dpi=100)
    print(f"wrote {png}")


def main():
    import wf_serial as W

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("measure")
    p.add_argument("-o", required=True)
    p.add_argument("--lo", type=int, default=2437)
    p.add_argument("--port")
    p.add_argument("--gains", default="0:83:2", help="start:stop:step or a comma list")
    p = sub.add_parser("plot")
    p.add_argument("files", nargs="+")
    p.add_argument("--png")
    args = ap.parse_args()
    if args.cmd == "plot":
        plot(args.files, args.png)
        return
    import numpy as np

    gains = (list(range(*map(int, args.gains.split(":")))) if ":" in args.gains
             else [int(v) for v in args.gains.split(",")])
    link = W.Link(args.port or W.find_port())
    try:
        status = link.sync()
        link.pause_screen()
        res = measure(link, args.lo, gains)
    finally:
        # back as found
        st = dict(zip(status.split()[1::2], status.split()[2::2]))
        gain = st["gain"] if int(st["gain"]) >= 0 else "auto"
        for line in (f"$speed {st['speed']}", f"$gain {gain}", f"$post {st.get('post', 0)}",
                     f"$span {int(st['span']) // 1000}", f"$tune {st['view']}"):
            try:
                link.command(line)
            except TimeoutError:
                pass
        link.close()
    np.savez_compressed(args.o, **res)
    print(f"wrote {args.o}")


if __name__ == "__main__":
    main()
