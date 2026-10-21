#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Pull data off the waterfall app over its USB console, without the keyboard or screen.

The console takes the app's keys as single bytes and lines starting with $ as commands
($help lists them). Full spectra and raw I/Q come back as binary frames between the text
lines: "ESDR", a 28-byte little-endian header, the payload, then the CRC-32 of everything
after the magic (see struct frame_hdr in src/main.c).

Usage: wf_serial.py [--port P] [--setup LINE]... <command> ...

  status                       the app's state
  cmd LINE...                  send console lines ($tune 2450000, or keys such as ;;)
  bins [-n N] [-o F.npz] [--png F.png]
                               N full 1024-bin spectra in dBFS, as the app saw them before
                               any patching, saved with their metadata and/or drawn
  iq [-n N] [-o F.npz]         N raw bursts of 4096 complex samples (full scale 512 -> 1.0)
  lines [-n N] [--db D]        steady narrow lines: bins D dB above their neighbours'
                               median in most rows, with their frequency and level
  live                         a live waterfall in a matplotlib window
  shot -o F.png [--scale N]    the screen as drawn, upright, N times larger (default 3)

--setup sends console lines first, e.g.
  wf_serial.py --setup '$span 2' --setup '$tune 2448000' lines

Captures pause the app's screen (up to 4 times the rows per second) unless --keep-screen;
settings made with cmd or --setup stay after the tool exits.

pyserial is needed; numpy for bins, iq and lines; matplotlib for --png and live.
"""

import argparse
import glob
import struct
import sys
import time
import zlib

MAGIC = b"ESDR"
HDR = struct.Struct("<BBHIIiIhBBI")  # after the magic
FRAME_BINS, FRAME_IQ = ord("B"), ord("I")
FRAME_RECT, FRAME_SHOT = ord("R"), ord("S")
SHOT_INFO = struct.Struct("<8H")  # struct shot_info
ORIENT_LEFT_DOWN, ORIENT_RIGHT_DOWN = 1, 2
FULL_SCALE = 512.0
MAX_PAYLOAD = 1 << 20


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("no USB serial port found; give one with --port")
    return ports[0]


class Frame:
    def __init__(self, fields, payload):
        (self.type, self.version, self.count, self.seq, self.ms, self.lo_khz, self.fs_hz,
         self.gain, self.bursts, self.segments, _) = fields
        self.payload = payload


class Link:
    """The console: text lines and binary frames, interleaved."""

    def __init__(self, port, verbose=False):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is needed (pip install pyserial)")
        self.ser = serial.Serial()
        self.ser.port = port
        # The ESP32-S3's USB Serial/JTAG resets the chip on RTS without DTR; macOS raises
        # both on open, and leaving them up keeps the app running (False, False passes
        # through RTS alone as pyserial drops DTR first).
        self.ser.dtr = True
        self.ser.rts = True
        self.ser.timeout = 0.05
        self.ser.open()
        self.buf = bytearray()
        self.bad = 0
        self.verbose = verbose
        self.paused = False
        self.text = False

    def send(self, line):
        if line.startswith("$"):
            line += "\n"
        self.ser.write(line.encode())

    def events(self, timeout):
        """Yields ("line", str) and ("frame", Frame) until timeout seconds pass."""
        end = time.time() + timeout
        while True:
            yield from self._parse()
            if time.time() >= end:
                return
            data = self.ser.read(65536)
            if data:
                self.buf += data

    def _parse(self):
        while True:
            m = self.buf.find(MAGIC)
            text_end = m if m >= 0 else self.buf.rfind(b"\n") + 1
            if text_end > 0:
                lines = bytes(self.buf[:text_end]).split(b"\n")
                del self.buf[:text_end]
                for line in lines:
                    line = line.decode(errors="replace").strip()
                    if line:
                        yield "line", line
                continue
            if m != 0 or len(self.buf) < 4 + HDR.size:
                return
            fields = HDR.unpack_from(self.buf, 4)
            plen = fields[-1]
            if plen > MAX_PAYLOAD:
                self.bad += 1
                del self.buf[:4]
                continue
            total = 4 + HDR.size + plen + 4
            if len(self.buf) < total:
                return
            body = bytes(self.buf[4:total - 4])
            (crc,) = struct.unpack_from("<I", self.buf, total - 4)
            if zlib.crc32(body) != crc:
                self.bad += 1
                del self.buf[:4]
                continue
            del self.buf[:total]
            yield "frame", Frame(fields, body[HDR.size:])

    def command(self, line, timeout=3.0):
        """Sends a $ command; returns its reply, "ok <name> ...", "err <name> ..." or "stat ..."."""
        self.send(line)
        name = line[1:].split(" ", 1)[0]
        for kind, v in self.events(timeout):
            words = v.split(" ") if kind == "line" else []
            if words[:1] == ["stat"] if name == "status" else \
                    words[:2] in (["ok", name], ["err", name]):
                return v
            if kind == "line" and self.verbose:
                print(v, file=sys.stderr)
        raise TimeoutError(f"no reply to {line}")

    def sync(self, timeout=15.0):
        """Waits for the app and silences its text lines.

        A boot banner soon after opening (another tool reset the board) means waiting for
        the radio to come up.
        """
        end = time.time() + timeout
        booting = False
        for kind, v in self.events(0.5):
            if kind == "line" and "Booting Zephyr" in v:
                booting = True
                break
        while booting and time.time() < end:
            if any(k == "line" and "radio ready" in v for k, v in self.events(0.2)):
                booting = False
        while time.time() < end:
            try:
                self.ser.write(b"\n")  # ends a half-typed command
                self.status = self.command("$status", timeout=1.0)
                self.text = " text 1 " in self.status
                if self.text:
                    self.command("$text off", timeout=1.0)
                return self.status
            except TimeoutError:
                pass
        sys.exit("the app does not answer on " + self.ser.port)

    def frames(self, kind, n, timeout_each=5.0):
        got = 0
        last = time.time()
        while got < n:
            for k, v in self.events(0.2):
                if k == "frame" and v.type == kind:
                    yield v
                    got += 1
                    last = time.time()
                    if got >= n:
                        return
                elif k == "line" and self.verbose:
                    print(v, file=sys.stderr)
            if time.time() - last > timeout_each:
                raise TimeoutError(f"{got} of {n} frames")

    def pause_screen(self):
        """Drawing takes most of the app's time: without it, rows come up to 4 times faster."""
        self.command("$screen off")
        self.paused = True

    def close(self):
        try:
            self.command("$bins off", timeout=1.0)
            self.command("$iq off", timeout=1.0)
            if self.text:
                self.command("$text on", timeout=1.0)
            if self.paused:
                self.command("$screen on", timeout=1.0)
        except TimeoutError:
            pass
        self.ser.close()


def bins_db(frame):
    import numpy as np
    v = np.frombuffer(frame.payload, dtype="<i2").astype(np.float32) / 100.0
    v[v <= -327.0] = np.nan
    return v


def freqs_mhz(frame):
    import numpy as np
    n = frame.count
    return frame.lo_khz / 1000.0 + (np.arange(n) - n // 2) * frame.fs_hz / n / 1e6


def collect_bins(link, n):
    import numpy as np
    link.command(f"$bins {n}")
    frames = list(link.frames(FRAME_BINS, n))
    return frames, np.vstack([bins_db(f) for f in frames])


def cmd_status(link, args):
    print(link.status)  # as found, before the tool silenced the text lines


def cmd_cmd(link, args):
    for line in args.lines:
        if line.startswith("$"):
            print(link.command(line))
            if line.split()[0] == "$text":
                link.text = False  # the user's choice stays
        else:
            link.send(line)
    time.sleep(0.3)
    print(link.command("$status"))


def save_png(path, frames, db):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    f = freqs_mhz(frames[0])
    t = (np.array([fr.ms for fr in frames]) - frames[0].ms) / 1000.0
    lo, hi = np.nanpercentile(db, [5, 99.9])
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(db, aspect="auto", vmin=lo, vmax=hi, cmap="viridis",
              extent=[f[0], f[-1], t[-1], t[0]])
    ax.set_xlabel("MHz")
    ax.set_ylabel("s")
    ax.set_title(f"LO {frames[0].lo_khz / 1000:g} MHz, {frames[0].fs_hz / 1e6:g} MS/s, "
                 f"gain {frames[0].gain}, {len(frames)} rows")
    fig.tight_layout()
    fig.savefig(path, dpi=100)


def cmd_bins(link, args):
    import numpy as np
    frames, db = collect_bins(link, args.n)
    secs = (frames[-1].ms - frames[0].ms) / 1000.0
    print(f"{len(frames)} rows in {secs:.1f} s, LO {frames[0].lo_khz} kHz, "
          f"{frames[0].fs_hz / 1e6:g} MS/s, median {np.nanmedian(db):.1f} dBFS, "
          f"max {np.nanmax(db):.1f} dBFS, {link.bad} bad frames")
    if args.o:
        np.savez_compressed(args.o, db=db, freq_mhz=freqs_mhz(frames[0]),
                            seq=[f.seq for f in frames], ms=[f.ms for f in frames],
                            lo_khz=[f.lo_khz for f in frames],
                            fs_hz=[f.fs_hz for f in frames], gain=[f.gain for f in frames],
                            bursts=[f.bursts for f in frames])
        print("saved", args.o)
    if args.png:
        save_png(args.png, frames, db)
        print("saved", args.png)


def cmd_iq(link, args):
    import numpy as np
    link.command(f"$iq {args.n}")
    frames = list(link.frames(FRAME_IQ, args.n))
    iq = np.vstack([np.frombuffer(f.payload, dtype="<i2").astype(np.float32).view(np.complex64)
                    for f in frames]) / FULL_SCALE
    rms = np.sqrt(np.mean(np.abs(iq) ** 2))
    print(f"{len(frames)} bursts of {iq.shape[1]} samples, LO {frames[0].lo_khz} kHz, "
          f"{frames[0].fs_hz / 1e6:g} MS/s, rms {20 * np.log10(rms):.1f} dBFS, "
          f"peak {np.max(np.abs(iq)):.3f}, DC {np.mean(iq):.4f}, {link.bad} bad frames")
    if args.o:
        np.savez_compressed(args.o, iq=iq, ms=[f.ms for f in frames],
                            lo_khz=[f.lo_khz for f in frames], fs_hz=[f.fs_hz for f in frames],
                            gain=[f.gain for f in frames])
        print("saved", args.o)


def cmd_lines(link, args):
    import numpy as np
    frames, db = collect_bins(link, args.n)
    f = freqs_mhz(frames[0])
    k = 15  # neighbours each side
    win = np.lib.stride_tricks.sliding_window_view(np.pad(db, ((0, 0), (k, k)), mode="edge"),
                                                   2 * k + 1, axis=1)
    above = db - np.nanmedian(win, axis=2)
    steady = np.mean(above > args.db, axis=0)
    level = np.nanmedian(above, axis=0)
    hits = np.flatnonzero(steady >= args.frac)
    print(f"{len(frames)} rows, LO {frames[0].lo_khz} kHz, "
          f"{frames[0].fs_hz / 1e6:g} MS/s, {f[1] - f[0]:.4f} MHz bins")
    print("       MHz   bin  above     rows")
    groups = np.split(hits, np.flatnonzero(np.diff(hits) > 1) + 1) if hits.size else []
    for g in groups:
        c = g[np.argmax(level[g])]
        print(f"{f[c]:10.4f} {c - len(f) // 2:+5d} {level[c]:5.1f} dB {steady[c]:6.0%}")


def cmd_live(link, args):
    import matplotlib.pyplot as plt
    import numpy as np
    link.command("$bins on")
    rows = None
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    im = None
    while plt.fignum_exists(fig.number):
        new = [v for k, v in link.events(0.1) if k == "frame" and v.type == FRAME_BINS]
        if not new:
            plt.pause(0.01)
            continue
        db = np.vstack([bins_db(fr) for fr in new])[::-1]
        rows = db if rows is None else np.vstack([db, rows])[:args.rows]
        f = freqs_mhz(new[-1])
        lo, hi = np.nanpercentile(rows, [5, 99.9])
        if im is None:
            im = ax.imshow(rows, aspect="auto", cmap="viridis", vmin=lo, vmax=hi)
            ax.set_xlabel("MHz")
            ax.set_ylabel("rows ago")
        else:
            im.set_data(rows)
            im.set_clim(lo, hi)
        im.set_extent([f[0], f[-1], rows.shape[0], 0])
        ax.set_title(f"LO {new[-1].lo_khz / 1000:g} MHz, {new[-1].fs_hz / 1e6:g} MS/s, "
                     f"gain {new[-1].gain}")
        plt.pause(0.01)


def shot_image(rects, info):
    """The screen as the viewer sees it, (rows, columns, 3) uint8, from $shot's frames.

    @p rects: (x, y, w, h, big-endian RGB565 bytes) display writes, in order; @p info:
    struct shot_info's fields. The display reads frame memory line panel_x0 + x for column x,
    except that inside the scroll area (portrait) line tfa + i shows tfa + (vsp - tfa + i) % vsa.
    """
    import numpy as np
    width, height, x0, orient, scroll_on, tfa, vsa, vsp = info
    mem = np.zeros((x0 + width + vsa + 1, height), np.uint16)  # [memory line, y]
    for x, y, w, h, px in rects:
        mem[x0 + x:x0 + x + w, y:y + h] = np.frombuffer(px, ">u2").reshape(h, w).T
    lines = x0 + np.arange(width)
    if scroll_on:
        inside = (lines >= tfa) & (lines < tfa + vsa)
        lines = np.where(inside, tfa + (vsp - tfa + lines - tfa) % vsa, lines)
    v = mem[lines].T.astype(np.uint32)  # [y, x]
    rgb = np.stack([(v >> 11) & 31, (v >> 5) & 63, v & 31], axis=-1)
    rgb = (rgb * 255 // np.array([31, 63, 31])).astype(np.uint8)
    if orient == ORIENT_LEFT_DOWN:  # the panel's right edge on top
        rgb = np.rot90(rgb, 1)
    elif orient == ORIENT_RIGHT_DOWN:
        rgb = np.rot90(rgb, -1)
    return np.ascontiguousarray(rgb)


def cmd_shot(link, args):
    import numpy as np
    reply = link.command("$shot")
    if not reply.startswith("ok"):
        sys.exit(reply)
    rects, end = [], time.time() + 15
    while time.time() < end:
        for kind, v in link.events(0.2):
            if kind != "frame":
                continue
            if v.type == FRAME_RECT:
                x, y, w, h = struct.unpack_from("<4H", v.payload)
                rects.append((x, y, w, h, v.payload[8:]))
            elif v.type == FRAME_SHOT:
                img = shot_image(rects, SHOT_INFO.unpack(v.payload))
                img = np.repeat(np.repeat(img, args.scale, axis=0), args.scale, axis=1)
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.image
                matplotlib.image.imsave(args.o, img)
                print(f"saved {args.o}: {img.shape[1]}x{img.shape[0]}, {len(rects)} writes")
                return
    sys.exit(f"no complete screenshot ({len(rects)} writes, {link.bad} bad frames)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n", 2)[2])
    ap.add_argument("--port", help="serial port (default: the first /dev/cu.usbmodem*)")
    ap.add_argument("--setup", action="append", default=[],
                    help="a console line to send first (repeatable)")
    ap.add_argument("--keep-screen", action="store_true",
                    help="keep drawing during captures (slower)")
    ap.add_argument("-v", "--verbose", action="store_true", help="print other console lines")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    p = sub.add_parser("cmd")
    p.add_argument("lines", nargs="+")
    p = sub.add_parser("bins")
    p.add_argument("-n", type=int, default=100)
    p.add_argument("-o", help="save as .npz")
    p.add_argument("--png", help="draw the waterfall")
    p = sub.add_parser("iq")
    p.add_argument("-n", type=int, default=1)
    p.add_argument("-o", help="save as .npz")
    p = sub.add_parser("lines")
    p.add_argument("-n", type=int, default=100)
    p.add_argument("--db", type=float, default=6.0, help="above the neighbours (default 6)")
    p.add_argument("--frac", type=float, default=0.8, help="of rows (default 0.8)")
    p = sub.add_parser("live")
    p.add_argument("--rows", type=int, default=300)
    p = sub.add_parser("shot")
    p.add_argument("-o", required=True, help="the image (.png)")
    p.add_argument("--scale", type=int, default=3, help="pixels per screen pixel (default 3)")
    args = ap.parse_args()

    link = Link(args.port or find_port(), args.verbose)
    try:
        link.sync()
        for line in args.setup:
            if line.startswith("$"):
                print(link.command(line), file=sys.stderr)
            else:
                link.send(line)
        if args.cmd in ("bins", "iq", "lines", "live") and not args.keep_screen:
            link.pause_screen()
        if args.setup:
            time.sleep(0.5)  # the radio retunes before the next capture
        globals()["cmd_" + args.cmd](link, args)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


if __name__ == "__main__":
    main()
