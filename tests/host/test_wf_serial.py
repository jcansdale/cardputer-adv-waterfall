# SPDX-License-Identifier: GPL-3.0-or-later
"""apps/waterfall/tools/wf_serial.py: the console's text and frame parser, on a fake port."""

import struct
import zlib

import pytest

from conftest import load

wf = load("apps/waterfall/tools/wf_serial.py")


def frame(kind, payload, count, seq=0, lo_khz=2440000, fs_hz=16000000, gain=42):
    body = wf.HDR.pack(kind, 1, count, seq, 1234, lo_khz, fs_hz, gain, 8, 4, len(payload))
    body += payload
    return wf.MAGIC + body + struct.pack("<I", zlib.crc32(body))


class FakeSerial:
    """Hands out the scripted bytes in small reads; replies to writes from a table."""

    port = "fake"

    def __init__(self, data=b"", replies=None):
        self.data = bytearray(data)
        self.replies = replies or {}
        self.written = []

    def read(self, n):
        chunk = bytes(self.data[:7])
        del self.data[:7]
        return chunk

    def write(self, b):
        self.written.append(b)
        self.data += self.replies.get(b, b"")


def link(ser):
    lk = wf.Link.__new__(wf.Link)
    lk.ser, lk.buf, lk.bad, lk.verbose = ser, bytearray(), 0, False
    lk.paused = lk.text = False
    return lk


def test_text_and_frames_interleaved():
    bins = struct.pack("<4h", -32768, -5000, 0, 1234)
    good = frame(wf.FRAME_BINS, bins, 4, seq=7)
    bad = bytearray(frame(wf.FRAME_BINS, bins, 4, seq=8))
    bad[-1] ^= 1
    data = b"spec 1\nhalf " + good + b"line\n" + bytes(bad) + b"after\n"
    events = list(link(FakeSerial(data)).events(0.2))
    lines = [v for k, v in events if k == "line"]
    frames = [v for k, v in events if k == "frame"]
    assert lines[0] == "spec 1"
    assert "half" in lines[1] and lines[-1].endswith("after")
    assert len(frames) == 1 and frames[0].seq == 7 and frames[0].count == 4
    assert frames[0].payload == bins


def test_bad_crc_counted_and_resynced():
    lk = link(FakeSerial())
    bad = bytearray(frame(wf.FRAME_IQ, b"\0" * 16, 4))
    bad[10] ^= 0xFF
    lk.buf += bad + frame(wf.FRAME_IQ, b"\1" * 16, 4, seq=3)
    got = [v for k, v in lk._parse() if k == "frame"]
    assert lk.bad == 1 and [f.seq for f in got] == [3]


def test_command_matches_its_reply():
    ser = FakeSerial(replies={
        b"$span 2\n": b"ok gain auto\nok span 2 rate 2000\n",
        b"$status\n": b"ok span 2\nstat lo 2440 span 2000\n",
        b"$lo 9\n": b"err lo range\n",
    })
    lk = link(ser)
    assert lk.command("$span 2") == "ok span 2 rate 2000"
    assert lk.command("$status") == "stat lo 2440 span 2000"
    assert lk.command("$lo 9") == "err lo range"
    with pytest.raises(TimeoutError):
        lk.command("$peak on", timeout=0.1)


def test_bins_db_and_freqs():
    np = pytest.importorskip("numpy")
    f = wf.Frame(wf.HDR.unpack_from(frame(wf.FRAME_BINS, b"", 4), 4),
                 struct.pack("<4h", -32768, -5000, 0, 1234))
    db = wf.bins_db(f)
    assert np.isnan(db[0]) and list(db[1:]) == pytest.approx([-50.0, 0.0, 12.34])
    assert list(wf.freqs_mhz(f)) == pytest.approx([2432.0, 2436.0, 2440.0, 2444.0])


def rgb565(r, g, b):
    return struct.pack(">H", ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3))


def test_shot_image_landscape():
    red, blue = rgb565(255, 0, 0), rgb565(0, 0, 255)
    rects = [(0, 0, 4, 2, blue * 8), (1, 1, 2, 1, red * 2)]
    img = wf.shot_image(rects, (4, 2, 40, 0, 0, 0, 0, 0))
    assert img.shape == (2, 4, 3)
    assert tuple(img[0, 0]) == (0, 0, 255)
    assert tuple(img[1, 1]) == (255, 0, 0) and tuple(img[1, 2]) == (255, 0, 0)
    assert tuple(img[1, 3]) == (0, 0, 255)


def test_shot_image_scroll_and_rotation():
    # 6 columns, the scroll area columns 2 to 5 (memory lines 42 to 45), scrolled by 1
    cols = [rgb565(v * 40, 0, 0) for v in range(6)]
    rects = [(x, 0, 1, 1, cols[x]) for x in range(6)]
    img = wf.shot_image(rects, (6, 1, 40, 0, 1, 42, 4, 43))
    reds = [int(img[0, x, 0]) // 40 for x in range(6)]
    assert reds == [0, 1, 3, 4, 5, 2]
    left = wf.shot_image(rects, (6, 1, 40, wf.ORIENT_LEFT_DOWN, 0, 0, 0, 0))
    assert left.shape == (6, 1, 3)
    assert int(left[0, 0, 0]) // 40 == 5  # the panel's right edge on top
