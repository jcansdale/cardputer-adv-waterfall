# SPDX-License-Identifier: GPL-3.0-or-later
"""apps/waterfall/tools/spurs.py: the receiver's lines from synthetic measurements."""

import numpy as np

from conftest import load

sp = load("apps/waterfall/tools/spurs.py")
N = sp.NFFT
BIN = sp.RATE_MHZ / N


def dataset(los, rng):
    med = -70 + rng.normal(0, 0.2, (len(los), N))
    p90 = med + 1.5
    return {"med": med.astype(np.float32), "p90": p90.astype(np.float32), "los": np.array(los)}


def add(d, i, k, db, steady=True):
    (d["med"] if steady else d["p90"])[i, k + N // 2] += db


def test_moving_line_kept_fixed_and_lone_dropped():
    rng = np.random.default_rng(1)
    los = list(range(2300, 2320))
    d = dataset(los, rng)
    for i, lo in enumerate(los):
        add(d, i, int(round((lo - 2280) / 3 / BIN)), 8)  # moves with the LO
        add(d, i, int(round((2310.0 - lo) / BIN)), 8)  # one radio frequency: a real signal
    add(d, 10, -150, 9, steady=False)  # once: noise or a passing signal
    lines = sp.find_lines(d)
    lo = 2310
    moving = int(round((lo - 2280) / 3 / BIN))
    assert lines[lo] == [moving]
    assert all(-150 not in v for v in lines.values())


def test_ranges_and_header():
    assert sp.ranges([-30, -29, -27, 40], grow=1) == [(-31, -26), (39, 41)]
    text = sp.header({1800: [-30, -29], 1801: [], 1802: [5]}, "x.npz")
    assert "#define SPUR_LO_MIN 1800" in text and "#define SPUR_LO_MAX 1802" in text
    assert "spur_idx[4] = {\n\t0, 1, 1, 2,\n};" in text
    assert "{-32, -27}, {3, 7}," in text


def test_header_merges_measurements(tmp_path, run_main):
    rng = np.random.default_rng(2)
    los = list(range(1800, 1810))
    files = []
    for n, ofs in enumerate((-100, -120)):  # the same LOs, a line in another place
        d = dataset(los, rng)
        for i in range(len(los)):
            add(d, i, ofs, 8)
        files.append(tmp_path / f"m{n}.npz")
        np.savez(files[-1], **d)
    out = tmp_path / "spur_tables.h"
    run_main(sp, "header", *files, "-o", out)
    text = out.read_text()
    assert "m0.npz, m1.npz" in text
    assert "{-122, -118}, {-102, -98}," in text
