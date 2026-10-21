# SPDX-License-Identifier: GPL-3.0-or-later
"""apps/waterfall/tools/flatfield.py: the receiver's shape from synthetic measurements."""

import numpy as np

from conftest import load

ff = load("apps/waterfall/tools/flatfield.py")
N = ff.NFFT


def dataset(shape_db, line_mhz=None, gains=(42,), seed=0):
    """measure()'s output: every tuning the same shape plus noise and an RF line that moves."""
    rng = np.random.default_rng(seed)
    d = {}
    for span, rate in ff.SPANS.items():
        d[f"s{span}_fs"] = rate * 1e6
        f_mhz = (np.arange(N) - N // 2) * rate / N
        for g in gains:
            rows = -70 + shape_db(f_mhz) + rng.normal(0, 0.3, (len(ff.LOS), N))
            if line_mhz is not None:
                for i, lo in enumerate(ff.LOS):
                    k = int(round((line_mhz - lo) * N / rate)) + N // 2
                    if 0 <= k < N:
                        rows[i, k] += 20
            d[f"s{span}_g{g}"] = rows
    return d


def test_gain_sets():
    assert [ff.gain_set(g) for g in (0, 35, 36, 55, 56, 82)] == [0, 0, 1, 1, 2, 2]


def test_shape_recovered_and_moving_line_dropped():
    def shape(f):
        return -3.0 * (np.abs(f) / 20) ** 4

    tables = ff.shapes([dataset(shape, line_mhz=2400.0)])
    assert set(tables) == {(80, 1), (40, 1), (16, 1)}
    corr, rows = tables[(80, 1)]
    assert rows == len(ff.LOS)
    f = (np.arange(N) - N // 2) * 80 / N
    inside = (np.abs(f) < 20) & (np.abs(f) > 0.2)
    # the correction undoes the shape, about the middle's level
    resid = shape(f) + corr
    assert np.ptp(resid[inside]) < 0.6
    assert corr[np.abs(f) <= ff.BRIDGE_KHZ / 1e3].max() == 0.0
    assert corr.max() <= ff.MAX_BOOST_DB


def test_header_is_natural_order_and_falls_back_to_mid_set():
    def shape(f):
        return np.where(f > 22, -2.0, 0.0)  # at 80 MS/s the top 18 MHz 2 dB down

    tables = ff.shapes([dataset(shape)])
    text = ff.header(tables, ["x.npz"])
    assert "static const float flat_tab[3][3][1024]" in text
    vals = [float(v.rstrip("f,")) for line in text.splitlines()
            if line.startswith("\t\t\t") for v in line.split()]
    assert len(vals) == 3 * 3 * N
    first = np.array(vals[:N])  # 80 MS/s, gain set 0 (absent: the mid set's)
    # DC first: bins 1 to N/2 - 1 are above the LO (the top ones boosted), the rest below
    assert np.median(first[300:N // 2 - 10]) > 1.4
    assert abs(np.median(first[10:250]) - 1.0) < 0.1
    assert abs(np.median(first[N // 2 + 10:-10]) - 1.0) < 0.1
    assert first[0] == 1.0
