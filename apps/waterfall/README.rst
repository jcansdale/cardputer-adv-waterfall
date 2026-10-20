.. SPDX-License-Identifier: GPL-3.0-or-later

ESP-SDR waterfall (M5Stack Cardputer-Adv)
#########################################

.. The path works from here (GitHub) and from doc/apps/waterfall.rst (Sphinx), both two
   levels below the repository root.

.. image:: ../../apps/waterfall/cardputer-adv.jpg
   :alt: The Cardputer-Adv showing the waterfall at 2454.5 MHz, 80 MHz span, with the Wi-Fi
         channel numbers over the trace
   :width: 480

A receive-only spectrum and waterfall on the Cardputer-Adv's 240x135 display, in plain C:
no MicroPython, no LVGL, no PSRAM (the Stamp-S3A has none).

Each row is a Welch average of Hann-windowed 1024-point FFTs over one 4096-sample burst at
16, 40 or 80 MS/s. The up and down keys step the screen's span through 1000, 640, 320, 160,
80, 40, 16, 8, 4 and 2 MHz (40 and wider sweep, below): 16 and 8 MHz at 40 MS/s, 4 and 2 MHz at 16 MS/s
(each showing part of the capture). The radio's analog filters pass about +-10 MHz at 16 MS/s and,
in the PHY's 40 MHz mode used at 40 and 80 MS/s, about +-18 MHz, so the outer parts of a wide
view are attenuated. At 16 MS/s that is wider than the capture itself: whatever lies 8 to
10 MHz out folds back in at the other edge (Wi-Fi at 2402 MHz showed 21 dB up at 2386 when
tuned to 2393), which is why 16 and 8 MHz are captured at 40 MS/s (39 kHz bins, about the
pixels' width at 8 MHz) and 4 and 2 MHz keep to the middle of their capture. At 40 MS/s the
filters' slow roll-off past +-18 MHz does the same at the edges (a mast at 2675 MHz showed at
2635 when tuned to 2655), so 40 MHz is captured at 80 MS/s; each pixel takes the strongest bin under it. Below 16 MHz, time resolution
grows with the frequency resolution: the screen is redrawn after 2, 4 or 8 captured rows
instead of one, each taking its share of the peak hold's bursts (8 at 16 MHz, 1 each at
2 MHz), so the waterfall scrolls at about the same speed with more detail.
Scrolling past the captured span retunes the radio, and the waterfall history is shifted and
rescaled to stay lined up.

The spans of 40 MHz and wider sweep: each row retunes across the view in steps of at most
32 MHz (2 at 40 MHz, 3 at 80, 5 at 160, 32 for the whole 1000 MHz from 1800 to 2800), captures 4096 samples at
80 MS/s at each and keeps the middle of each step, inside the filters' passband, so the band
plan shows at a glance (the band names fit). The library waits 3 ms after every retune by
default; captures right after a 40 MHz hop matched settled ones with no wait at all
(passband within 0.15 dB, clock harmonics 0.5 dB, also across the 5/6 divider at 2210 MHz), so
the app sets 200 us (``esp_sdr_set_retune_settle()``) and a retune takes about 1 ms. The P
bursts of a row are shared out over the steps (at least one each), so a row is as much work
as elsewhere and the scroll speed falls smoothly with the width (rows a second):

==========  ====  ===  ===  ===  ===  ===  ===  ===  ===  ===
Span (MHz)  1000  640  320  160   80   40   16    8    4    2
==========  ====  ===  ===  ===  ===  ===  ===  ===  ===  ===
``P8``         6    9   15   21   23   25   25   49   85  115
``P1``         6    9   15   29   50   64   99  112  113  116
==========  ====  ===  ===  ===  ===  ===  ===  ===  ===  ===

At 320 MHz and wider a step gets one burst, so short bursts are often missed there, and the
steps show as a faint 1 to 2 dB ripple. The steady-line mask is off while sweeping; ``x``
patches the clock harmonics and each step's LO spurs instead, including those the receiver
makes when tuned below about 1842 MHz (up to +14 dB, 2.5 to 3.3 MHz below the LO). ``$settle``
changes the wait after each retune, for experiments.

The FFTs are esp-dsp's ESP32-S3 kernel (west project ``esp-dsp``, checked against the C
FFT at boot): 0.4 ms for 1024 points, and a burst takes about 2.9 ms in all. A redraw takes
about 25 ms (the display runs at 40 MHz SPI), so rows are captured until a frame is due, at
most 15 a second, and the screen then scrolls by all of them at once; a key press redraws at
once. At 16 MHz that is about 23 rows a second with peak hold (``P8``) and 57 with one burst a
row (``P1``); the table under ``p`` has the rest.

Held upright (either edge down, past about 60 degrees; back to landscape within about 30 degrees
of upright, so it does not flip back and forth), the BMI270 accelerometer switches to a portrait
layout laid out as the landscape one: the settings on two lines, the scale, and the trace with
the Wi-Fi and Bluetooth LE labels on top, the waterfall under them with the newest row at the
top. The panel scrolls the waterfall in hardware, so a new row costs one 135-pixel column
instead of a redraw: about 29 rows a second at ``P8`` and 80 at ``P1``, against 23 and 57 in
landscape. The arrow keys follow the
rotation (left and right still scroll, up and down still step the span as seen on the screen);
``v`` turns the automatic rotation off and on. The key list stays in landscape.

Wi-Fi is bursty and one burst is only 256 us (16 MS/s) to 51 us (80 MS/s), so by default
each row is the per-bin maximum over 8 bursts (peak hold, ``P8`` in the header). ``p`` sets
the speed: each press halves the bursts per row, 8, 4, 2, 1 (one burst's plain average), then
wraps round to the slow end, 32 and 16. Fewer bursts scroll faster but miss more of short
bursts; 16 and 32 catch more and hold more seconds of history. Rows a second at 16 MHz, in
landscape (portrait is a little faster):

==========  ===  ===  ===  ===  ===  ===
Bursts       32   16    8    4    2    1
==========  ===  ===  ===  ===  ===  ===
Landscape     8   13   23   36   44   57
Portrait      9   17   29   44   62   80
==========  ===  ===  ===  ===  ===  ===

The top line shows the view's centre frequency, span, gain and colour scale; under
it a frequency scale with ticks (taller on whole MHz) and their values in MHz where they
fit. A white mark under the scale is the view centre; a red one is where the radio is tuned,
which has a narrow artefact of its own (LO leakage, patched over, with a skirt of about
+-110 kHz) to ignore. Zoomed in to 8, 4 or 2 MHz the radio tunes just below the view instead
(offset tuning, 1 MHz below), so that line is off the screen. The
receiver also makes a spur that moves with the LO, at LO + (LO - 2400 MHz) / 3 (2386.67 MHz
when tuned to 2390): offset tuning goes above the view instead when below would put it on
the screen. Scrolling keeps the tuning while it still serves (the view inside the capture,
the LO and its spur off it), so at 4 and 2 MHz most steps do not retune. Magenta marks
are the board's own clock harmonics: steady, narrow carriers exactly on multiples of the
40 MHz crystal and the 48 MHz USB clock (2352, 2360, 2400, 2440, 2448 and 2496 MHz measured; 2448 on battery too).

There are more such lines than any list or formula covers (2416, 2504 and others at fixed
frequencies, several that move with the LO), but each repeats to the bin and within a few
tenths of a dB. So ``x`` (on at start, ``X`` in the header) learns them: for the current
tuning each FFT bin keeps the lower envelope of its level over a couple of seconds, and a bin
whose envelope stands more than 2 dB above its neighbours' (9 to 24 bins either side) is
divided by that excess. A steady line drops to the floor around it; a burst on top of it, or
anywhere else, is left alone (the busiest rows came out the same with it on and off). The
LO's own line is bridged over 120 kHz either side, and its skirt, a smooth bump of +3 dB at
125 kHz falling to +1 dB at 500 kHz, is flattened the same way against the ring from 600 to
1200 kHz out. After a retune the mask relearns over 40 rows; the last 24 tunings' masks are
kept (their 64 strongest bins), so going back to one applies its mask at once. A real steady
carrier is masked too: ``x`` turns the mask off. At 40 and
80 MS/s the trace and waterfall are dimmed beyond about 18 MHz either side of it, where the
analog filters cut reception off; the automatic colour floor follows only the part in between,
so the attenuated edges do not pull it down and wash out the middle. Below come a 30 pixel spectrum trace and the
waterfall, newest row on top. A ``!`` after the frequency means the PLL reported the
frequency outside its tuning window (``-ERANGE``).

Build and flash
***************

The board is ``m5stack_stamps3/esp32s3/procpu`` with this repository's
``m5stack_cardputer_adv`` shield (added automatically for that board)::

   west build -b m5stack_stamps3/esp32s3/procpu esp-sdr-zephyr/apps/waterfall
   west flash

The Stamp-S3A flashes over its USB Serial/JTAG port; if it does not enumerate, enter download
mode: power switch off, hold G0, power on.

Keys
****

======  ===========================================
Key     Action
======  ===========================================
Esc     show / hide the key list on screen (the `` ` `` key; any
        other key also closes it)
``,``   scroll left by 1/8 screen (left arrow)
``/``   scroll right by 1/8 screen (right arrow)
``;``   narrower span, down to 2 MHz (up arrow)
``.``   wider span, up to 1000 MHz swept (down arrow)
``h``   back to 2440 MHz, 16 MHz span
``1``   to ``9`` and ``0``: jump to a band (table below)
``-``   darker: colour floor +2 dB (manual)
``=``   brighter (``+``): colour floor -2 dB (manual)
``[``   colour range -5 dB
``]``   colour range +5 dB
``a``   automatic colour floor on / off
``w``   2.4 GHz Wi-Fi channel overlay on / off
``p``   speed: bursts per row (peak hold), 8 (at start), 4, 2, 1, 32, 16
``x``   mask the receiver's own steady lines on / off (on at start)
``n``   gain -1 (default fixed 42; AGC hides bursty Wi-Fi)
``m``   fixed gain +1
``g``   AGC
``v``   automatic portrait layout when held upright on / off
======  ===========================================

Wi-Fi channels 1 to 12 are always labelled at the top of the spectrum trace, each number
centred over a short tick at the channel centre (labels that would collide are skipped).
Below them, the Bluetooth LE advertising channels are labelled ``B37``, ``B38`` and ``B39``
in cyan (2402, 2426 and 2480 MHz), where BLE devices announce themselves in short bursts; small
dark cyan ticks under them mark the 37 data channels (every 2 MHz from 2404 to 2478 MHz),
which connected devices hop between. The
overlay adds dotted lines through the trace and waterfall (cyan for Bluetooth) and tints the
20 MHz of the non-overlapping Wi-Fi channels 1, 6 and 11 in the trace.

The same row marks the other services in the tuning range, which never overlap 2.4 GHz Wi-Fi:
each band is a coloured line through the middle of the row, broken around its name where it
fits, orange where masts transmit (``4G MAST``), blue where users' phones, tablets and hotspots do (``4G USER``), violet
where both take turns (TDD, ``4G/5G`` for B40 and B38), pink DECT, grey satellites. Where a readable name does not fit
(the band mostly off the screen) its short code shows instead, for example ``B1 DL``. The number keys jump to the bands' centres, in order:

===  ==================================  ==================================
Key  Europe/UK (default)                 US (``$region us``)
===  ==================================  ==================================
1    B3 DL, 4G masts, 1805-1880          B25 UL, phones, 1850-1915
2    DECT cordless phones, 1880-1900     DECT 6.0, 1920-1930
3    B1 UL, 3G/4G phones, 1920-1980      B25 DL, masts, 1930-1995
4    B1 DL, 3G/4G masts, 2110-2170       B66 DL, masts, 2110-2200
5    B40, 4G/5G TDD, 2300-2400           B30 UL, phones, 2305-2315
6    ISM: Wi-Fi, BLE, Zigbee, 2400-2483  SXM, SiriusXM satellite, 2320-2345
7    GSTAR, Globalstar down, 2483-2500   B30 DL, masts, 2350-2360
8    B7 UL, 4G phones, 2500-2570         ISM, 2400-2483
9    B38, 4G TDD, 2570-2620              GSTAR, Globalstar down, 2483-2500
0    B7 DL, 4G masts, 2620-2690          B41, 4G/5G TDD, 2496-2690
===  ==================================  ==================================

The allocations are approximate and vary by country and operator.

The view centre is limited to 1800 to 2800 MHz. The keys also work typed on the USB console.
With ``$text on`` (below) the console prints a ``spec`` line every 2 s: the view, then the 240
pixels' maximum in dB since the previous line, and the rows per second every 5 s. They are off
by default: the screen hitches each time one goes out, a pause every few seconds (the binary
frames, larger and more frequent, do not do it; the cause is not known). Every key press is logged with its raw TCA8418 event on the console,
which helps if the key map needs adjusting.

Host interface
**************

For development the USB console also takes commands, one per line starting with ``$``; each
answers with one ``ok <command> ...`` or ``err <command> ...`` line (``stat ...`` for
``$status``):

====================  ===========================================================
Command               Action
====================  ===========================================================
``$help``             list the commands
``$status``           the state: LO, view, span, rate, zoom, gain, peak hold, the
                      orientation and accelerometer (0.1 m/s^2), ...,
                      and the main loop's longest pass since the last ``$status``
``$bins [n|on|off]``  send the next n rows' (default 1) full 1024-bin spectra
``$iq [n|off]``       send the next n bursts' raw I/Q (4096 samples each)
``$text on|off``      the periodic ``spec`` and status lines (default off)
``$screen on|off``    drawing; without it rows come up to 4 times faster
``$tune <kHz>``       the view centre (retunes as needed)
``$lo <MHz>``         the LO alone, the view staying put when it fits
``$span <MHz>``       1000, 640, 320, 160, 80, 40 (swept), 16, 8, 4 or 2
``$gain <n|auto>``    fixed gain table index or AGC
``$peak <n|on|off>``  bursts per row: 1, 2, 4, 8, 16 or 32 (on: 8, off: 1)
``$hide on|off``      the steady-line mask (display only)
``$region eu|us``     which band table labels the bands and the number keys
``$post on|off``      ``$bins`` frames after the mask, as drawn, instead of before it
``$rotate <how>``     ``auto`` (the accelerometer), or fixed: ``land``, ``left`` or
                      ``right`` (that edge down, portrait)
====================  ===========================================================

Spectra and I/Q come back as binary frames between the text lines: ``ESDR``, a 28-byte
little-endian header (type, sequence number, uptime, LO, sample rate, gain, bursts), the
payload, then a CRC-32 (see ``struct frame_hdr`` in ``src/main.c``). Spectra are the rows as
captured, before the LO patch and the steady-line mask (after them with ``$post on``): int16
hundredths of a dBFS from -fs/2 up,
the LO in bin 512. I/Q samples are int16 pairs in raw units (-512 to 511).

``tools/wf_serial.py`` (pyserial; numpy and matplotlib for the data) does the framing:

.. code-block:: console

   tools/wf_serial.py status
   tools/wf_serial.py cmd '$span 2' '$tune 2448000'   # settings stay after the tool exits
   tools/wf_serial.py bins -n 200 -o rows.npz --png rows.png
   tools/wf_serial.py iq -n 4 -o iq.npz
   tools/wf_serial.py --setup '$lo 2443' lines        # steady narrow lines, e.g. the spurs
   tools/wf_serial.py live                            # waterfall in a window

Captures pause the screen unless ``--keep-screen`` is given. The tool opens the port with DTR
and RTS raised: the USB Serial/JTAG resets the chip on RTS without DTR, which a plain open with
both low passes through.
