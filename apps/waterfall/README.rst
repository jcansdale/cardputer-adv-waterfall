.. SPDX-License-Identifier: GPL-3.0-or-later

ESP-SDR waterfall (M5Stack Cardputer-Adv)
#########################################

A receive-only spectrum and waterfall on the Cardputer-Adv's 240x135 display, in plain C:
no MicroPython, no LVGL, no PSRAM (the Stamp-S3A has none).

Each row is a Welch average of Hann-windowed 1024-point FFTs over one 4096-sample burst at
16, 40 or 80 MS/s. The up and down keys step the screen's span through 80, 40, 16, 8, 4 and
2 MHz, and each span takes the narrowest sample rate that covers it, for the finest bins:
80 and 40 MHz capture at 80 and 40 MS/s, 16 MHz and below at 16 MS/s (8 to 2 MHz show part of
that capture). The radio's analog filters only pass about +-10 MHz at 16 MS/s and, in the
PHY's 40 MHz mode used at 40 and 80 MS/s, about +-18 MHz, so the outer parts of a wide view
are attenuated; each pixel takes the strongest bin under it. Below 16 MHz, time resolution
grows with the frequency resolution: the screen is redrawn after 2, 4 or 8 captured rows
instead of one, each taking its share of the peak hold's bursts (8 at 16 MHz, 1 each at
2 MHz), so the waterfall scrolls at about the same speed with more detail.
Scrolling past the captured span retunes the radio, and the waterfall history is shifted and
rescaled to stay lined up.

Wi-Fi is bursty and one burst is only 256 us (16 MS/s) to 51 us (80 MS/s), so by default
each row is the per-bin maximum over 8 bursts (peak hold, ``P`` in the header); ``p`` turns
it off for a plain average.

The top line shows the view's centre frequency, span, gain and colour scale; under
it a frequency scale with ticks (taller on whole MHz) and their values in MHz where they
fit. A white mark under the scale is the view centre; a red one is where the radio is tuned,
which has a narrow artefact of its own (LO leakage, patched over) to ignore. Magenta marks
are the board's own clock harmonics: steady, narrow carriers exactly on multiples of the
40 MHz crystal and the 48 MHz USB clock (2352, 2360, 2400, 2440, 2448 and 2496 MHz measured; 2448 on battery too);
``x`` patches them over with the neighbouring level (``X`` in the header), which would also hide
any real signal on them, so it starts off. At 40 and
80 MS/s the trace and waterfall are dimmed beyond about 18 MHz either side of it, where the
analog filters cut reception off. Below come a 30 pixel spectrum trace and the
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
``.``   wider span, up to 80 MHz (down arrow)
``j``   frequency -10 MHz
``k``   frequency +10 MHz
``h``   back to 2440 MHz, 16 MHz span
``-``   colour floor -2 dB (manual)
``=``   colour floor +2 dB (manual)
``[``   colour range -5 dB
``]``   colour range +5 dB
``a``   automatic colour floor on / off
``w``   2.4 GHz Wi-Fi channel overlay on / off
``p``   peak hold over 8 bursts on / off (on at start)
``x``   hide the board's clock harmonics on / off (off at start)
``n``   gain -1 (default fixed 42; AGC hides bursty Wi-Fi)
``m``   fixed gain +1
``g``   AGC
======  ===========================================

Wi-Fi channels 1 to 14 are always labelled at the top of the spectrum trace, each number
centred over a short tick at the channel centre (labels that would collide are skipped).
Below them, the Bluetooth LE advertising channels are labelled ``B37``, ``B38`` and ``B39``
in cyan (2402, 2426 and 2480 MHz), where BLE devices announce themselves in short bursts; small
dark cyan ticks under them mark the 37 data channels (every 2 MHz from 2404 to 2478 MHz),
which connected devices hop between. The
overlay adds dotted lines through the trace and waterfall (cyan for Bluetooth) and tints the
20 MHz of the non-overlapping Wi-Fi channels 1, 6 and 11 in the trace.

The view centre is limited to 1800 to 2800 MHz. The keys also work typed on the USB console, and every 2 s the console prints a
``spec`` line: the view, then the 240 pixels' maximum in dB since the previous line. Every key press is logged with its raw TCA8418 event
on the console, which helps if the key map needs adjusting.
