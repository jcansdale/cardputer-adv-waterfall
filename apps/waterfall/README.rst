.. SPDX-License-Identifier: GPL-3.0-or-later

ESP-SDR waterfall (M5Stack Cardputer-Adv)
#########################################

A receive-only spectrum and waterfall on the Cardputer-Adv's 240x135 display, in plain C:
no MicroPython, no LVGL, no PSRAM (the Stamp-S3A has none).

Each row is a Welch average of Hann-windowed 1024-point FFTs over one 4096-sample burst at
16, 40 or 80 MS/s. The screen shows a view of that span, as wide as the sample rate at x1
(16, 40 or 80 MHz) down to an eighth of it at x8; the radio's analog filters only pass about
+-10 MHz at 16 MS/s and, in the PHY's 40 MHz mode used at 40 and 80 MS/s, about +-18 MHz,
so the outer parts of a wide view are attenuated; each pixel takes the strongest bin under it. Zooming stretches
time as well: each row is drawn zoom pixels tall, so the waterfall runs that much faster.
Scrolling past the captured span retunes the radio, and the waterfall history is shifted and
rescaled to stay lined up.

Wi-Fi is bursty and one burst is only 256 us (16 MS/s) to 51 us (80 MS/s), so by default
each row is the per-bin maximum over 8 bursts (peak hold, ``P`` in the header); ``p`` turns
it off for a plain average.

The top line shows the view's centre frequency, rate, zoom, gain and colour scale, with
frequency ticks under it (taller on whole MHz); then a 30 pixel spectrum trace and the
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
``,``   scroll left by 1/8 screen (left arrow)
``/``   scroll right by 1/8 screen (right arrow)
``;``   zoom in, up to x8 (up arrow)
``.``   zoom out, down to x1 (down arrow)
``j``   frequency -10 MHz
``k``   frequency +10 MHz
``h``   back to 2440 MHz at x1
``r``   rate 16 / 40 / 80 MS/s
``-``   colour floor -2 dB (manual)
``=``   colour floor +2 dB (manual)
``[``   colour range -5 dB
``]``   colour range +5 dB
``a``   automatic colour floor on / off
``w``   2.4 GHz Wi-Fi channel overlay on / off
``p``   peak hold over 8 bursts on / off (on at start)
``n``   gain -1 (default fixed 42; AGC hides bursty Wi-Fi)
``m``   fixed gain +1
``g``   AGC
======  ===========================================

The Wi-Fi overlay marks channels 1 to 14 with dotted lines and their numbers (labels that
would collide are skipped) and tints the 20 MHz of the non-overlapping channels 1, 6 and 11
in the spectrum trace.

The view centre is limited to 1800 to 2800 MHz. The keys also work typed on the USB console, and every 2 s the console prints a
``spec`` line: the view, then the 240 pixels' maximum in dB since the previous line. Every key press is logged with its raw TCA8418 event
on the console, which helps if the key map needs adjusting.
