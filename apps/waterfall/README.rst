.. SPDX-License-Identifier: GPL-3.0-or-later

ESP-SDR waterfall (M5Stack Cardputer-Adv)
#########################################

A receive-only spectrum and waterfall on the Cardputer-Adv's 240x135 display, in plain C:
no MicroPython, no LVGL, no PSRAM (the Stamp-S3A has none).

Each row is a Welch average of Hann-windowed 1024-point FFTs over one 4096-sample burst at
16 MS/s (or 40 MS/s). The screen shows a view of that span, 16 MHz (or 40 MHz) wide at x1
down to 2 MHz (or 5 MHz) at x8; each pixel takes the strongest bin under it. Zooming stretches
time as well: each row is drawn zoom pixels tall, so the waterfall runs that much faster.
Scrolling past the captured span retunes the radio, and the waterfall history is shifted and
rescaled to stay lined up.

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
``r``   rate 16 / 40 MS/s
``-``   colour floor -2 dB (manual)
``=``   colour floor +2 dB (manual)
``[``   colour range -5 dB
``]``   colour range +5 dB
``a``   automatic colour floor on / off
``n``   fixed gain -1
``m``   fixed gain +1
``g``   gain back to AGC
======  ===========================================

The view centre is limited to 1800 to 2800 MHz. Every key press is logged with its raw TCA8418 event
on the console, which helps if the key map needs adjusting.
