.. SPDX-License-Identifier: GPL-3.0-or-later

ESP-SDR waterfall (M5Stack Cardputer-Adv)
#########################################

A receive-only spectrum and waterfall on the Cardputer-Adv's 240x135 display, in plain C:
no MicroPython, no LVGL, no PSRAM (the Stamp-S3A has none).

Each row is a Welch average of Hann-windowed 256-point FFTs over one 4096-sample burst at
16 MS/s (or 40 MS/s), the middle 240 bins shown: about 15 MHz (or 37.5 MHz) around the LO.
The top line shows the frequency, rate, gain and colour scale; under it a 30 pixel spectrum
trace, then the waterfall, newest row on top. A ``!`` after the frequency means the PLL
reported the frequency outside its tuning window (``-ERANGE``).

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
``,``   frequency -1 MHz (left arrow)
``/``   frequency +1 MHz (right arrow)
``;``   frequency +10 MHz (up arrow)
``.``   frequency -10 MHz (down arrow)
``h``   back to 2440 MHz
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

Tuning is limited to 1800 to 2800 MHz. Every key press is logged with its raw TCA8418 event
on the console, which helps if the key map needs adjusting.
