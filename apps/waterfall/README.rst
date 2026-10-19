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

The FFTs are esp-dsp's ESP32-S3 kernel (west project ``esp-dsp``, checked against the C
FFT at boot): 0.4 ms for 1024 points, and a burst takes about 2.9 ms in all. A redraw takes
about 25 ms (the display runs at 40 MHz SPI), so rows are captured until a frame is due, at
most 15 a second, and the screen then scrolls by all of them at once; a key press redraws at
once. At 16 MHz that is about 24 rows a second with peak hold and 70 without (34 and 115 with
the screen paused).

Held upright (either edge down, past about 60 degrees; back to landscape within about 30 degrees
of upright, so it does not flip back and forth), the BMI270 accelerometer switches to a portrait
layout laid out as the landscape one: the settings on two lines, the scale, and the trace with
the Wi-Fi and Bluetooth LE labels on top, the waterfall under them with the newest row at the
top. The panel scrolls the waterfall in hardware, so a new row costs one 135-pixel column
instead of a redraw: about 30 rows a second with peak hold and 95 without, against 24 and 70
in landscape. The arrow keys follow the
rotation (left and right still scroll, up and down still step the span as seen on the screen);
``v`` turns the automatic rotation off and on. The key list stays in landscape.

Wi-Fi is bursty and one burst is only 256 us (16 MS/s) to 51 us (80 MS/s), so by default
each row is the per-bin maximum over 8 bursts (peak hold, ``P8`` in the header). ``p`` sets
the speed: each press halves the bursts per row, 8, 4, 2, 1 (one burst's plain average), then
wraps round to the slow end, 32 and 16. Fewer bursts scroll faster but miss more of short
bursts; 16 and 32 catch more and hold more seconds of history. Rows a second at 16 MHz:

==========  ===  ===  ===  ===  ===  ===
Bursts       32   16    8    4    2    1
==========  ===  ===  ===  ===  ===  ===
Landscape     8   13   24   38   55   75
Portrait     10   18   30   50   75  100
==========  ===  ===  ===  ===  ===  ===

The top line shows the view's centre frequency, span, gain and colour scale; under
it a frequency scale with ticks (taller on whole MHz) and their values in MHz where they
fit. A white mark under the scale is the view centre; a red one is where the radio is tuned,
which has a narrow artefact of its own (LO leakage, patched over, with a skirt of about
+-110 kHz) to ignore. Zoomed in to 8, 4 or 2 MHz the radio tunes just below the view instead
(offset tuning: 250 kHz below at 8 MHz, whose last 250 kHz on the right are then beyond the
capture and stay dark; 1 MHz below at 4 and 2 MHz), so that line is off the screen. Magenta marks
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
``h``   back to 2440 MHz, 16 MHz span
``-``   colour floor -2 dB (manual)
``=``   colour floor +2 dB (manual)
``[``   colour range -5 dB
``]``   colour range +5 dB
``a``   automatic colour floor on / off
``w``   2.4 GHz Wi-Fi channel overlay on / off
``p``   speed: bursts per row (peak hold), 8 (at start), 4, 2, 1, 32, 16
``x``   hide the board's clock harmonics on / off (off at start)
``n``   gain -1 (default fixed 42; AGC hides bursty Wi-Fi)
``m``   fixed gain +1
``g``   AGC
``v``   automatic portrait layout when held upright on / off
======  ===========================================

Wi-Fi channels 1 to 14 are always labelled at the top of the spectrum trace, each number
centred over a short tick at the channel centre (labels that would collide are skipped).
Below them, the Bluetooth LE advertising channels are labelled ``B37``, ``B38`` and ``B39``
in cyan (2402, 2426 and 2480 MHz), where BLE devices announce themselves in short bursts; small
dark cyan ticks under them mark the 37 data channels (every 2 MHz from 2404 to 2478 MHz),
which connected devices hop between. The
overlay adds dotted lines through the trace and waterfall (cyan for Bluetooth) and tints the
20 MHz of the non-overlapping Wi-Fi channels 1, 6 and 11 in the trace.

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
``$span <MHz>``       80, 40, 16, 8, 4 or 2
``$gain <n|auto>``    fixed gain table index or AGC
``$peak <n|on|off>``  bursts per row: 1, 2, 4, 8, 16 or 32 (on: 8, off: 1)
``$hide on|off``      patch over the clock harmonics (display only)
``$rotate <how>``     ``auto`` (the accelerometer), or fixed: ``land``, ``left`` or
                      ``right`` (that edge down, portrait)
====================  ===========================================================

Spectra and I/Q come back as binary frames between the text lines: ``ESDR``, a 28-byte
little-endian header (type, sequence number, uptime, LO, sample rate, gain, bursts), the
payload, then a CRC-32 (see ``struct frame_hdr`` in ``src/main.c``). Spectra are the rows as
captured, before the LO and clock-harmonic patches: int16 hundredths of a dBFS from -fs/2 up,
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
