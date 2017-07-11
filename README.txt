An ATMega328p controller receiver for Solar Survey 200R radio messages.

Installation
============

* mkdir cmake-build-debug
* cd cmake-build-debug
* cmake ..

NOTES
=====

- in CMakeLists.txt
  - set the AVR_UPLOADTOOL_MCU to atmega328 OR atmega328p depending on chip.
  - Leave AVR_MCU as atmega328p in all cases (so far)

Connecting programmer to board
==============================

- to ping the programmer, without connecting to the board:

    avrdude -p atmega328 -c usbasp -P usb

  This should make the LED on the programmer blink (though avrdude will report an error).

- important: disable 5v and 3.3v power from the programmer so that it doesn't try to power the board (which is powered by the RP)

- connect programmers to our board, matching up the labels on the controller with the labels on the board. Match up VCC with 5V.

- Test reading the low and high byte fuses:

    avrdude -p atmega328 -c usbasp -P usb -U lfuse:r:low_fuse_val.hex:h
    avrdude -p atmega328 -c usbasp -P usb -U hfuse:r:high_fuse_val.hex:h

  These are bitfields that control all sorts of aspects of how the board works.

  For example, if these fuses are incorrect then you'll use the wrong clock (i.e. the one on the board) and the LEDs will blink very slowly.

- Compile the program

     make rx200r

  This will ultimately spit out a .hex file

Upload the program
==================

- make upload_rx200r

Tuning the radio
================

The function alpha_rx_tune() tries to find the right settings for the radio. To do this, it prints out information to the terminal. To see this output you need to ssh to the solarpi box and use picocom to watch the terminal.

- install picocom

    sudo apt-get install picocom

- monitor the terminal with picocom

    sudo picocom -b 9600 /dev/ttyAMA0

Interpreting the tuning parameters
----------------------------------

LNA = Low Noise Amplifier
DRSSI = Digital Received Signal Strength Indication https://en.wikipedia.org/wiki/Received_signal_strength_indication
Base-band bandwidth: bandwidth around central frequency which we will listen to.

We are trying to find the lowest DRSSI (i.e. the quietest we can listen to) that
isn't picking up background noise/radiation/etc. And we want the narrowest
baseband bandwidth that we can get away with. We want the lowest gain possible.

To interpret the table, look high and to the right for 0%. This gives you
narrowest baseband width and lowest drssi and minimizes gain. You'll have
several options in all likelihood...pick one and try it.

You can assess the quality of your settings by trying to talk to the radio on
the 200R solar meter.

Testing the receiver/tuning parameters
--------------------------------------

First, in your main.c, enable the alpha_rc_monitor_rssi() function. This will
dump rssi readings to the terminal which you can monitor with picocom.

Turn on the meter: hold down the first two buttons (two leftmost buttons) until
you get a beep. This also turns it off.

To start transmitting: Hold down the first and last button until you get a beep.
You'll see a flashing inverted triangle in the lower left.

At this point, if things are working, your picocom output will show a stream of
rssi percentage readings every second or so (which is how often the sensor
transmits). If you walk away from the box with sensor, you will see reduced
strength.

NOTES
=====

- if you need to disable agetty on raspbian

   sudo systemctl stop serial-getty@ttyAMA0.service

TODO
====

Our ansible playbook needs to "disable" the serial port console, not just stop it.
