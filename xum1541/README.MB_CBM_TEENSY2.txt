What is MB_CBM_TEENSY2 ?
========================

A carrier board for Teensy2 with DIN & GPIB connectors and +5V buffers able to drive Commodore floppy drives.

This project has been made for Musée Bolo (https://www.museebolo.ch).

How to program to Teensy2
=========================

Install teensy-loader-cli package on your debian linux system.

Then execute, the following commands:
  teensy_loader_cli --mcu=TEENSY2 -s -v xum1541-MB_CBM_TEENSY2-v08.hex

