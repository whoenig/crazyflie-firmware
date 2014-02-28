##CrazyFlie Firmware (Latency measurement)

This project contains the source code for the Crazyflie firmware. More information can be found on the
[Bitcraze wiki](http://wiki.bitcraze.se/projects:crazyflie:index)

The folder contains the firmware to test the latency of the radio and a testing script.
It can be used in the following way:
`make`
`make cload`
`python3 scripts/latency.py`

The firmware is a minimal version polling in a loop, not depending on FreeRTOS.
The crayzradio.py driver is based on the original firmware, but updated for python3 and with improved handling of the with clause.
