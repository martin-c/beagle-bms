beagle-bms
==========
### A Linux-based real time system utilizing the LTC6804 battery stack monitor IC

This project is the result of my final lab for [ECEN5623 at CU Boulder](http://ecee.colorado.edu/~ecen5623/).
The BMS utilizes a [Beagle-xM](http://beagleboard.org/Products/BeagleBoard-xM) running [Arch Linux Arm](http://archlinuxarm.org/), as well as two [LTC6804 battery stack monitor](http://www.linear.com/product/LTC6804-1) ICs on demo boards. Communication is perfomed using a [LTC6820 demo board](http://www.linear.com/product/LTC6820) linked to a SPI interface on the Beagle-xM.

The software was written over a period of approximately 10 days during the summer of 2013, and as such may contain errors and is cosidered to be incomplete. It does however demonstate one possible method for implementing a driver to interface with the '6804 under Linux. If I had had more time I would consider moving some of this driver code into a kernel module instead of relying on SPIDEV.

A video demo of my project is available on [youtube here](http://www.youtube.com/watch?v=Ke4MwVpD0xs).
