DavisRFM69 Library
-------------------

This library is a modified version of the [DeKay's DavisRFM69 library](https://github.com/dekay/DavisRFM69) that enables reception of weather data from a Davis Instruments Integrated Sensor Suite (ISS) weather station.

## Background

The Davis Instruments ISS is a solar powered and battery-backed set of outdoor weather sensors monitored by a PIC microcontroller.  It uses a TI CC1020 RF Transmitter chip to send the weather data it collects back to a Davis Vantage VP2 or Vantage Vue console located indoors.  The ISS transmissions have been reverse engineered, and this has allowed receivers based on the TI CC1110 chip (amongst others) to receive its transmissions.

The drawback of the CC1110 is that it has its own embedded microcontroller [that requires separate hardware to program](http://madscientistlabs.blogspot.ca/2012/01/troubles-with-im-me-and-goodfet.html).  The simpler CC1101 does not have this controller and can be commanded over a SPI bus, but it is difficult to find this board in a 915 MHz flavor as used by Davis ISS units in North America.  Beware EBay units saying they are 915 MHz: they likely actually operate at 433 MHz and are of no use.  [Ask me how I know...](http://madscientistlabs.blogspot.ca/2013/04/dead-end.html)

The one potential bright spot in the CC11xx story is the [RFBee](http://www.seeedstudio.com/depot/rfbee-v11-wireless-arduino-compatible-node-p-614.html).  It couples a TI CC1101 with an Arduino and is easy to use and program, but it is based on an Atmega 168 whose limited RAM and FLASH have hampered efforts to implement a full blown emulation of the Davis indoor console.

The new kid on the block is the **RFM69 module from HopeRF**.  

This module is inexpensive and can be bought either standalone or integrated on a ["Moteino"](http://lowpowerlab.com/blog/category/moteino/) Atmega 328 Arduino clone from LowPowerLabs.  This library demonstrates that the RFM69 is flexible enough to receive transmissions from the TI transmitter chip in the ISS.

## Features
This library sniffs the wireless packets transmitted from a Davis ISS.  
This library has been developed on a Moteino R3 [(see here for the new R4 version)](http://lowpowerlab.com/shop/Moteino-R4)
fitted with an RFM69W (Semtech SX1231/SX1231) transceiver module, but works also with standalone mradio module and any otrher microcontroller on SPI bus.

##Miscellaneous / Possible Issues

Reception quality has been greatly improved in this release.  ~~There looks to be a bug where the hop-ahead code has broken, but I expect that will be fixed soon~~ I am getting around 99% good packets now.  Please let me know if you find any issues.

##Sample Usage
[VP2.ino](https://github.com/cotestatnt/DavisRFM69/blob/master/examples/VP2/VP2.ino) is an emulation of a "custom" console


##Blog Writeups
[Davis Console Emulation](http://madscientistlabs.blogspot.ca/2014/02/build-your-own-davis-weather-station_17.html) combines ISS Reception capabilities along with hookups to sensors for indoor monitoring of temperature, pressure, and humidity.

##Why
I started playing around with my VP2 Wireless console when I discovered its little expansion port tucked away in the back.  Its purpose is primarily for connection of an exorbitantly priced datalogger that is little more than a one dollar flash chip.  After figuring out how to connect [first a serial interface](http://madscientistlabs.blogspot.ca/2011/01/davis-weatherlink-software-not-required.html) and then [building my own datalogger](http://madscientistlabs.blogspot.ca/2011/10/build-your-own-davis-console-datalogger.html), I figured out the [wireless protocol between the ISS and the console](http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html) and put together the first standalone receive using an [IM-ME Pretty Pink Pager](http://madscientistlabs.blogspot.ca/2012/04/achievement-unlocked-im-me-weather.html).

I learned a lot by doing this and I like to think that opening up the console has been an overall win for Davis.  I also consider this to be MY data, and I want access to it ([Davis' failed attempts to shut this down notwithstanding](http://meteo.annoyingdesigns.com/DavisSPI.pdf)).

And, just because.
