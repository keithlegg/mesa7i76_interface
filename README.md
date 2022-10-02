# mesa7i76_interface








------------------------------------------

NOTES ON MEGA128 CHIP - ITS QUIRKY 

- One fuse that every ATmega128 user will become familiar with very early on is the ATmega103 Compatibility Mode fuse.

All '128's come from the factory with this fuse pre-programmed. In that state, the chip will behave almost exactly as if it were actually an ATmega103 instead of an ATmega128.

Any software you write with the ATmega128 in in mind is 99% guaranteed to fail if you attempt to run it while the chip is in M103 compatibility mode... you'll have to at least modify that fuse.


- you can hook up a 1MHZ clock to XTAL 1 if you loose clock via bad fuses!!


- http://www.avrfreaks.net/forum/need-help-atmega-128-uart-programming-solved-m103c

- the ATmega128 has some special differences you need to be aware of. 

First look at section SPI Serial Programming Pin Mapping Table 127. Pin Mapping SPI Serial Programming. In some AVRs like the ATmega128 the USART0 pins are used for the ISP function, and the SPI (do not confuse this with the ISP acronym) MOSI and MISO pins are not used. In lots of other AVRs the SPI pins MOSI and MISO are used for the ISP function.

Then look at sections ATmega103 and ATmega128 Compatibility and ATmega103 Compatibility Mode for the M103C fuse information. The ATmega128 is shipped from ATMEL with the M103C fuse programmed (see Table 117. Extended Fuse Byte). So, every brand new ATmega128 ships from the factory with the M103C fuse programmed making your new chip default into an ATmega103 (an old obsolete processor). If you purchased a circuit board with an ATmega128 already soldered onto the board, then the board manufacturer may have already unprogrammed the M103C fuse. You need to make sure the M103C fuse is unprogrammed if you really want a working ATmega128 chip.

Since you are new to AVRs I just thought you should be aware of these special ATmega128 differences. For example, if the M103C fuse is still programmed you could write perfectly good ATmega128 program code that will fail because the AVR is still in ATmega103 mode.


- CHANGE FUSE BITS 

# DEFAULT FUSES  - WITHOUT 103 COMPATIBILITY MODE OR JTAG!!!! 
sudo avrdude -V -c usbtiny -p atmega128  -U lfuse:w:0xc1:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m 

# external xtal  - THIS SEEMS TO WORK!!
sudo avrdude -V -c usbtiny -p atmega128  -U lfuse:w:0xff:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m 
