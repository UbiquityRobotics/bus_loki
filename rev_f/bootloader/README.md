# Bootloader Notes

## Fuse Bits

There are four configuration bytes that can be programmed:

* Lock bits (e.g. `lock`)
* Extended fuse bits (e.g. `efuse`)
* High fuse bits (e.g. `hfuse`)
* Low fuse bits (e.g. `lfuse`)

These bits are described below:

### Boot Loader Lock Bits

The boot loader lock bits have the following format:

        11abcdef

where:

* `ab` is BLB12/BLB11 the bootloader area lock bits.  The values are:
   
  * `11`: Bootloader area is both readable and writable.
  * `10`: Bootloader section is not writable.  (The data sheet does not say
    whether or not bootloader section is readable; presumably it is.)
  * `01`: Application code can not read bootload section.  (The data sheet does
     not say if thw bootloader section is writable;  Presuably it is.)
  * `00`: Bootloader area can not be written.  Application code can not read
    from bootloader section.  (Read data sheet about interrupt vectors in
    application section.)

* `cd` is BLB02/BLB02 the application area lock bits.  The values uare:

  * `11`: Application area is both readable and writable
  * `10`: Application area is readable but not writable
  * `01`: Bootloader code can not read application area
  * `00`: Application area is not writable.  Bootloader code can not read
    from the application section.

* `ef` is the lock bit mode.  The values are:

  * `11` No memory lock features enabled.
  * `10` Flash and EEPROM programming is disabled.  Fuse bits ard locked.
  * `00` Verification of flash and EEPROM is disabled.

### Extended Fuse Bits:

The extend fuse bits are:

        11111abcd

where:

* `abc` specifies the brown out level detector trigger.


### High Fuse Bits:

The high fuse bits are:

        abcdeffg

where:

* `a`: `1` disables OCD (On Chip Debugging) and `0` enables On Chip Debugging
* `b`: `1` disables JTAG and `0` enables JTAG
* `c`: `1` disables SPIEN (Serial Program and Data Downloading) and `0` enables
* `d`: `1` disables WTDON (watch dog timer) and `0` enables
* `e`: `1` disables EESAVE (EEPROM erased on chip erase) and `0` enables (EEPROM preserved)
* `ff` is BOOTSZ1/BOOTSZ0 specifies the boot loader size.  The values are:

  * `11`: for  512 words (256x Boot reset = 0x1FE00)
  * `10`: for 1024 words (256x Boot reset = 0x1FC00)
  * `01`: for 2048 words (256x Boot reset = 0x1F800)
  * `00`: for 4096 words (256x Boot reset = 0x1F000)

* `g`: `1`disables starting at the boot loader and `0` enables start at the boot loader.

The bootloader code says to set the bits as follows:

* Set the boot loader size to 1024.

This advice is ***WRONG***!!!

### Low Fuse biits:

The low fuse bits are:

        abccdddd

where:

* `a`: CKDIV8: Divide clock by 8
* `b`: CKOUT: Clock output
* `cc`: SUT/SUT1 Start up time
* `dddd`: CKSEL3/CKSEL2/CKSEL1/CKSEL0 Select clock source:

   * `1111-1000`: Low power crystal oscillator
   * `0111-0110`: Full swing crystal oscillator
   * `0101-0100`: Low frewquency crystal oscillator
   * `0011`: Internal 128KHz R Oscillator
   * `0010`: Calibrated Internal RC Oscillator
   * `0001': Reserved
   * `0000`: External clock


* It comments in stk600boot.c say `Program Boot Lock Mode 3 (program BootLock 11
  and BootLock 12 lock bits) // (leave them)`.  Presumeably this means BLB11 and
  BLB12, which is for the bootloader section.  It is unclear what either `program`
  or `leave them` means.  It could mean either `00` or `11`.


### Recommended fuse and lock values for bootloader

* unlock: `1111 1111`: Everything open:
* lock:   `0000 1111`: Bootloader locked down
* efuse:  `1111 1101`: Probably does not matter
* hfuse:  `1101 1000`: ODD off, JTAG off, SPIN on, TD off, EEPROM erased, BOOTSZ=1024,
  BOOTRST enabled.
* lfuse:  `1111 1111`: CKDIV8 off, CKOUT off, SUT?, CKSEL=Low power crystal oscialltor

## `avrdude` Commands

To download the bootloader:

        # Program the lock bits (must have -e option to force chip erase):
        avrdude -p atmega2560 -P usb -c stk500v2 -C ./avrdude.conf -e -U lock:w:0x3F:m
        # Program the fuses:
        avrdude -p atmega2560 -P usb -c stk500v2 -C ./avrdude.conf -U efuse:w:0xFD:m -U hfuse:w:0xD8:m -U lfuse:w:0xFF:m
        # Program the bootloader:
        avrdude -p atmega2560 -P usb -c stk500v2 -C ./avrdude.conf -U flash:w:stk500boot.hex
        # Lock the bootloader down (do *NOT* use -e option -- it will erase bootloader):
        avrdude -p atmega2560 -P usb -c stk500v2 -C ./avrdude.conf -U lock:w:0x0F:m

## Comments

* The `Makefile` that ships with the `stk500boot.c` is pretty bad.  It was necessary
  to comment out all of the architectures other than `mega2560` to get it to work.
  The real problem was that the bootloader address getting set incorrectly.  For
  the meg2560, the correct address is `0x3E000` (= 2 &times 0x1F000 for 4096 bootloader).

* The Rev. D Loki needs to have a RasPi2 plugged into to provide 3.3V for the level
  shifting electronics.

* The new Chinese AVRISP mk II programmer from china needs to be ununplugged from
  the ISP connector after the bootloader programming.  On the Rev. D, the programmer
  stalls until the ISP connector is unplugged.

