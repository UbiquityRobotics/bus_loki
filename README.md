# bus_loki

bus_loki is the code for the Loki robot.

## Installing the Arduino-Makefile Development Environment

The software environment for the bus_loki is more complicated
than we would like.  The bus_loki deliberately uses an ATmega2560
microcontroller to be as compatible as possible with the Arduino
community.

For a variety of reasons, the code is spread across multiple
repositories.  Furthermore, some of these other repositories
have other dependencies that are annoying (e.g. the
`bus_raspberry_pi` depends upon ATmega324P support, which
is not readily available.)  To be consistent across all
of these repositories, requires more effort.  In addition,
the Arduino-IDE does not play nice with version control systems.
So, we use a fairly popular alternative called `Arduino-Makefile`
which uses fairly standard Makefiles and standalone editors
(e.g. `vim` or `emacs`) to develop Arduino code that is
complatible with the Arduino IDE.

To set things up:

        cd .../catkin_ws/src
        git clone https://github.com/UbiquityRobotics/Arduino-Makefile.git
        git clone https://github.com/UbiquityRobotics/bus_common.git
        git clone https://github.com/UbiquityRobotics/bus_loki.git
        git clone https://github.com/UbiquityRobotics/bus_server.git
        git clone https://github.com/UbiquityRobotics/bus_slave.git
        # The following command downloads ~2GB of git version control stuff.
	# Even with a fast link it takes a while...
        git clone https://github.com/UbiquityRobotics/Arduino.git

Next, we need to bring over all the compilers and stuff to support
the AVR compilation environment:

        sudo apt-get avr-libc avrdude binutils-avr gcc-avr
	# We also need GNU make:
        sudo apt-get install build-essential

## Installing the Bootloader

The bootloader is installed onto Loki using the Arduino IDE.

For Loki Rev. C, the 2x3 ISP header N26 does not properly connect
to RESET.  The work around is to use patch cables to connect the
programmer header to various test points on Loki.  The table below
shows the six connections:

<BlockQuote>
<Table Border="1">
<TR> <TH>Signal</TH> <TH>Programmer</TH> <TH>Loki</TH> </TR>
<TR> <TD>MISO</TD>   <TD>Pin 1</TD>      <TD>TP8</TD>  </TR>
<TR> <TD>VREF</TD>   <TD>Pin 2</TD>      <TD>TP31</TD> </TR>
<TR> <TD>SCK</TD>    <TD>Pin 3</TD>      <TD>TP10</TD> </TR>
<TR> <TD>MOSI</TD>   <TD>Pin 4</TD>      <TD>TP9</TD>  </TR>
<TR> <TD>RESET</TD>  <TD>Pin 5</TD>      <TD>TP5</TD>  </TR>
<TR> <TD>GND</TD>    <TD>Pin 6</TD>      <TD>TP32</TD> </TR>
</Table>
</BlockQuote>

Perform the following steps:

1. Make sure you have the Arduino IDE installed:

        sudo apt-get install arduino

2. Start the Arduino IDE.

3. Using the [Tools] pull down menu,
   select [Board] => [Arduino Mega 2560 or Mega ADK]

4. Using the [Programmer] pull down menu, select the appropriate
   programmer.

5. Connect the programmer 2x3 connector to the Loki test pins as
   shown in the table above for Loki Rev. C.  For Rev. D and on,
   just connect to the 2x3 ISP connector N26.

6. Connect your programmer to your laptop/desktop.

7. Using the [Tools] pull down menu, select [Burn Bootloader].

That should do it.

When all of the dust settles, you should be able to do the following:

        cd .../catkin_ws/src/bus_loki
        make
        make upload       # Read below for more about uploading


## Compiling the Firmware

The actual code that implements the firmware starts in
`bus_loki.ino`.  The key line is:

        #define TEST TEST_...

The values for `TEST_...` are found in `../bus_server/bus_server.h`:

* `TEST_BUS_OUTPUT` outputs a stream of chacters on the serial port.
* `TEST_BUS_ECHO` is a strange bus debugging mode (too hard to explain.)
* `TEST_BUS_COMMAND` blinks an LED on the motor controller board.
* `TEST_BUS_BRIDGE` runs the software as a bus bridge.
* `TEST_BUS_LINE` runs the ROS Arduino Bridge protocol.

To compile the code, type:

        make

## Uploading the Firmware.

The `make` upload target expects you to have USB-serial cable
plugged into your laptop/desktop.  We use the DFRobot FTDI
Basic Breakout board.  This board has two features that we like:

* One you can select between 3.3V and 5V.  We always use 3.3V.
  Ground is pin 1 and plugs into pin 1 of N4 on the bus_loki.

* It puts DTR on pin 6 instead of RTS.  The DTR signal is
  what the Arduinos use for their reset signal.

The DFRobot board requires a USB-A to mini-USB-B cable.  We purchased
ours from Jameco (part number: 2152243.)

To upload the firmware, do the following:

1. Plug the serial download cable into your laptop/desktop.

2. Plug the serial download cable to into N4 on the bus_loki.
   Make sure the GND is connected to pin 1 (which is labeled
   on the Loki board.)

3. Type the following:

        make upload

That should do it.



