# bus_loki

This repository contains the documentation and code for the
Loki robot.

<BlockQuote>
<Img Src="Loki_Side_with_Arm_Out.jpg" Alt="Side View of Loki">
</BlockQuote>

## Introduction

Loki is a small robot that is intended to be a used to learn
how to use ROS (Robot Operating System.)  ROS runs on a
Raspberry Pi 2 computer board and it plugs into the Loki
printed circuit board to access 2 motors with drive electronics
and wheel encoders, 16 sonars, a 5 mega pixel camera,
a 4 degree of freedom Arm, and a 6000mAH battery.
Using a USB Wireless dongle, software is developed on a
laptop/desktop system and downloaded into the Raspberry Pi 2
processor board.

## Specifications:

* Runs ROS (Robot Operating System) on a Raspberry Pi 2 computer.

* Dimensions: 250mm (L) x 110mm (W) x ~200mm (H)

* Battery: 6000mAH Lithium-Ion Battery Pack

* Sonars: 16 HC-SR04 sonars

* Motors: 2 Micro Metal Gearmotors 298:1 Gear Redution

* Wheels: 70mm (D) x 8mm (W)

* Encoders: 16.26 ticks/mm

* Speed: 1 M/Sec

* Embedded Processor: 20MHz @ 5V Atmel ATmega2560 (Arduino Mega compatible)

* Arm: 4 degree of freedom Me Arm 0.4 

* LED's: 8 

* Expansion: 2x5 .1in Expansion Bus Connector

* Camera: Upward pointing Raspberry Pi Camera (2592 x 1944 pixels)

## Running Loki

(These instructions are too brief!!!)

Do the following:

1. Run `roscore` on RasPi2

2. Run `roslaunch ros_arduino_python arduino.launch` on RasPi2

3. Run `roslaunch fiducial_lib loki.launch` on laptop

4. Run `rviz` on laptop.

   * In left sidebar, under `Global Option` change `Fixed Frame
     to `base_link`.

   * Click on [Add] and select `Range`.  You will need to do this
     16 times, because there are 16 sonars. 

   * Under `Range`=>`Topic`, select the correct topic to listen to.
     This will be one of `/arduino/sensor/sensor_1` through 
     `/arduino/sensor/sensor_16`.

   * Rinse, Lather, Repeat.

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



