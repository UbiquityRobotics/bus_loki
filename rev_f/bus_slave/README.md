# bus_slave

This package contains some common code for bus slave modules.
In particular, it contains:

* `Bus_Slave.h`: The header file for the `UART`, `AVR_UART`, `NULL_UART`,
  `Bus_Buffer`, `Bus_Slave`, and `Bus_Module` C++ classes.

* `Bus_Slave.cpp`: The shared code for the `UART`, `AVR_UART`, `NULL_UART`,
  `Bus_Buffer`, `Bus_Slave`, and `Bus_Module` C++ classes.

* `bus_code_generator.py': A Python program than takes an .xml file
  that specifieds the various registers and methods supported by a
  bus module and generates support C++ `.h` and `.cpp` files.

## Using the Code

The intention is that this code will coexist in a ROS Catkin
workspace along side each module.  The following gives an
indication of how 

        .../catkin_ws/
          src/
            bus_module1/
            ...
            bus_moduleN/
            bus_slave/
            Arduino/
            Arduino-Make/

The following commands will set things up for you:

        cd .../catkin_ws/src
	git clone https://github.com/UbiquityRobotics/bus_slave.git
	git clone https://github.com/UbiquityRobotics/Arduino.git
	git clone https://github.com/UbiquityRobotics/Arduino-Makefile.git

You need the following packages also need to be installed:

        sudo apt-get install python-serial
        sudo apt-get install avrdude binutils-avr gcc-avr avr-libc gdb-avr

If you want to build the `bus_bridge_encoders_sonar` firmware:

        git clone https://github.com/UbiquityRobotics/bus_bridge_encoders_sonar.git
        cd bus_bridge_encoders_sonar
        make

## `bus_code_generator.py` Code Generator:

The `bus_code_generator.py` code generator takes a `.xml` file
and generates an associated `.h` and `.cpp`.  For example,

        bus_code_generator.py Bus_Module_Name.xml

will generate the files `Bus_Module_Name_Slave.h` and
`Bus_Module_Name_Slave.cpp`.

(Eventually this program will also generate C++ and Python code
for accessing the modules from a ROS node.)

### `.xml` File Format

The basic format of an `.xml` file is:

        <Module Name="..." ...>
          <Overview> ... </Overview>
          <Register Name="..." Type="..." Number="..." Brief="...">
            <Description> ... </Description>
          ... <!-- more register declarations-->
          <Function Name="..." Number="..." Brief="...">
            <Description> ... </Descrption>
            <Argument/ Name="..." Type="...">
            <Return/ Type="...">
          ... <!-- more function declarations-->
        </Module>

A module is treated as a bunch of registers and/or functions.
Each register has a name, type (e.g. `Logical, `Byte`, `UInteger`, etc.)
and number.  Each function has a name, some arguments,
some return values, and a number.

### `<Module>` Tag

The `<Moudle>` Tag is the top level tag and has the following attributes:

* `Name="..."`: The name of the bus module (e.g. Bus_Sonar10).

* `Vendor="..."`: The vendor name who makes the module.

* `Generate="..."`: (Depricated)

* `Address_RE="..."`: (Depricated)

* `Address_Type="..."`: For now this attribute must be `MakerBus`.
  (Fix this!!!)

* `Brief="..."`: A brief desciption of the module functionality.

There is no text associated with this tag.

### `<Overview>` Tag

This tag nests under the `<Module>` tag and is text only.  The text
is one or more paragraphs of text that describe the module.

### `<Classificatin>` Tag

*{ This tag needs to be rethought. }*

### `<Register>` Tag

This tag nests under the `<Module>` tag and describes a module resister.
This tag has the following attributes:

* `Name="..."`: The register name.

* `Type="..."`: The type must be one of the following:

  * `Byte`: Signed 8-bit byte.

  * `Integer`: Signed 32-bit value.

  * `Long`: Signed 64-bit value.

  * `Short`: Signed 16-bit value.

  * `Logical`: 8-bits where 0 is false and non-zero is true.

  * `Byte`: Unsigned 8-bit byte.

  * `Integer`: Unsigned 32-bit value.

  * `Long`: Unsigned 64-bit value.

  * `Short`: Unigned 16-bit value.

* `Number="..."`: This is the register number.  If N is the register
  number, N+0 is the read command and N+1 is the write command.  Thus,
  registers always take up two command numbers.

* `Brief="...": This is a brief textural description of the
  what the register does.

### `<Description>' Tag

This tag is a textual tag that contains one or more paragraphs that
describe the register

### `<Function>` Tag

*{ More goes here }*




