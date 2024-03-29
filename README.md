# I2C Core(s)

This repository contains contains a couple of I2C cores.  There's a basic,
[Wishbone accessible I2C master](rtl/wbi2cmaster.v), as well as a basic
[Wishbone accessible I2C slave](rtl/wbi2cslave.v) controller.
Both work using a small piece of shared memory,
which may be read or written to via the wishbone bus.  Changes made to the
slave's memory can be read/written by the [I2C master](rtl/wbi2cmaster.v)
when it chooses to make a transaction happen.  With respect to the [I2C master
controller](rtl/wbi2cmaster.v), changes made to the shared memory will
require an explicit command in order to push those changes from the master to
the slave, or for example to read them back from the slave.

Since writing those original basic controllers, I've now added an
[I2C CPU](https://zipcpu.com/blog/2021/11/15/ultimate-i2c.html).  This is
really more of a scripted FSM than a full purpose CPU, however it does fit
in nicely with many of the telemetry projects I'm dealing with.  There are
now two versions of this
[I2C CPU](https://zipcpu.com/blog/2021/11/15/ultimate-i2c.html) contained
within this repository: a [Wishbone version](rtl/wbi2ccpu.v)
and an [AXI-Lite version](rtl/axili2ccpu.v).  Other than the bus
implementation (and possibly any bus induced endianness issues), the two should
be compatible at the software level.

## The Slave Core

The basic I2C [slave core](rtl/wbi2cslave.v) acts as a simple two port memory
interface to 128 octets of memory.  The first port may be accessed via a
32-bit Wishbone bus.  The second port may be accessed via an I2C bus, where
this core sits as a slave.  Both ports may read or written at the same time,
and the core properly sets the wishbone stall line on any collision, so as
to arbitrate who gets to write.

## The Master Core

The [wishbone master core](rtl/wbi2cmaster.v) has been used successfully
to query monitors for their EDID information.   This also acts as a memory.
Given a command, the memory will be written to or read from the slave.

Sadly, this core is tied to an I2C device requiring a specific command
format: I2C ADDR, SLV ADDR, SEND data, or I2C ADDR, READ data.  Devices that
have either no address, or a two byte address, will not work with this
particular master--although they can still be queried by the CPU following.

## The I2C CPU

This module works by following an externally provided script describing its
interaction.  This script can either be provided via a bus slave port, or it
can be read from external memory, using either Wishbone or AXI-lite bus
protocols.  The script contains commands that will then be fed to the I2C
controller within.
Specific commands include sending a START condition, STOP condition, particular
bytes or data, or reading bytes of data.  Other commands, such as repeating
a sequence or halting, are also available to control the instruction handler.
The full command list can be found [here](doc/ultramicro-isa.png), and [its
(most recent) documentation may be found
here](https://zipcpu.com/blog/2021/11/15/ultimate-i2c.html).

A [small assembler](sw/i2casm.l) exists in the [SW/ directory](sw/) which can
either assemble a given script or disassemble such a script.  Run "make test"
in the [sw/ directory](sw/) to build this assembler, assemble [a test
"program"](sw/testfil.s), and then to disassemble it back to its component
instructions for verification purposes.

The output of this CPU is an AXI stream containing all of the bytes that have
been read from the interface while following the script.

# Status

Both the [slave](rtl/wbi2cslave.v) and [master](rtl/wbi2cmaster.v) controllers
have been tested as part of the EDID support to (now) multiple video
applications.

The [I2C CPU](https://zipcpu.com/blog/2021/11/15/ultimate-i2c.html) has also
proven itself valuable for both [temperature reading](https://github.com/ZipCPU/eth10g/blob/master/sw/i2c/temp.txt)
[(and monitoring)](https://github.com/ZipCPU/eth10g/blob/master/rtl/wbfan.v),
[EDID handling](https://github.com/ZipCPU/eth10g/blob/master/sw/i2c/edid.txt),
[DDR3 configuration
reading](https://github.com/ZipCPU/eth10g/blob/master/sw/i2c/ddr3.txt),
[SFP+ configuration
reading](https://github.com/ZipCPU/eth10g/blob/master/sw/i2c/sfp.txt),
[Si5324 controlling](https://github.com/ZipCPU/eth10g/blob/master/sw/i2c/siclk.txt),
and [I2C OLED control](https://www.amazon.com/Teyleten-Robot-Display-SSD1306-Raspberry/dp/B08ZY4YBHL).
[Example software may be found here](https://github.com/ZipCPU/eth10g/blob/master/sw/zipcpu/board).
The demonstration control scripts, linked above, also included commands for a
TCA9548 I2C hub, used for deconflicting the I2C addresses of multiple
(otherwise identical) I2C devices.

An [external Wishbone
DMA](https://github.com/ZipCPU/eth10g/blob/master/rtl/wbi2c/wbi2cdma.v) also
exists, to put data read from this controller into memory.  That DMA has yet to
be integrated into this repository.  An AXI-Lite version of this DMA is also
planned, but not yet built.

The biggest item missing from this repository at present is a good
specification for these IP components.

Bottom line: this IP has been such a success, that it is likely to be used in
multiple projects going forward.

# License

Gisselquist Technology is pleased to offer this core to all who are interested,
subject to the GPLv3 license.

# Commercial Opportunities

If the GPLv3 license is insufficient for your needs, other licenses may be
purchased from Gisselquist Technology, LLC.
