# Wishbone Accessable I2C Core(s)

This repository contains contain two wishbone accessable I2C cores, one which
creates a [master](rtl/wbi2cmaster.v) and the other a
[slave](rtl/wbi2cslave.v).  Both work using a small piece of shared memory,
which may be read or written to via the wishbone bus.  Changes made to the
slave's memory can be read/written by the I2C master when it chooses to
make a transaction happen.  With respect to the I2C master controller,
changes made to the shared memory will require an explicit command in order to
push those changes from the master to the slave, or for example to read them
back from the slave.

## The Slave Core

The [slave core](rtl/wbi2cslave.v) acts as a simple two port memory interface
to 128 octets of memory.  The first port may be accessed via a 32-bit Wishbone
bus.  The second port may be accessed via an I2C bus, where this core sits as
a slave.  Both ports may read or written at the same time, and the core
properly sets the wishbone stall line on any collision, so as to arbitrate
who gets to write.

## The Master Core

The [wishbone master core](rtl/wbi2cmaster.v) has just been written.  It now
passes bench tests using Verilator, but has not yet been included in any
design(s), so it may (or may not) work on an actual FPGA.

# Status

Currently, the slave has not only implemented, but it has also been both
[bench tested](bench/cpp/wbi2cs_tb.cpp) as well proven by use within an HDMI
application.

The I2C master now passes [bench testing](bench/cpp/wbi2cm_tb.cpp) alone.
Testing within an HDMI application will be next.


# License

Gisselquist Technology is pleased to offer this core to all who are interested,
subject to the GPLv3 license.

# Commercial Opportunities

If the GPLv3 license is insufficient for your needs, other licenses may be
purchased from Gisselquist Technology, LLC.
