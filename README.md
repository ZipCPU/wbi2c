# Wishbone Accessable I2C Core(s)

This repository is intended to (eventually) contain two wishbone accessable
I2C cores, one for a master and slave. 

## The Slave Core

The [slave core](rtl/wbi2cslave.v) acts as a simple two port memory interface
to 127 octets of memory.  The first port may be accessed via a 32-bit Wishbone
bus.  The second port may be accessed via an I2C bus, where this core sits as
a slave.  Both ports may read or written at the same time, and the core
properly sets the wishbone stall line on any collision, so as to arbitrate
who gets to write.

## The Master Core

The wishbone master core is ... not yet written.  It is a work in progress.
My thought so far has been to implement a shared memory peripheral, like the
slave, but to also create a command word.  Writes to that command word would
initiate reads (or writes) from (or to) the bus, whereas the values read would
be placed into the shared memoory, and the shared memory would also be used as
the source for any writes.  A second configuration word would control the speed
of the bus.

# Status

Currently, only the slave is implemented.  Not only has it been implemented, but
it has also been verified to work both in
[bench testing](bench/cpp/wbi2c_tb.cpp) as well as within an initial HDMI
application.

Work on the I2C master will be next.


# License

Gisselquist Technology is pleased to offer this core to all who are interested,
subject to the GPLv3 license.

# Commercial Opportunities

If the GPLv3 license is insufficient for your needs, other licenses may be
purchased from Gisselquist Technology, LLC.
