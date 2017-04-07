# Wishbone Accessable I2C Core(s)

This repository is intended to (eventually) contain two wishbone accessable
I2C cores, one for a master and slave. 

## The Slave Core

The [slave core](rtl/wbi2cslave.v) acts as a simple two port memory interface
to 127 octets of memory.  The first port may be accessed via a 32-bit Wishbone
bus.  The second port may be accessed via an I2C bus, where this core sits as
a slave.  Both ports may read or written at the same time, although if both
writes come to the core on the same clock the I2C will write.

# Status

Currently, only the slave is implemented.  Further, although implemented, it
so far only passes Verilator based [bench testing](bench/cpp/wbi2c_tb.cpp).  I
expect to place it onto an FPGA within a week or so (of 7 April, 2017).

# License

Gisselquist Technology is pleased to offer this core to all who are interested,
subject to the GPLv3 license.

# Commercial Opportunities

If the GPLv3 license is unsufficient for your needs, other licenses can be
purchased from Gisselquist Technology, LLC.
