# I2C CPU Assembler

The main component of this software directory is an I2C CPU assembler.  This
assembler takes files, such as [test testfil.s](testfil.s), containing
assembler commands, and it generates a binary file which can then be fed to
command the [I2CCPU](../rtl/wbi2ccpu.v).

Assembler commands are:

- NOOP: Do nothing, but consume a clock cycle.

- START: Issue an I2C Start condition

- STOP: Issue an I2C Stop condition

- SEND <byte>: Sends the given byte.  There are two additional yet special
  forms of this command recognized by the assembler.  `SEND <byte>,WR` will
  send the byte shifted left by one.  `SEND <byte>,RD` will send the byte
  upshifted by one, and or'd with one.  These latter two forms are useful for
  starting a communication, where the first byte needs to identify the ID of
  the desired device (the `<byte>`), and RD or WR are the last bit in the
  sequence used to tell the device if you'll be reading from or writing to it.

- RXK: Receive a byte and forward it to the outgoing AXI stream once complete.
  ACK the result once it has been received.

- RXN: The same as RXK above, but without the ACK at the end

- RXLK: Same as RXK, but raise the AXI stream TLAST signal when this byte has
  been received.

- RXLN: Same as RXN or RXLK--raises TLAST when the byte is available, and does
  not ACK the result

Those are the commands actually sent to the underlying I2C controller.  Another
5 commands exist as well, which will be handled by the instruction decoder:

- WAIT: Will pause all instructions to the I2C controller until an external
  signal has been received.

- HALT: Once received, no further commands will be issued to the I2C controller.

- ABORT: Indicates an address to return to should any command receive a NAK
  of any type--indicating a potential arbitration lost condition.

- TARGET: Sets the address for a future JUMP instruction to return to.

  The I2C controller does not support conditional jumps or halts.  Therefore,
  it can only support one of two control structures: Run from a start to a
  completion, or run from a start to a `TARGET` command followed by an infinite
  loop from the last `TARGET` command to the final `JUMP` command.

- JUMP: This is the other half of the `TARGET` loop structure.  Once `JUMP`
  is received, the CPU will `JUMP` to the `TARGET` instruction.

- CHANNEL: In shared I2C sytems, the channel command sets the `TID` field
  of the outgoing AXI stream, so the stream values can be sent to different
  targets if necessary.

Note that all logic is address independent: any jump addresses, whether via
`ABORT` or `JUMP`, are defined by the locations of these instructions.

The assembler has some support for named immediate values.  A statement of
the form "A=0xff", for example, will define a symbol "A" as having the value
"0x0ff".  This value can later be used in either SEND or CHANNEL commands
if desired.

## Testing

A test of the I2C assembler is provided.  The usage of this assembler can be
found via the `-h` option:

> i2casm -h

To test, use the assembler to build a binary:

> i2casm testfil.s -o dump.bin

You can then disassemble this file to see how well the assembler worked.

> i2casm -d dump.bin


