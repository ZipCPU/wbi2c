# I2C Components

- The I2C CPU: This comes in one of two versions, either the AXI-lite version
  called [AXILI2CCPU](axili2ccpu.v), or the Wishbone version called
  [WBI2CCPU](wbi2ccpu.v).  Both depend on other components not kept here, but
  which can be found on Github.

  - [AXISI2C](axisi2c.v) implements the actual I2C instructions for the I2C
    CPU.  These are instructions 0-7.  Instructions 8-15 are implemented in the
    CPU itself.

- [WBI2CMASTER](wbi2cmaster.v)

  - [LLI2CM](lli2cm.v): This is used by the [WBI2CMASTER](wbi2cmaster.v) to help
    encapsulate some of the I2C processing.

- [WBI2CSLAVE](wbi2cslave.v): A basic WB I2C slave.  Implements
  a shared 128-byte memory which can be read/written via either WB or I2C.

