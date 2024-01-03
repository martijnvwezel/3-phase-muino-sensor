# Address

## Light sensor

* high (VDD) = 0x48
* low (GND) = 0x10

## IO expander PI4IOE5V6408
* low (GND) = 0x43

The PI4IOE5V6408 can be reset by the processor using an I2
C write command to change bit 0 of register 01h to a 1.
Immediately following this change, the PI4IOE5V6408 resets and all register values return to their default values. In this
case, the software reset bit returns to 0 as soon as the reset sequence is completed. 