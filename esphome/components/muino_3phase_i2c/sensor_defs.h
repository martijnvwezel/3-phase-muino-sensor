#pragma once

// * Light-sensor defs
#define VEML6030_I2C_ADDR_H 0x48
#define VEML6030_ALS_SD 0x00
#define VEML6030_ALS_READ_REG 0x04

// * IO-extender defs
#define PI4IO_I2C_ADDR 0x43
#define PI4IO_CHIP_ID 0x1
#define PI4IO_SETTINGS_REG 0x0
#define PI4IO_IO_DIRECTION 0x3
#define PI4IO_OUTPUT 0x5
#define PI4IO_OUTPUT_HI_IMPEDANCE 0x7
#define PI4IO_INPUT_STATUS 0xF // This register is not writable
#define PI4IO_INTERRUPT_STATUS 0x13
#define PI4IO_CHIP_ID_VAL 0xA0
#define PI4IO_CHIP_ID_MASK 0xFC // This register is not writable


// * Defines for IO-extender
#define SENS0 0b00001000
#define SENS1 0b00010000
#define SENS2 0b00100000
#define LED   0b01000000


typedef struct {
    int8_t   phase;
    int32_t  liters;

    int a_min;
    int b_min;
    int c_min;

    int a_max;
    int b_max;
    int c_max;

    uint16_t a;
    uint16_t b;
    uint16_t c;

} state_t;