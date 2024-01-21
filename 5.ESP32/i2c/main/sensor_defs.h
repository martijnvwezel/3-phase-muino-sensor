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
#define LED 1 << 6
#define SENS0 1 << 3
#define SENS1 1 << 4
#define SENS2 1 << 5

/************************************************
 * Devices
 ************************************************/

static uint16_t read_sensor(uint8_t sensor_id, bool led_on);
static void set_pin_io(uint8_t pin_number, bool value);