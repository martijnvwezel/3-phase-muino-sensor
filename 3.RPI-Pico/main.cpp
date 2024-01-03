#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define I2C_SDA 26
#define I2C_SCL 27

#define VEML6030_I2C_ADDR_H 0x48
#define LED 1 << 6

#define PI4IO_CHIP_ID 0x1
#define PI4IO_IO_DIRECTION 0x3
#define PI4IO_OUTPUT 0x5
#define PI4IO_OUTPUT_HI_IMPEDANCE 0x7
#define PI4IO_INPUT_STATUS 0xF // This register is not writable
#define PI4IO_INTERRUPT_STATUS 0x13
#define PI4IO_CHIP_ID_VAL 0xA0
#define PI4IO_CHIP_ID_MASK 0xFC // This register is not writable
#define PI4IO_I2C_ADDR 0x43

void init_IO_extender();

void enable_boardled() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

void write_reg(uint8_t reg, uint8_t val) {
    uint8_t read;
    uint8_t data[2] = {reg, val};
    i2c_write_blocking(i2c1, PI4IO_I2C_ADDR, (uint8_t*)&data, 2, false); // keep control of bus
}

uint8_t read_reg(uint8_t reg) {
    uint8_t read;
    i2c_write_blocking(i2c1, PI4IO_I2C_ADDR, &reg, 1, true); // keep control of bus
    i2c_read_blocking(i2c1, PI4IO_I2C_ADDR, &read, 1, false);
    return read;
}

uint16_t read_reg_light(uint8_t reg) {
    uint16_t read;
    i2c_write_blocking(i2c1, VEML6030_I2C_ADDR_H, &reg, 1, true); // keep control of bus
    i2c_read_blocking(i2c1, VEML6030_I2C_ADDR_H, (uint8_t*)&read, 2, false);
    return read;
}
void write_reg_light(uint8_t reg, uint16_t val) {
    uint8_t read;
    uint8_t data[3] = {
        reg,
        (uint8_t)(val & 0xFF), // Low byte of val
        (uint8_t)(val >> 8)    // High byte of val
    };
    i2c_write_blocking(i2c1, VEML6030_I2C_ADDR_H, (uint8_t*)&data, 3, false); // keep control of bus
}

void init_i2c() {
    i2c_init(i2c1, 300 * 1000); // max 400KHz for light sensor
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
}

void set_pin_io(uint8_t pin_number, bool value) {
    // Set the output state for the specified pin
    uint8_t output_data[] = {PI4IO_OUTPUT, (uint8_t)(1 << pin_number)};
    if (!value)
        output_data[1] = ~output_data[1];

    i2c_write_blocking(i2c1, PI4IO_I2C_ADDR, output_data, 2, false);
}

void init_IO_extender() {
    // Check if io externder is alive
    uint8_t rxdata;

    // 0x01 test register
    uint8_t reg = 0x01;
    i2c_write_blocking(i2c1, PI4IO_I2C_ADDR, &reg, 1, true);
    int ret = i2c_read_blocking(i2c1, PI4IO_I2C_ADDR, &rxdata, 1, false);
    printf("Device ID: %d\n", ret, rxdata);

    write_reg(PI4IO_IO_DIRECTION, (1 << 3) | (1 << 4) | (1 << 5) | LED);
    write_reg(PI4IO_OUTPUT, LED); // (1<<3)|(1<<4)|(1<<5)
    write_reg(PI4IO_OUTPUT_HI_IMPEDANCE, ~((1 << 3) | (1 << 4) | (1 << 5) | LED));

    printf("IODIR: %i.\n", read_reg(0x03));
    printf("Output: %i.\n", read_reg(PI4IO_OUTPUT));
    printf("HighZ: %i.\n", read_reg(PI4IO_OUTPUT));

}

int read_sensor(int sensor_id) {
    if (sensor_id == 0) {
        write_reg(PI4IO_OUTPUT, (1 << 3) | LED);
    } else if (sensor_id == 1) {
        write_reg(PI4IO_OUTPUT, (1 << 4) | LED);
    } else if (sensor_id == 2) {
        write_reg(PI4IO_OUTPUT, (1 << 5) | LED);
    } else {
        return -1;
    }

    // Enable ALS_SD sensor on
    write_reg_light(0x00, 1 << 11);

    // ALS response
    return read_reg_light(0x04);
}

int main() {
    // * Enable the board LED of the PICO
    enable_boardled();

    // * Enable the USB, UART stack
    stdio_init_all();

    // * init I2C of the PICO
    init_i2c();

    // * Start communicating with the pico
    init_IO_extender();

    // * Enable LED on the board
    set_pin_io(LED, true);

    while (true) {
        // * Continuesly print values of the 3 light sensors
        printf("%d\t%d\t%d\n", read_sensor(0), read_sensor(1), read_sensor(2));
    }
    return 1;
}