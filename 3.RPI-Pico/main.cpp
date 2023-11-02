#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define I2C_SDA 31
#define I2C_SCL 32

#define VEML6030_I2C_ADDR 0x10
#define IO_EXTENDER_I2C_ADDR 0x43
#define LED 6


void enable_boardled() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

void init_i2c(){
    i2c_init(i2c1, 100 * 1000); // max 400KHz for light sensor
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    i2c_set_slave_mode(i2c1, false, 0x00);
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
}
void set_pin_high(uint8_t pin_number) {
    // Set the output state for the specified pin
    uint8_t output_data[] = {0x05, (uint8_t)(1 << pin_number)};
    i2c_write_blocking(i2c1, IO_EXTENDER_I2C_ADDR, output_data, 2, false);
}
void init_IO_extender(){
    uint8_t pin_number = 6; 
    printf("Configuring I/O Direction...\n");
    uint8_t output_data[] = {0x03, (uint8_t)(1 << pin_number)};
    i2c_write_blocking(i2c1, IO_EXTENDER_I2C_ADDR, output_data, 2, false);
    printf("I/O Direction configured.\n");
    set_pin_high(pin_number);
}



int read_sensor(int sensor_id){

    i2c_write_blocking(i2c1, VEML6030_I2C_ADDR, NULL, 0, true); // start sensor

    uint8_t data[2];
    i2c_read_blocking(i2c1, VEML6030_I2C_ADDR, data, 2, false); // Read 2 bytes of data

    // Calculate the light intensity from the received data
    uint16_t light_data = (data[1] << 8) | data[0];



    return light_data;
}


int main() {

    enable_boardled();
    stdio_init_all();
    sleep_ms(2000);
    printf("init i2c..\n");
    init_i2c();
    printf("init io extender..\n");
    init_IO_extender();
    printf("loop..\n");
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        // float light_lux = light_data / 100.0;
        // printf("Light Intensity: %.2f Klux\n", light_data);
        uint8_t data[2];
        printf("Sensor value %d, %d, %d\n", read_sensor(0),read_sensor(1),read_sensor(2));
        // uint addr = 0x07;
        // i2c_read_blocking(i2c1, addr, &data, 2, false);

        sleep_ms(100); // Wait 100ms
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100); // Wait 100ms
        // init_sensors();
    }
    return 1;
}