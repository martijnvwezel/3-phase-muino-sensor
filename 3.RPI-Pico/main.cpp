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
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
}

void init_IO_extender(){
    printf("set pin high.....");
    // Configure the PI4IOE5V6408 to input mode
    uint8_t config_data[] = {0x01, 0xff}; // Configure all pins as inputs, except pin led
    i2c_write_blocking(i2c0, IO_EXTENDER_I2C_ADDR, config_data, 2, false);
    printf("set pin high");
    // Set LED to HIGH
    uint8_t output_data[] = {0x02, 0xff};
    i2c_write_blocking(i2c0, IO_EXTENDER_I2C_ADDR, output_data, 2, false);
    printf("set pin high");
}



int read_sensor(int sensor_id){

    i2c_write_blocking(i2c1, VEML6030_I2C_ADDR, NULL, 0, true); // start sensor

    uint8_t data[2];
    i2c_read_blocking(i2c0, VEML6030_I2C_ADDR, data, 2, false); // Read 2 bytes of data

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