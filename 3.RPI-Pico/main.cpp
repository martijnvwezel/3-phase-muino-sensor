#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define I2C_SDA 31
#define I2C_SCL 32


// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}



void enable_boardled() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

void init_sensors(){
    i2c_init(i2c1, 100 * 1000); // max 400KHz for light sensor
    printf("bbbbbbbbbbbbb\n");
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    printf("aaaaaaaaaaaaaaa\n");
    // Enable pullups
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
    
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
}


int read_sensor(int sensor_id){
    return 1;
}


int main() {

    enable_boardled();
    stdio_init_all();
printf("adfafasdfasfddasf\n");
sleep_ms(3000); // Wait 100ms
    // init_sensors();
printf("adfafasdfasfddasf\n");

    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        
        printf("Sensor value %d, %d, %d\n", read_sensor(0),read_sensor(1),read_sensor(2));
        
        sleep_ms(100); // Wait 100ms
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100); // Wait 100ms
        init_sensors();
    }
    return 1;
}