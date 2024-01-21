/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* ULP RISC-V RTC I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "esp_sleep.h"
#include "ulp_riscv.h"
#include "ulp_riscv_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ulp_main.h"
#include "bmp180_defs.h"
#include "sensor_defs.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

/************************************************
 * ULP utility APIs
 ************************************************/
static void init_ulp_program(void);

/************************************************
 * RTC I2C utility APIs
 ************************************************/
static void init_i2c(void);



void app_main(void)
{
    uint8_t data_rd = 0;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    /* Not a wakeup from ULP
     * Initialize RTC I2C
     * Setup BMP180 sensor
     * Store current temperature and pressure values
     * Load the ULP firmware
     * Go to deep sleep
     */
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not a ULP-RISC V wakeup (cause = %d)\n", cause);

        /* Initialize RTC I2C */
        init_i2c();

        /* Configure I2C slave address */
        ulp_riscv_i2c_master_set_slave_addr(PI4IO_I2C_ADDR);

        // /* Reset the BMP180 sensor*/
        // ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_CHIP_ID);
        // uint8_t data_wr = 0x1;
        // ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

        /* Confirm that the sensor is alive
         * The BMP180 returns the chip id 0x55 on quering reg addr 0xD0
         */
        printf("Read chip ID..\n");
        ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_CHIP_ID);
        data_rd = 0x1; // read register value of chip id
        ulp_riscv_i2c_master_read_from_device(&data_rd, 1);
        if (data_rd != PI4IO_CHIP_ID) {
            printf("Read ID %x\n", data_rd);
            printf("ERROR: Cannot communicate with I2C sensor\n");

            abort();
        } else{
            printf("Read ID %x\n", data_rd);
            // abort();
        }

        /* Setup pin direction io extender*/
        printf("Setup pin direction..\n");
        uint8_t data_wr = SENS0 | SENS1 | SENS2 | LED;
        ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_IO_DIRECTION);
        ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

        /* Setup pin output */
        printf("Setup pin output..\n");
        data_wr = LED;
        ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_OUTPUT);
        ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

        /* Setup pin impedance */
        printf("Setup pin impedance..\n");
        data_wr =  ~(SENS0 | SENS1 | SENS2 | LED);
        ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_OUTPUT_HI_IMPEDANCE);
        ulp_riscv_i2c_master_write_to_device(&data_wr, 1);
        
        while (false) {
                // * Continuesly print values of the 3 light sensors
                // printf("Led uit\n");
                // printf("%d\t%d\t%d\n", read_sensor(0, false), read_sensor(1, false), read_sensor(2, false));
                // vTaskDelay(100);
                // printf("Led aan\n");
                printf("%x\t%x\t%x\n", read_sensor(0, true), read_sensor(1, true), read_sensor(2, true));
                // vTaskDelay(100);
            }
        /* Config ALS_SD*/
        /* Load ULP firmware
         *
         * The ULP is responsible of monitoring the temperature and pressure values
         * periodically. It will wakeup the main CPU if the temperature and pressure
         * values are above a certain threshold.
         */
        init_ulp_program();
    }

    /* ULP RISC-V read and detected a temperature or pressure above the limit */
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        printf("ULP RISC-V woke up the main CPU\n");

        /* Pause ULP while we are using the RTC I2C from the main CPU */
        ulp_timer_stop();
        ulp_riscv_halt();

        /* Resume ULP and go to deep sleep again */
        ulp_timer_resume();
    }


    /* Add a delay for everything to the printed before heading in to deep sleep */
    vTaskDelay(100);

    /* Go back to sleep, only the ULP RISC-V will run */
    printf("Entering deep sleep\n\n");

    /* RTC peripheral power domain needs to be kept on to keep RTC I2C related configs during sleep */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    esp_deep_sleep_start();
}

static void init_i2c(void)
{
    /* Configure RTC I2C */
    printf("Initializing RTC I2C ...\n");
    ulp_riscv_i2c_cfg_t i2c_cfg = {
        .i2c_pin_cfg.sda_io_num = GPIO_NUM_3,
        .i2c_pin_cfg.scl_io_num = GPIO_NUM_2,
        .i2c_pin_cfg.sda_pullup_en = false,
        .i2c_pin_cfg.scl_pullup_en = false,
        .i2c_timing_cfg.scl_low_period = 5,
        .i2c_timing_cfg.scl_high_period = 5,
        .i2c_timing_cfg.sda_duty_period = 2,
        .i2c_timing_cfg.scl_start_period = 3,
        .i2c_timing_cfg.scl_stop_period = 6,
        .i2c_timing_cfg.i2c_trans_timeout = 20,// was 20 //* ms

    }; //  Max SCL freq of 100 KHz.

    esp_err_t ret = ulp_riscv_i2c_master_init(&i2c_cfg);
    if (ret!= ESP_OK) {
        printf("ERROR: Failed to initialize RTC I2C. Aborting...\n");
        abort();
    }
}

static void set_pin_io(uint8_t pin_number, bool value){

    uint8_t data_wr = pin_number;

    if(!value) {
        data_wr = ~data_wr;
    }

    ulp_riscv_i2c_master_set_slave_addr(PI4IO_I2C_ADDR);
    ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_OUTPUT);
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);
}

static uint16_t read_sensor(uint8_t sensor_id, bool led_on) {
    uint8_t reg = 0x00;
    if (sensor_id == 0) {
        reg = SENS0;
    } else if (sensor_id == 1) {
        reg = SENS1;
    } else if (sensor_id == 2) {
        reg = SENS2;
    } else {
        return -1;
    }
    
    if(!led_on)
        reg &= ~LED;
    else
        reg |= LED;

    set_pin_io(reg, true); // Always true !

    // * Make sure LIGHT sensor add is called
    ulp_riscv_i2c_master_set_slave_addr(VEML6030_I2C_ADDR_H);

    // Gain x2 and 800ms 
    // printf("set gain and ms..\n");
    uint8_t data_wr = (uint8_t)((1 << 11) | ((1<<6)|(1<<7)));
    ulp_riscv_i2c_master_set_slave_reg_addr(VEML6030_ALS_SD);
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);
    
    // * Read value
    printf("read data..\n");
    uint8_t data_rd[2] = {0x00, 0x00};

    ulp_riscv_i2c_master_set_slave_reg_addr(VEML6030_ALS_READ_REG);
    ulp_riscv_i2c_master_read_from_device(data_rd, 2);
    // vTaskDelay(100);

    return ((uint16_t)data_rd[0] << 8) | data_rd[1];
    // return (uint16_t) data_rd[0];
}



// static void veml6030_read16(uint16_t *data_out, uint32_t reg_msb, uint32_t reg_lsb)
// {
//     uint8_t data_rd = 0;
//     *data_out = 0;

//     ulp_riscv_i2c_master_set_slave_reg_addr(reg_msb);
//     ulp_riscv_i2c_master_read_from_device(&data_rd, 1);
//     *data_out |= (uint16_t)(data_rd << 8);
//     ulp_riscv_i2c_master_set_slave_reg_addr(reg_lsb);
//     data_rd = 0;
//     ulp_riscv_i2c_master_read_from_device(&data_rd, 1);
//     *data_out |= (uint16_t)(data_rd);
// }

static void init_ulp_program(void)
{
    esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

    /* The first argument is the period index, which is not used by the ULP-RISC-V timer
     * The second argument is the period in microseconds, which gives a wakeup time period of: 40ms
     */
    ulp_set_wakeup_period(0, 40000);

    /* Start the program */
    err = ulp_riscv_run();
    ESP_ERROR_CHECK(err);
}
