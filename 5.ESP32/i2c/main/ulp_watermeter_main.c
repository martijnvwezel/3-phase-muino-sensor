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
    // uint8_t data_rd = 0;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    printf("Cause to wakeup: %d\n", cause);
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

        init_ulp_program();
    }

    /* ULP RISC-V read and detected a temperature or pressure above the limit */
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        /* Pause ULP while we are using the RTC I2C from the main CPU */
        ulp_timer_stop();
        ulp_riscv_halt();
        printf("Muino liters: %ld\n", ulp_temp_reg_val);

        

        /* Resume ULP and go to deep sleep again */
        ulp_timer_resume();

        /* Add a delay for everything to the printed before heading in to deep sleep */
        
        // printf("ULP RISC-V woke up the main CPU\n");
        // printf("liter:1:is being found!\n");
        // vTaskDelay(10000);

    }

    /* Add a delay for everything to the printed before heading in to deep sleep */
    vTaskDelay(100);


    
    // printf("Muino liters: %ld\n", ulp_state.muino_ml_meters);
    printf("Muino liters: %ld\n", ulp_temp_reg_val);
    
    printf("Cause to wakeup: %d\n", cause);
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
    // printf("Initializing RTC I2C ...\n");
    // float scl_low_period;    /*!< SCL low period in micro seconds */
    // float scl_high_period;   /*!< SCL high period in micro seconds */
    // float sda_duty_period;   /*!< Period between the SDA switch and the falling edge of SCL in micro seconds */
    // float scl_start_period;  /*!< Waiting time after the START condition in micro seconds */
    // float scl_stop_period;   /*!< Waiting time before the END condition in micro seconds */
    // float i2c_trans_timeout; /*!< I2C transaction timeout in micro seconds */
    // /* Nominal I2C bus timing parameters for I2C standard mode. Max SCL freq of 100 KHz. */
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
        .i2c_timing_cfg.i2c_trans_timeout = 20,
    };

    // /* Nominal I2C bus timing parameters for I2C fast mode. Max SCL freq on S2 is about 233 KHz due to timing constraints. */
    // ulp_riscv_i2c_cfg_t i2c_cfg = {
    //     .i2c_pin_cfg.sda_io_num = GPIO_NUM_3,
    //     .i2c_pin_cfg.scl_io_num = GPIO_NUM_2,
    //     .i2c_pin_cfg.sda_pullup_en = false,
    //     .i2c_pin_cfg.scl_pullup_en = false,
    //     .i2c_timing_cfg.scl_low_period = 2,
    //     .i2c_timing_cfg.scl_high_period = 0.7,
    //     .i2c_timing_cfg.sda_duty_period = 1.7,
    //     .i2c_timing_cfg.scl_start_period = 2.4,
    //     .i2c_timing_cfg.scl_stop_period = 1.3,
    //     .i2c_timing_cfg.i2c_trans_timeout = 20,
    // };

    /* Nominal I2C bus timing parameters for I2C fast mode. Max SCL freq of 400 KHz. */
    // ulp_riscv_i2c_cfg_t i2c_cfg = {
    //     .i2c_pin_cfg.sda_io_num = GPIO_NUM_3,
    //     .i2c_pin_cfg.scl_io_num = GPIO_NUM_2,
    //     .i2c_pin_cfg.sda_pullup_en = false,
    //     .i2c_pin_cfg.scl_pullup_en = false,
    //     .i2c_timing_cfg.scl_low_period = 1.4,   
    //     .i2c_timing_cfg.scl_high_period = 0.3,  
    //     .i2c_timing_cfg.sda_duty_period = 1,    
    //     .i2c_timing_cfg.scl_start_period = 2,   
    //     .i2c_timing_cfg.scl_stop_period = 1.3,  
    //     .i2c_timing_cfg.i2c_trans_timeout = 20,
    // };


    i2c_cfg = (ulp_riscv_i2c_cfg_t)ULP_RISCV_I2C_DEFAULT_CONFIG();
    esp_err_t ret = ulp_riscv_i2c_master_init(&i2c_cfg);
    if (ret!= ESP_OK) {
        printf("ERROR: Failed to initialize RTC I2C. Aborting...\n");
        abort();
    }
}

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
