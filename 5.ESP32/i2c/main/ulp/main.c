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

   This code runs on ULP RISC-V coprocessor
*/

#include <stdint.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_i2c_ulp_core.h"
#include "../sensor_defs.h"

/************************************************
 * Shared data between main CPU and ULP
 ************************************************/
int16_t ut_data = 0;
int32_t up_data = 0;


int main (void)
{

    // * Enable LED on the board
    set_pin_io(LED, true);
    ulp_riscv_delay_cycles(15 * ULP_RISCV_CYCLES_PER_MS);
    uint32_t sen_a = read_sensor(0, true);
    uint32_t sen_b = read_sensor(1, true);
    uint32_t sen_c = read_sensor(2, true);
    
    
    // // * Disable LED on the board
    set_pin_io(LED, false);
    ulp_riscv_delay_cycles(15 * ULP_RISCV_CYCLES_PER_MS);

    uint32_t sen_a_zero = read_sensor(0, false);
    uint32_t sen_b_zero = read_sensor(1, false);
    uint32_t sen_c_zero = read_sensor(2, false);

    // muino_obj.loop(sen_a, sen_b, sen_c, sen_a_zero, sen_b_zero, sen_c_zero);
    

    /* Wakeup the main CPU if either the uncompensated temperature or uncompensated pressure values
     * are more than their respective threshold values.
     */
    // if ((ut_data > ut_threshold) || (up_data > up_threshold)) {
        
    // }
    if (sen_a > 300){
        ulp_riscv_wakeup_main_processor();
    }

    return 0;
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