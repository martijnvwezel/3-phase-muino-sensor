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
#include <ulp_riscv_utils.h>
#include "ulp_riscv_i2c_ulp_core.h"
#include "ulp_riscv_i2c.h"
#include "esp_attr.h"


#include "../sensor_defs.h"


typedef struct {
    int8_t   phase;
    int8_t   fine;
    int32_t  liters;
    uint32_t muino_ml_meters;

    int a_min;
    int b_min;
    int c_min;

    int a_max;
    int b_max;
    int c_max;
} state_t;


/************************************************
 * Shared data between main CPU and ULP
 ************************************************/
state_t state;
int32_t temp_reg_val = INT32_MIN;


// Function declarations
int  mini_average(int x, int y, int alpha_cor);
int  max_average(int x, int y, int alpha_cor);
void magnitude_offset_iter(state_t* state, int a, int b, int c);
void phase_coarse_iter(state_t* state, int a, int b, int c);

// Fuction declaration for io communication
static void            set_pin_io(uint8_t pin_number, bool value);
static inline uint16_t read_sensor(uint8_t sensor_id, bool led_on);
bool                   magic_code_box(state_t* state, int sen_a, int sen_b, int sen_c);



int main(void) {
    
    static uint32_t old_liters_update = 0;
    uint32_t bliep = 0;



    /* Setup pin direction io extender*/
    uint8_t data_wr = SENS0 | SENS1 | SENS2 | LED;
    ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_IO_DIRECTION);
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

    /* Setup pin output */
    data_wr = LED;
    ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_OUTPUT);
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

    /* Setup pin impedance */
    data_wr = ~(SENS0 | SENS1 | SENS2 | LED);
    ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_OUTPUT_HI_IMPEDANCE);
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

    while (true) {
        
        // * Enable LED on the board
        set_pin_io(LED, true);
        ulp_riscv_delay_cycles(150 * ULP_RISCV_CYCLES_PER_MS);
        uint16_t sen_a_light = read_sensor(0, true);
        uint16_t sen_b_light = read_sensor(0, true);
        uint16_t sen_c_light = read_sensor(0, true);

  
        if ( sen_a_light > 100 || bliep++ > 60) {
            old_liters_update = state.liters;
            ulp_riscv_wakeup_main_processor();
            temp_reg_val = sen_a_light;
        }

        // * Disable LED on the board
        set_pin_io(LED, false);
        ulp_riscv_delay_cycles(850 * ULP_RISCV_CYCLES_PER_MS);
        // ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_MS);

        uint16_t sen_a = sen_a_light - read_sensor(0, false);
        uint16_t sen_b = sen_b_light - read_sensor(0, false);
        uint16_t sen_c = sen_c_light - read_sensor(0, false);

        bool send = magic_code_box(&state, sen_a, sen_b, sen_c);

        /* Wakeup the main CPU if either the uncompensated temperature or uncompensated pressure values
         * are more than their respective threshold values.
         */
        // state.liters > (old_liters_update + 1) ||

    }
    return 0;
}

static void set_pin_io(uint8_t pin_number, bool value) {

    uint8_t data_wr = pin_number;

    if (!value) {
        data_wr = ~data_wr;
    }

    ulp_riscv_i2c_master_set_slave_addr(PI4IO_I2C_ADDR);
    ulp_riscv_i2c_master_set_slave_reg_addr(PI4IO_OUTPUT);
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);
}
static volatile inline uint16_t read_sensor(uint8_t sensor_id, bool led_on) {
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

    if (!led_on)
        reg &= ~LED;
    else
        reg |= LED;

    set_pin_io(reg, true); // Always true !
    // ulp_riscv_delay_cycles(50 * ULP_RISCV_CYCLES_PER_MS);
    // * Make sure LIGHT sensor add is called
    ulp_riscv_i2c_master_set_slave_addr(VEML6030_I2C_ADDR_H);

    

    // Gain x2 and 800ms
    // uint16_t data_wr = (uint16_t)((1 << 11) | ((1 << 6) | (1 << 7)));
    uint8_t data_wr[2] = {0x00, 0x00};
    ulp_riscv_i2c_master_set_slave_reg_addr(VEML6030_ALS_SD);
    ulp_riscv_i2c_master_write_to_device(data_wr, 2);
    // ulp_riscv_delay_cycles(50 * ULP_RISCV_CYCLES_PER_MS); // Delay to ensure stable I/O setup.

    // * Read value
    uint8_t data_rd[2] = {0x00, 0x00};
    ulp_riscv_i2c_master_set_slave_reg_addr(VEML6030_ALS_READ_REG);
    ulp_riscv_i2c_master_read_from_device(data_rd, 2);

    return ((uint16_t)data_rd[0] << 8) | data_rd[1];
    // return (uint16_t) data_rd[0];
}

bool magic_code_box(state_t* state, int sen_a, int sen_b, int sen_c) {
    // * Liter berekening
    // * asin^2(σ)+bsin^2(σ+π/3)+c*sin^3(σ-π/3)
    // * a⋅sin²(σ±ε)+b⋅sin²(σ±ε+π/3)+c⋅sin²(σ±ε-π/3)

    static bool not_inited = true; // if start of pulse detected ignore liters overcommunicating
    if (not_inited) {
        state->phase  = 0;
        state->fine   = 0;
        state->liters = 0;

        state->a_min = sen_a;
        state->b_min = sen_b;
        state->c_min = sen_c;

        state->a_max = 0;
        state->b_max = 0;
        state->c_max = 0;
        not_inited   = false;
    }

    // float alpha_cor = 0.01;
    int alpha_cor = 10; // /1000
    if (state->liters < 2) {
        // alpha_cor = 0.1; // First 2 liters correct faster
        alpha_cor = 100; // /1000
        if (state->liters < 0) {
            state->liters = 0;
        }
    }

    // // * Calculate minimum value
    state->a_min = mini_average(state->a_min, sen_a, alpha_cor);
    state->b_min = mini_average(state->b_min, sen_b, alpha_cor);
    state->c_min = mini_average(state->c_min, sen_c, alpha_cor);

    state->a_max = max_average(state->a_max, sen_a, alpha_cor);
    state->b_max = max_average(state->b_max, sen_b, alpha_cor);
    state->c_max = max_average(state->c_max, sen_c, alpha_cor);

    int a_zc = (state->a_min + state->a_max) >> 1;
    int b_zc = (state->b_min + state->b_max) >> 1;
    int c_zc = (state->c_min + state->c_max) >> 1;

    int sa = sen_a - a_zc;
    int sb = sen_b - b_zc;
    int sc = sen_c - c_zc;

    phase_coarse_iter(state, sa, sb, sc);
    magnitude_offset_iter(state, sen_a, sen_b, sen_c);

    // float liters_float_coarse = (float)state->liters + ((float)state->liters / 6);
    // float liters_float_fine   = (float)state->liters + ((float)state->phase / 6) + ((float)state->fine / (16 * 6));
    // uint32_t mililiters = (uint32_t)(liters_float_fine * 1000);
    // mili_liters_total   = mililiters;
    // liter               = liters_float_fine;

    uint32_t liters_ml     = state->liters * 1000;
    uint32_t phase_ml      = (state->phase * 167) / 6;        // Approximate to 166.67
    state->muino_ml_meters = (state->fine * 1042) / (16 * 6); // Approximate to 10.42

    return 1;
}

int mini_average(int x, int y, int alpha_cor) {

    if ((x + 5) <= y && y > 10) {
        return x;
    } else {
        return (int)((1000 - alpha_cor) * x) / 1000 + (alpha_cor * y) / 1000;
    }
}

int max_average(int x, int y, int alpha_cor) {
    if ((x - 5) >= y && y < 2500) {
        return x;
    } else {
        return ((1000 - alpha_cor) * x) / 1000 + (alpha_cor * y) / 1000;
    }
}

void magnitude_offset_iter(state_t* state, int a, int b, int c) {
    int8_t phase = state->phase;
    if (state->fine > 8) {
        phase = (phase + 7) % 6;
    }
    int* a_min     = &state->a_min;
    int* b_min     = &state->b_min;
    int* c_min     = &state->c_min;
    int* a_max     = &state->a_max;
    int* b_max     = &state->b_max;
    int* c_max     = &state->c_max;
    int* u[6]      = {a_max, b_min, c_max, a_min, b_max, c_min};
    int* signal[6] = {&a, &b, &c, &a, &b, &c};
    if (state->liters > 2) {
        if ((state->fine > 14) || state->fine < 3)
            *u[phase] = (((((*u[phase]) << SMOOTHING_FACTOR) - (*u[phase]) + *signal[phase]) >> SMOOTHING_FACTOR) + 1 - (phase & 1));
    }
}

// check with three 2pi/3 steps peak autocorrelation and adjust towards max by pi/3 steps
// 2cos(0)=2 2cos(pi/3)=1 2cos(2pi/3)=-1 2cos(pi)=-2 2cos(4pi/3)=-1 2cos(5pi/3)=1
void phase_coarse_iter(state_t* state, int a, int b, int c) {
    int8_t*  phase  = &state->phase;  // ?
    int32_t* liters = &state->liters; // ?
    short    pn[5];
    if (*phase & 1)
        pn[0] = a + a - b - c, pn[1] = b + b - a - c,
        pn[2] = c + c - a - b; // same
    else
        pn[0]     = b + c - a - a, // less
            pn[1] = a + c - b - b, // more
            pn[2] = a + b - c - c; // same
    pn[3] = pn[0], pn[4] = pn[1];
    short i = *phase > 2 ? *phase - 3 : *phase;
    if (pn[i + 2] < pn[i + 1] && pn[i + 2] < pn[i])
        if (pn[i + 1] > pn[i])
            (*phase)++;
        else
            (*phase)--;
    if (*phase == 6)
        (*liters)++, *phase = 0;
    else if (*phase == -1)
        (*liters)--, *phase = 5;
}