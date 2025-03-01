#include "esphome.h"
#include "sensor_defs.h"
#include "muino_3phase_i2c.h"

namespace esphome {
namespace muino_3phase_i2c {

bool Muino3PhaseI2CSensor::phase_coarse(int a, int b, int c) {
    static bool first = true;

    int max = 1500;
    int min = 5;

    if (a < min || b < min || c < min ){
        ESP_LOGW("light_level", "Too dark, ignoring this measurement");
        return false;
    }

    if (a > max || b > max || c > max){
        ESP_LOGW("light_level", "Too bright, ignoring this measurement");
        return false;
    }

    if (!init_ok) {
        state.liters = 0;
        state.phase = 0;
        state.a_min = a;
        state.b_min = b;
        state.c_min = c;
        state.a_max = 0;
        state.b_max = 0;
        state.c_max = 0;
        init_ok = true;
        return 0;
    }

    float alpha_cor = 0.001;
    if (state.liters < 2) {
        alpha_cor = 0.1; // when 2 liter not found correct harder
        if (state.liters < 0) {
            state.liters = 0;
        }
    }

    state.a_min = mini_average(state.a_min, a, alpha_cor);
    state.b_min = mini_average(state.b_min, b, alpha_cor);
    state.c_min = mini_average(state.c_min, c, alpha_cor);
    state.a_max = max_average(state.a_max, a, alpha_cor);
    state.b_max = max_average(state.b_max, b, alpha_cor);
    state.c_max = max_average(state.c_max, c, alpha_cor);

    /*
    TODO
    if (manual_calibration) {
        // Manual offsets
        a -= id(manual_offset_a);
        b -= id(manual_offset_b);
        c -= id(manual_offset_c);
    } else {
        // Auto-calibration offsets
        a -= (id(a_min) + id(a_max)) >> 1;
        b -= (id(b_min) + id(b_max)) >> 1;
        c -= (id(c_min) + id(c_max)) >> 1;
    }
    */

    short    pn[5];
    if (state.phase & 1)
        pn[0] = a + a - b - c, pn[1] = b + b - a - c,
        pn[2] = c + c - a - b; // same
    else
        pn[0]     = b + c - a - a, // less
            pn[1] = a + c - b - b, // more
            pn[2] = a + b - c - c; // same
    pn[3] = pn[0], pn[4] = pn[1];

    short i = state.phase > 2 ? state.phase - 3 : state.phase;
    if (pn[i + 2] < pn[i + 1] && pn[i + 2] < pn[i]){
        time_since_last_flow_ = millis();
        if (pn[i + 1] > pn[i])
            state.phase++;
        else
            state.phase--;
    }

    if (state.phase == 6) {
        state.liters++;
        state.phase = 0;

        update_consumption(1);
    }
    else if (state.phase == -1) {
        state.liters--;
        state.phase = 5;

        update_consumption(-1);
    }

    return true;
}

void Muino3PhaseI2CSensor::update_consumption(int8_t value) {
    main_consumption += value;
    secondary_consumption += value;
    tertiary_consumption += value;
    last_consumption_ += value;
}

float Muino3PhaseI2CSensor::mini_average(float x, float y, float alpha_cor) {

    if ((x + 5) <= y && y > 10) {
        return x;
    } else {
        return (1 - alpha_cor) * x + alpha_cor * y;
    }
}

float Muino3PhaseI2CSensor::max_average(float x, float y, float alpha_cor) {
    if ((x - 5) >= y && y < 2500) {
        return x;
    } else {
        return (1 - alpha_cor) * x + alpha_cor * y;
    }
}

bool Muino3PhaseI2CSensor::write_(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);

    if (Wire.endTransmission() != 0) {
        ESP_LOGE(TAG, "Failed to write to 0x%02X", addr);
        return false;
    }

    return true;
}

bool Muino3PhaseI2CSensor::write_(uint8_t addr, uint8_t reg, uint8_t* val, size_t length) {
    Wire.beginTransmission(addr);
    Wire.write(reg);

    for (size_t i = 0; i < length; i++) {
        Wire.write(val[i]);
    }

    if (Wire.endTransmission() != 0) {
        ESP_LOGE(TAG, "Failed to write to 0x%02X", addr);
        return false;
    }

    return true;
}

bool Muino3PhaseI2CSensor::read_(uint8_t addr, uint8_t reg, uint8_t* data, size_t length) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        ESP_LOGE(TAG, "Failed to read from 0x%02X", addr);
        return false;
    }

    Wire.requestFrom(addr, length);
    for (size_t i = 0; i < length && Wire.available(); i++) {
        data[i] = Wire.read();
    }

    return true;
}

void Muino3PhaseI2CSensor::set_pin_io_(uint8_t pin_number, bool value) {
    // GOOD
    uint8_t current;
    read_(PI4IO_I2C_ADDR, PI4IO_OUTPUT, &current, 1);
    if (value)
        current |= (1 << pin_number);
    else
        current &= ~(1 << pin_number);

    write_(PI4IO_I2C_ADDR, PI4IO_OUTPUT, current);
}

void Muino3PhaseI2CSensor::init_light_sensor_(uint8_t sensor_id) {
    uint8_t reg = 0x00;
    if (sensor_id == 0) {
        set_pin_io_(3, true);
    } else if (sensor_id == 1) {
        set_pin_io_(4, true);
    } else if (sensor_id == 2) {
        set_pin_io_(5, true);
    } else {
        return;
    }

    // Gain: x2, It: 100ms
    uint8_t data_wr[2] = {0x00, (1 << 0)};
    write_(VEML6030_I2C_ADDR_H, VEML6030_ALS_SD, (uint8_t*)&data_wr, 2);

    if (sensor_id == 0) {
        set_pin_io_(3, false);
    } else if (sensor_id == 1) {
        set_pin_io_(4, false);
    } else if (sensor_id == 2) {
        set_pin_io_(5, false);
    } else {
        return;
    }
}

uint16_t Muino3PhaseI2CSensor::read_sensor_(uint8_t sensor_id) {
    uint16_t val = 0;

    // Set address input for correct sensor
    if (sensor_id == 0)
        this->set_pin_io_(3, true);
    else if (sensor_id == 1)
        this->set_pin_io_(4, true);
    else if (sensor_id == 2)
        this->set_pin_io_(5, true);

    // Read sensor value
    read_(VEML6030_I2C_ADDR_H, VEML6030_ALS_READ_REG, (uint8_t*)&val, 2);

    if (sensor_id == 0)
        set_pin_io_(3, false);
    else if (sensor_id == 1)
        set_pin_io_(4, false);
    else if (sensor_id == 2)
        set_pin_io_(5, false);

    return val;
}

// Main sensor implementation
void Muino3PhaseI2CSensor::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Muino 3-Phase Sensor...");

    uint8_t data_wr;

    data_wr = SENS0 | SENS1 | SENS2 | LED;
    write_(PI4IO_I2C_ADDR, PI4IO_IO_DIRECTION, data_wr);

    write_(PI4IO_I2C_ADDR, PI4IO_OUTPUT, 0);

    data_wr = ~(SENS0 | SENS1 | SENS2 | LED);
    write_(PI4IO_I2C_ADDR, PI4IO_OUTPUT_HI_IMPEDANCE, data_wr);

    init_light_sensor_(0);
    init_light_sensor_(1);
    init_light_sensor_(2);

    state.liters = 0;
    state.phase = 0;
    state.a_min = 0;
    state.b_min = 0;
    state.c_min = 0;
    state.a_max = 0;
    state.b_max = 0;
    state.c_max = 0;

    main_consumption = 0;
    secondary_consumption = 0;
    tertiary_consumption = 0;

    main_ml_reset = 0;
    secondary_ml_reset = 0;
    tertiary_ml_reset = 0;
}

void Muino3PhaseI2CSensor::update_values_() {
    static uint32_t last_time = 0;

    float ml_part = state.phase / 6.0;

    if (total_sensor_ != nullptr)
        total_sensor_->publish_state((uint32_t)state.liters);

    if (ml_sensor_ != nullptr)
        ml_sensor_->publish_state((uint32_t)(1000 * (state.liters + ml_part)));

    if (main_consumption_sensor_ != nullptr)
        main_consumption_sensor_->publish_state(
            (float)(main_consumption + (ml_part - main_ml_reset))
        );

    if (secondary_consumption_sensor_ != nullptr)
        secondary_consumption_sensor_->publish_state(
            (float)(secondary_consumption + (ml_part - secondary_ml_reset))
        );

    if (tertiary_consumption_sensor_ != nullptr)
        tertiary_consumption_sensor_->publish_state(
            (float)(tertiary_consumption) + (ml_part - tertiary_ml_reset)
        );

    if (time_since_last_flow_sensor_ != nullptr)
        time_since_last_flow_sensor_->publish_state((millis() - time_since_last_flow_) / 1000);

    if (last_consumption_sensor_ != nullptr)
        last_consumption_sensor_->publish_state(last_consumption_);
}

void Muino3PhaseI2CSensor::reset_total() {
    init_ok = false;
    state.liters = 0;
    state.phase = 0;
    last_consumption_ = 0;
    time_since_last_flow_ = millis();
    update_values_();
    ESP_LOGI(TAG, "Total consumption reset to 0");
}

void Muino3PhaseI2CSensor::reset_main_consumption() {
    main_consumption = 0;
    main_ml_reset = state.phase / 6.0;
    update_values_();
};

void Muino3PhaseI2CSensor::reset_secondary_consumption() {
    secondary_consumption = 0;
    secondary_ml_reset = state.phase / 6.0;
    update_values_();
};

void Muino3PhaseI2CSensor::reset_tertiary_consumption() {
    tertiary_consumption = 0;
    tertiary_ml_reset = state.phase / 6.0;
    update_values_();
};

void Muino3PhaseI2CSensor::set_led(bool state) {
    this->set_pin_io_(6, state);
}

void Muino3PhaseI2CSensor::update() {
    static uint32_t last_time = 0;
    uint32_t now = millis();
    uint32_t interval = now - last_time;
    char buffer[50];

    if (millis() - last_update_ >= 1000) {
        if (debug_mode_) {
            if (a_sensor_ != nullptr)
                snprintf(
                    buffer, sizeof(buffer),
                    "[%d, %d] raw: %d, avg: %d",
                    sen_a_dark_, sen_a_light_, state.a, history_a_[history_index_]
                );
                a_sensor_->set_entity_category(esphome::EntityCategory::ENTITY_CATEGORY_DIAGNOSTIC);
                a_sensor_->publish_state(buffer);

            if (b_sensor_ != nullptr)
                snprintf(
                    buffer, sizeof(buffer),
                    "[%d, %d] raw: %d, avg: %d",
                    sen_b_dark_, sen_b_light_, state.b, history_b_[history_index_]
                );
                b_sensor_->publish_state(buffer);

            if (c_sensor_ != nullptr)
                snprintf(
                    buffer, sizeof(buffer),
                    "[%d, %d] raw: %d, avg: %d",
                    sen_c_dark_, sen_c_light_, state.c, history_c_[history_index_]
                );
                c_sensor_->publish_state(buffer);

            if (phase_sensor_ != nullptr)
                phase_sensor_->publish_state(state.phase);
        }

        update_values_();
        last_update_ = millis();
    }

    // Consumption reset if last water flow > 1 minute
    if ((millis() - time_since_last_flow_) / 1000 > 60) {
        last_consumption_ = 0;
    }

    switch (status) {
    case 0:
        // First: Do the setup
        setup();

        update_values_();
    case 1:
        // Second: Read the sensor without LED
        sen_a_dark_ = read_sensor_(0);
        sen_b_dark_ = read_sensor_(1);
        sen_c_dark_ = read_sensor_(2);

        // Power ON LED for the next step
        set_led(true);

        status = 2;
        break;
    case 2:
        // Third: Read the sensor with LED
        sen_a_light_ = read_sensor_(0);
        sen_b_light_ = read_sensor_(1);
        sen_c_light_ = read_sensor_(2);

        // Power OFF LED for the next step
        set_led(false);

        history_a_[history_index_] = sen_a_light_ - sen_a_dark_;
        history_b_[history_index_] = sen_b_light_ - sen_b_dark_;
        history_c_[history_index_] = sen_c_light_ - sen_c_dark_;
        history_index_ = (history_index_ + 1) % 3;

        state.a = (history_a_[0] + history_a_[1] + history_a_[2]) / 3;
        state.b = (history_b_[0] + history_b_[1] + history_b_[2]) / 3;
        state.c = (history_c_[0] + history_c_[1] + history_c_[2]) / 3;

        /*
        uint8_t val;
        read_(VEML6030_I2C_ADDR_H, VEML6030_ALS_SD, (uint8_t*)&val, 2);
        ESP_LOGI(TAG, "X %d", val);
        */

        ESP_LOGI(TAG, "Valeurs %d: %d, %d | Smoothed: %d", 0, sen_a_light_, sen_a_dark_, state.a);
        ESP_LOGI(TAG, "Valeurs %d: %d, %d | Smoothed: %d", 1, sen_b_light_, sen_b_dark_, state.b);
        ESP_LOGI(TAG, "Valeurs %d: %d, %d | Smoothed: %d", 2, sen_c_light_, sen_c_dark_, state.c);
        // ESP_LOGD(TAG, "mL %d", this->calculate(&state));
        ESP_LOGI(TAG, "--");

        phase_coarse(state.a, state.b, state.c);

        status = 1;
        break;
    }

    ESP_LOGW(TAG, "Status: %i, Interval: %ums, Duration %ums, hindex: %i", status, interval, millis() - now, history_index_);
    last_time = now;
}

void Muino3PhaseI2CSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "Muino 3-Phase Sensor:");
    LOG_I2C_DEVICE(this);
    if (this->is_failed()) {
        ESP_LOGE(TAG, "Communication failed");
    }
}

} // namespace muino_3phase_i2c
} // namespace esphome