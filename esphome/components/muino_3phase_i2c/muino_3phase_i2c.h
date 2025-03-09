#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/i2c/i2c.h"

#include "sensor_defs.h"

namespace esphome {
namespace muino_3phase_i2c {

static const char *const TAG = "muino_3phase_i2c";

class Muino3PhaseI2CSensor : public PollingComponent, public i2c::I2CDevice {
public:
    Muino3PhaseI2CSensor(): PollingComponent(250) {}
    void setup() override;
    void update() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::DATA; }
    void before_flash() { save_consumptions(true); }
    void on_shutdown() { save_consumptions(true); }

    void set_index_sensor(sensor::Sensor *sensor) { index_sensor_ = sensor; }
    void set_main_consumption_sensor(sensor::Sensor *sensor) { main_consumption_sensor_ = sensor; }
    void set_secondary_consumption_sensor(sensor::Sensor *sensor) { secondary_consumption_sensor_ = sensor; }
    void set_tertiary_consumption_sensor(sensor::Sensor *sensor) { tertiary_consumption_sensor_ = sensor; }
    void set_a_sensor(text_sensor::TextSensor *sensor) { a_sensor_ = sensor; }
    void set_b_sensor(text_sensor::TextSensor *sensor) { b_sensor_ = sensor; }
    void set_c_sensor(text_sensor::TextSensor *sensor) { c_sensor_ = sensor; }
    void set_phase_sensor(sensor::Sensor *sensor) { phase_sensor_ = sensor; }
    void set_time_since_last_flow_sensor(sensor::Sensor *sensor) { time_since_last_flow_sensor_ = sensor; }
    void set_previous_consumption_sensor(sensor::Sensor *sensor) { previous_consumption_sensor_ = sensor; }
    void set_current_consumption_sensor(sensor::Sensor *sensor) { current_consumption_sensor_ = sensor; }
    void set_measurements_consistency_sensor(binary_sensor::BinarySensor *sensor) { measurements_consistency_sensor_ = sensor; }
    void set_flow_rate_sensor(sensor::Sensor *sensor) { flow_rate_sensor_ = sensor; }

    void reset_total();
    void reset_main_consumption();
    void reset_secondary_consumption();
    void reset_tertiary_consumption();

    void save_consumptions(bool shutdown_occured = false);
    void restore_consumptions(bool startup_occured = false);

    bool phase_coarse(int sen_a, int sen_b, int sen_c);
    float mini_average(float x, float y, float alpha_cor);
    float max_average(float x, float y, float alpha_cor);

    void set_led(bool state);

    void set_index(int value);
    uint32_t get_index(int value) const { return index_; }

    void set_debug_mode(bool value) {
        debug_mode_ = value;
    }

    float flow_rate;

    state_t state;

    uint32_t main_consumption;
    uint32_t secondary_consumption;
    uint32_t tertiary_consumption;

    // Used during the reset consumption to be able
    // to set 0 for the ml part
    float main_ml_reset;
    float secondary_ml_reset;
    float tertiary_ml_reset;

protected:
    ESPPreferenceObject index_pref_;
    ESPPreferenceObject main_pref_;
    ESPPreferenceObject secondary_pref_;
    ESPPreferenceObject tertiary_pref_;

    // We don't want a direct access to this variable
    uint32_t index_;

    // After consumption has been saved during a shutdown event (`before_flash`, `on_shutdown`),
    // a specific value (shutdown_value_: 0xCC) is written.
    // At the next startup, if this value is not the one expected, it is assumed that
    // the consumption saving did not take place, and therefore that consumption is
    // no longer completely in phase.
    uint8_t shutdown_value_ = 0xCC;
    ESPPreferenceObject shutdown_consistency_pref_;

    bool init_ok_ = false;

    uint32_t sen_a_dark_;
    uint32_t sen_b_dark_;
    uint32_t sen_c_dark_;

    uint32_t sen_a_light_;
    uint32_t sen_b_light_;
    uint32_t sen_c_light_;

    int32_t history_a_[3];
    int32_t history_b_[3];
    int32_t history_c_[3];

    int history_index_ = 0;

    uint32_t last_update_ = 0;

    uint32_t time_since_last_flow_ = 0;
    uint32_t current_consumption_ = 0;
    uint32_t previous_consumption_ = 0;

    float flow_rate_last_time_ = 0;
    float flow_rate_values_[5] = {0};
    uint8_t flow_rate_index_ = 0;

    sensor::Sensor *index_sensor_{nullptr};
    sensor::Sensor *main_consumption_sensor_{nullptr};
    sensor::Sensor *secondary_consumption_sensor_{nullptr};
    sensor::Sensor *tertiary_consumption_sensor_{nullptr};
    text_sensor::TextSensor *a_sensor_{nullptr};
    text_sensor::TextSensor *b_sensor_{nullptr};
    text_sensor::TextSensor *c_sensor_{nullptr};
    sensor::Sensor *phase_sensor_{nullptr};
    sensor::Sensor *time_since_last_flow_sensor_{nullptr};
    sensor::Sensor *previous_consumption_sensor_{nullptr};
    sensor::Sensor *current_consumption_sensor_{nullptr};
    binary_sensor::BinarySensor *measurements_consistency_sensor_{nullptr};
    sensor::Sensor *flow_rate_sensor_{nullptr};

    text_sensor::TextSensor *debug_text_sensor_{nullptr};

    bool debug_mode_{false};

    void set_pin_io_(uint8_t pin_number, bool value);
    bool write_(uint8_t addr, uint8_t reg, uint8_t val);
    bool write_(uint8_t addr, uint8_t reg, uint8_t* val, size_t length);
    bool read_(uint8_t addr, uint8_t reg, uint8_t* data, size_t length);
    void init_light_sensor_(uint8_t sensor_id);
    uint16_t read_sensor_(uint8_t sensor_id);
    bool write_light_sensor_(uint8_t reg, uint16_t val);
    void save_total_();
    void update_values_();
    void update_consumption(int8_t value);
    void flow_rate_push_value_(float value);
    void flow_rate_increment_();
    void flow_rate_reset_();
};

} // namespace muino_3phase_i2c
} // namespace esphome
