#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include <array>
#include <cmath>

namespace esphome {
namespace muino_3phase {

// Device constants
static const char *const TAG = "muino_3phase";
static const uint8_t LED_PIN = 6;
static const uint32_t LED_STABILIZATION_TIME = 15;
static const uint8_t SMOOTHING_FACTOR = 3;
static const uint8_t AC_STEPS = 16;
static const float PI2_3 = 2.09439510239f;
static const float PI_3 = 1.0471975512f;

// I2C addresses
static const uint8_t VEML6030_I2C_ADDR_H = 0x48;
static const uint8_t PI4IO_I2C_ADDR = 0x43;

// Register addresses
static const uint8_t PI4IO_IO_DIRECTION = 0x03;
static const uint8_t PI4IO_OUTPUT = 0x05;
static const uint8_t PI4IO_OUTPUT_HI_IMPEDANCE = 0x07;

class Muino3PhaseSensor;

// Helper class for automatic LED control
class LEDControl {
 public:
  LEDControl(Muino3PhaseSensor* sensor);
  ~LEDControl();
 private:
  Muino3PhaseSensor* sensor_;
};

class Muino3PhaseSensor : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_flow_sensor(sensor::Sensor *flow_sensor) { flow_sensor_ = flow_sensor; }
  void set_total_sensor(sensor::Sensor *total_sensor) { total_sensor_ = total_sensor; }
  void set_sensor_a(sensor::Sensor *sensor_a) { sensor_a_ = sensor_a; }
  void set_sensor_b(sensor::Sensor *sensor_b) { sensor_b_ = sensor_b; }
  void set_sensor_c(sensor::Sensor *sensor_c) { sensor_c_ = sensor_c; }

  void reset_total() {
    this->total_milliliters_ = 0;
    this->save_total_();
    ESP_LOGI(TAG, "Total consumption reset to 0");
  }

  void calibrate() {
    this->initialized_ = false;
    ESP_LOGI(TAG, "Sensor calibration initiated");
  }

  friend class LEDControl;

 protected:
  struct SensorState {
    int8_t phase{0};
    int8_t fine{0};
    int32_t liters{0};
    int a_min{2500};
    int b_min{2500};
    int c_min{2500};
    int a_max{0};
    int b_max{0};
    int c_max{0};
  } state_;

  sensor::Sensor *flow_sensor_{nullptr};
  sensor::Sensor *total_sensor_{nullptr};
  sensor::Sensor *sensor_a_{nullptr};
  sensor::Sensor *sensor_b_{nullptr};
  sensor::Sensor *sensor_c_{nullptr};

  uint32_t last_update_{0};
  float current_flow_{0.0f};
  uint32_t total_milliliters_{0};
  bool initialized_{false};
  ESPPreferenceObject total_pref_;

  void set_pin_io_(uint8_t pin_number, bool value);
  bool write_reg_(uint8_t reg, uint8_t val);
  uint8_t read_reg_(uint8_t reg);
  uint16_t read_light_sensor_(uint8_t sensor_id);
  bool write_light_sensor_(uint8_t reg, uint16_t val);
  void init_io_extender_();
  void save_total_();

  float mini_average_(float x, float y, float alpha_cor);
  float max_average_(float x, float y, float alpha_cor);
  void magnitude_offset_iter_(int a, int b, int c);
  void phase_coarse_iter_(int a, int b, int c);
  int phase_fine_iter_(int a, int b, int c);
  bool process_readings_(int32_t sen_a, int32_t sen_b, int32_t sen_c,
                        int32_t sen_a_dark, int32_t sen_b_dark, int32_t sen_c_dark);
};

} // namespace muino_3phase
} // namespace esphome
