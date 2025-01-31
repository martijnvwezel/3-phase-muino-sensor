#include "esphome.h"
#include "muino_3phase_sensor.h"

namespace esphome {
namespace muino_3phase {

// LED Control implementation
LEDControl::LEDControl(Muino3PhaseSensor* sensor) : sensor_(sensor) {
  sensor_->set_pin_io_(LED_PIN, true);
  delay(LED_STABILIZATION_TIME);
}

LEDControl::~LEDControl() {
  sensor_->set_pin_io_(LED_PIN, false);
  delay(LED_STABILIZATION_TIME);
}

// Main sensor implementation
void Muino3PhaseSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Muino 3-Phase Sensor...");
  
  if (!this->write_reg_(0x03, (1 << 3) | (1 << 4) | (1 << 5) | (1 << LED_PIN))) {
    ESP_LOGE(TAG, "Failed to initialize IO extender");
    this->mark_failed();
    return;
  }

  this->init_io_extender_();
  this->set_pin_io_(LED_PIN, true);
  
  ESP_LOGD(TAG, "Muino 3-Phase Sensor initialized");

  this->total_pref_ = global_preferences->make_preference<uint32_t>(this->address_);
  uint32_t total_saved;
  if (this->total_pref_.load(&total_saved)) {
    this->total_milliliters_ = total_saved;
    ESP_LOGD(TAG, "Loaded total consumption: %.3f m³", total_saved / 1000.0);
  }
}

void Muino3PhaseSensor::loop() {
  if (millis() - this->last_update_ < 100) {
    return;
  }
  
  LEDControl led_control(this);
  
  uint32_t sen_a = this->read_light_sensor_(0);
  uint32_t sen_b = this->read_light_sensor_(1);
  uint32_t sen_c = this->read_light_sensor_(2);
  
  uint32_t sen_a_dark = this->read_light_sensor_(0);
  uint32_t sen_b_dark = this->read_light_sensor_(1);
  uint32_t sen_c_dark = this->read_light_sensor_(2);
  
  if (this->process_readings_(sen_a, sen_b, sen_c, sen_a_dark, sen_b_dark, sen_c_dark)) {
    if (this->flow_sensor_ != nullptr)
      this->flow_sensor_->publish_state(this->current_flow_);
    
    if (this->total_sensor_ != nullptr)
      this->total_sensor_->publish_state(this->total_milliliters_ / 1000.0);
      
    if (this->sensor_a_ != nullptr)
      this->sensor_a_->publish_state(sen_a - sen_a_dark);
      
    if (this->sensor_b_ != nullptr)
      this->sensor_b_->publish_state(sen_b - sen_b_dark);
      
    if (this->sensor_c_ != nullptr)
      this->sensor_c_->publish_state(sen_c - sen_c_dark);
  }
  
  this->last_update_ = millis();
}

void Muino3PhaseSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Muino 3-Phase Sensor:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication failed");
  }
}

bool Muino3PhaseSensor::write_reg_(uint8_t reg, uint8_t val) {
  return this->write_byte(reg, val);
}

uint8_t Muino3PhaseSensor::read_reg_(uint8_t reg) {
  uint8_t val;
  if (!this->read_byte(reg, &val)) {
    return 0;
  }
  return val;
}

void Muino3PhaseSensor::set_pin_io_(uint8_t pin_number, bool value) {
  uint8_t current = this->read_reg_(0x05);
  if (value)
    current |= (1 << pin_number);
  else
    current &= ~(1 << pin_number);
  this->write_reg_(0x05, current);
}

uint16_t Muino3PhaseSensor::read_light_sensor_(uint8_t sensor_id) {
  uint16_t val = 0;
  
  // Set address input for correct sensor
  if (sensor_id == 0)
    this->set_pin_io_(3, true);
  else if (sensor_id == 1)
    this->set_pin_io_(4, true);
  else if (sensor_id == 2)
    this->set_pin_io_(5, true);
    
  // Read sensor value
  this->read_bytes(0x04, (uint8_t*)&val, 2);
  
  // Reset address inputs
  this->set_pin_io_(3, false);
  this->set_pin_io_(4, false);
  this->set_pin_io_(5, false);
  
  return val;
}

bool Muino3PhaseSensor::write_light_sensor_(uint8_t reg, uint16_t val) {
  return this->write_bytes(reg, (uint8_t*)&val, 2);
}

void Muino3PhaseSensor::init_io_extender_() {
  // Configure IO direction
  this->write_reg_(0x03, (1 << 3) | (1 << 4) | (1 << 5) | (1 << LED_PIN));
  
  // Configure output state
  this->write_reg_(0x05, 0);
  
  // Configure high-impedance
  this->write_reg_(0x07, ~((1 << 3) | (1 << 4) | (1 << 5) | (1 << LED_PIN)));
}

float Muino3PhaseSensor::mini_average_(float x, float y, float alpha_cor) {
  if ((x + 5) <= y && y > 10) {
    return x;
  }
  return (1 - alpha_cor) * x + alpha_cor * y;
}

float Muino3PhaseSensor::max_average_(float x, float y, float alpha_cor) {
  if ((x - 5) >= y && y < 2500) {
    return x;
  }
  return (1 - alpha_cor) * x + alpha_cor * y;
}

void Muino3PhaseSensor::magnitude_offset_iter_(int a, int b, int c) {
  int8_t phase = this->state_.phase;
  if (this->state_.fine > 8) {
    phase = (phase + 7) % 6;
  }

  int* values[6] = {&this->state_.a_max, &this->state_.b_min, &this->state_.c_max,
                    &this->state_.a_min, &this->state_.b_max, &this->state_.c_min};
  int* signals[6] = {&a, &b, &c, &a, &b, &c};

  if (this->state_.liters > 2) {
    if ((this->state_.fine > 14) || this->state_.fine < 3) {
      *values[phase] = (((((*values[phase]) << SMOOTHING_FACTOR) - (*values[phase]) + *signals[phase]) 
                        >> SMOOTHING_FACTOR) + 1 - (phase & 1));
    }
  }
}

void Muino3PhaseSensor::phase_coarse_iter_(int a, int b, int c) {
  // Implementation from your original code
  short pn[5];
  if (this->state_.phase & 1) {
    pn[0] = a + a - b - c;
    pn[1] = b + b - a - c;
    pn[2] = c + c - a - b;
  } else {
    pn[0] = b + c - a - a;
    pn[1] = a + c - b - b;
    pn[2] = a + b - c - c;
  }
  pn[3] = pn[0];
  pn[4] = pn[1];

  short i = this->state_.phase > 2 ? this->state_.phase - 3 : this->state_.phase;
  if (pn[i + 2] < pn[i + 1] && pn[i + 2] < pn[i]) {
    if (pn[i + 1] > pn[i])
      this->state_.phase++;
    else
      this->state_.phase--;
  }

  if (this->state_.phase == 6) {
    this->state_.liters++;
    this->state_.phase = 0;
  } else if (this->state_.phase == -1) {
    this->state_.liters--;
    this->state_.phase = 5;
  }
}

bool Muino3PhaseSensor::process_readings_(int32_t sen_a, int32_t sen_b, int32_t sen_c,
                                        int32_t sen_a_dark, int32_t sen_b_dark, int32_t sen_c_dark) {
  // Remove dark current
  sen_a -= sen_a_dark;
  sen_b -= sen_b_dark;
  sen_c -= sen_c_dark;

  if (!this->initialized_) {
    this->state_.phase = 0;
    this->state_.fine = 0;
    this->state_.liters = 0;
    this->state_.a_min = sen_a;
    this->state_.b_min = sen_b;
    this->state_.c_min = sen_c;
    this->state_.a_max = 0;
    this->state_.b_max = 0;
    this->state_.c_max = 0;
    this->initialized_ = true;
    return false;
  }

  float alpha_cor = (this->total_milliliters_ < 2000) ? 0.1f : 0.01f;

  this->state_.a_min = this->mini_average_(this->state_.a_min, sen_a, alpha_cor);
  this->state_.b_min = this->mini_average_(this->state_.b_min, sen_b, alpha_cor);
  this->state_.c_min = this->mini_average_(this->state_.c_min, sen_c, alpha_cor);

  this->state_.a_max = this->max_average_(this->state_.a_max, sen_a, alpha_cor);
  this->state_.b_max = this->max_average_(this->state_.b_max, sen_b, alpha_cor);
  this->state_.c_max = this->max_average_(this->state_.c_max, sen_c, alpha_cor);

  int a_zc = (this->state_.a_min + this->state_.a_max) >> 1;
  int b_zc = (this->state_.b_min + this->state_.b_max) >> 1;
  int c_zc = (this->state_.c_min + this->state_.c_max) >> 1;

  int sa = sen_a - a_zc;
  int sb = sen_b - b_zc;
  int sc = sen_c - c_zc;

  this->phase_coarse_iter_(sa, sb, sc);
  this->phase_fine_iter_(sa, sb, sc);
  this->magnitude_offset_iter_(sen_a, sen_b, sen_c);

  return true;
}

void Muino3PhaseSensor::save_total_() {
  this->total_pref_.save(&this->total_milliliters_);
}

int Muino3PhaseSensor::phase_fine_iter_(int a, int b, int c) {
  const float step = (M_PI) / (3 * AC_STEPS);
  std::array<float, AC_STEPS * 3 + 1> array{};
  int largest_index = -AC_STEPS;
  float largest = 0;

  for (int i = -AC_STEPS; i <= (AC_STEPS * 2); i++) {
    float x = (this->state_.phase * M_PI / 3) + (step * i);
    float cora = a * cosf(x);
    float corb = b * cosf(x + PI2_3);
    float corc = c * cosf(x - PI2_3);
    float cor = cora + corb + corc;
    array[i + AC_STEPS] = cor;
    if (cor > largest) {
      largest = cor;
      largest_index = i;
      this->state_.fine = largest_index;
    }
  }
  return largest_index;
}

} // namespace muino_3phase
} // namespace esphome
