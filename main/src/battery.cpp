#include "battery.hpp"
#include "config.hpp"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// #include "esp_log.h"
// static const char *TAG = __FILENAME__;

#define R7 10000.0
#define R3 30000.0

Battery::Battery() { }

void Battery::init() {
  adc_oneshot_unit_init_cfg_t init_config = {};
  init_config.unit_id = BAT_CHECK_ADC;
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {};
  config.atten = ADC_ATTEN_DB_12;
  config.bitwidth = ADC_BITWIDTH_12;
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, BAT_CHECK_CH, &config));

  adc_cali_curve_fitting_config_t cali_config = {};
  cali_config.unit_id = BAT_CHECK_ADC;
  cali_config.atten = ADC_ATTEN_DB_12;
  cali_config.bitwidth = ADC_BITWIDTH_12;

  ESP_ERROR_CHECK(
      adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle));
}

float Battery::get_voltage() {
  int raw_adc = 0;
  adc_oneshot_read(adc1_handle, BAT_CHECK_CH, &raw_adc);
  int voltage_mv = 0;

  adc_cali_raw_to_voltage(adc_cali_handle, raw_adc, &voltage_mv);

  return voltage_mv * ((R7 + R3) / R7) / 1000.0 + 0.5; // +0.5V - hack
}

Battery::~Battery() {}