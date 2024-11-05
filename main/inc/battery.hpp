#pragma once
#include "esp_adc/adc_oneshot.h"

class Battery {
public:
  Battery();
  ~Battery();
  float get_voltage();
  void init();

private:
  adc_oneshot_unit_handle_t adc1_handle = NULL;
  adc_cali_handle_t adc_cali_handle = NULL;
};
