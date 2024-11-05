#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include <cstdint>

class Ttf {
private:
  static void task_wrapper(void *pvParameters) {
    Ttf *pThis = static_cast<Ttf *>(pvParameters);
    pThis->task();
  }
  void task();
  esp_err_t i2c_switch_to_ch(uint8_t ch);
  uint16_t laser_values[10];

public:
  Ttf();
  ~Ttf();
  void init();
  void get_laser_data(uint16_t laser_buff[10]);
};