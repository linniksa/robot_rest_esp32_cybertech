#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "vl53l0x.hpp"
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
  VL53L0X sensor[6];
  bool enabled_sensors[6];

public:
  Ttf();
  ~Ttf();
  void init();
  void get_laser_data(uint16_t laser_buff[10]);
  void set_enabled_sensors(bool l,bool l45,bool f,bool r45,bool r,bool b);
  void set_interval(uint8_t interval);
};