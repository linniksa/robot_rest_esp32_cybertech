#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class Imu {
private:
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  static void task_wrapper(void *pvParameters) {
    Imu *pThis = static_cast<Imu *>(pvParameters);
    pThis->task();
  }
  void task();

public:
  Imu(/* args */);
  ~Imu();
  void init();
  float get_yaw();
  float get_pitch();
  float get_roll();
};