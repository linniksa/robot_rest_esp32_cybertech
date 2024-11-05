#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "battery.hpp"
#include "config.hpp"
#include "imu.hpp"
#include "led.hpp"
#include "motor.hpp"
#include "server.hpp"
#include "ttf.hpp"

static const char *TAG = __FILENAME__;

#include "driver/pulse_cnt.h"
extern pcnt_unit_handle_t pcnt_unit_left;
extern pcnt_unit_handle_t pcnt_unit_right;

Battery battery;
Server server;
Imu imu;
Ttf ttf;
Led led;
Robot robot;

extern "C" void app_main() {
  ESP_LOGI(TAG, "Program start...");
  ESP_ERROR_CHECK(nvs_flash_init());

  battery.init();
  led.init();
  imu.init();
#ifdef ENABLE_DISTANCE_SENSOR
  ttf.init();
#endif // ENABLE_DISTANCE_SENSOR
  robot.init();
  server.init();

  DELAY(2000);

  while (1) {
    DELAY(100);
  }
}