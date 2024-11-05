#include "led.hpp"
#include "config.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include <cstdint>

Led::Led() {}
Led::~Led() {}

void Led::init() {
  gpio_reset_pin(LED_1);
  gpio_reset_pin(LED_2);
  gpio_reset_pin(LED_3);
  gpio_reset_pin(LED_WIFI);
  gpio_reset_pin(LED_BT);
  gpio_reset_pin(LED_ERROR);

  gpio_set_direction(LED_1, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_2, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_3, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_WIFI, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_BT, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_ERROR, GPIO_MODE_OUTPUT);
}

void Led::set_led_n(uint8_t n, uint32_t state) {
  switch (n) {
  case 1:
    gpio_set_level(LED_1, state);
    break;
  case 2:
    gpio_set_level(LED_2, state);
    break;
  case 3:
    gpio_set_level(LED_3, state);
    break;
  default:
    gpio_set_level(LED_1, 0);
    gpio_set_level(LED_2, 0);
    gpio_set_level(LED_3, 0);
    break;
  }
}

void Led::wifi_enable() {
  gpio_set_level(LED_WIFI, 1);
  gpio_set_level(LED_BT, 0);
}

void Led::bt_enable() {
  gpio_set_level(LED_WIFI, 0);
  gpio_set_level(LED_BT, 1);
}

void Led::show_error() {
  while (1) {
    gpio_set_level(LED_ERROR, 1);
    DELAY(100);
    gpio_set_level(LED_ERROR, 0);
    DELAY(100);
  }
}