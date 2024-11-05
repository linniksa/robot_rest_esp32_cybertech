#pragma once
#include "stdint.h"

class Led {
public:
  Led();
  ~Led();
  void set_led_n(uint8_t n, uint32_t state);
  void wifi_enable();
  void bt_enable();
  void show_error();
  void init();

private:
};