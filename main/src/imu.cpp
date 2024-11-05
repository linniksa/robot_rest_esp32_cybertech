#include "imu.hpp"
#include "bno055.hpp"
#include "config.hpp"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <stdio.h>

static const char *TAG = __FILENAME__;

Imu::Imu() {}
Imu::~Imu() {}

i2c_master_dev_handle_t i2c_bno055_handle;

float Imu::get_yaw() { return yaw; }
float Imu::get_pitch() { return pitch; }
float Imu::get_roll() { return roll; }

void Imu::init() {
  i2c_master_bus_config_t i2c_bus_config = {};
  i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_bus_config.i2c_port = I2C_NUM_1;
  i2c_bus_config.scl_io_num = IMU_SCL;
  i2c_bus_config.sda_io_num = IMU_SDA;
  i2c_bus_config.glitch_ignore_cnt = 100;
  i2c_bus_config.flags.enable_internal_pullup = 1;
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

#ifdef I2C_SCAN_ENABLE
  ESP_LOGI(TAG, "I2C scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2c_master_probe(bus_handle, addr, 100) == ESP_OK)
      ESP_LOGI(TAG, " ---> find device: 0x%02X <---", addr);
  }
#endif // I2C_SCAN_ENABLE

  i2c_device_config_t i2c_bno055_config = {};
  i2c_bno055_config.device_address = BNO055_ADDR;
  i2c_bno055_config.scl_speed_hz = 100000;
  i2c_bno055_config.scl_wait_us = 5 * 1000;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_bno055_config,
                                            &i2c_bno055_handle));

  xTaskCreate(&task_wrapper, "bno055", 1024 * 4, this, 10, NULL);
}

uint8_t buff[200] = {};
static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
  u8 iError = BNO055_INIT_VALUE;
  buff[0] = reg_addr;
  for (size_t i = 0; i < cnt; i++)
    buff[i + 1] = reg_data[i];

  if (i2c_master_transmit(i2c_bno055_handle, buff, cnt + 1, 10000) == ESP_OK)
    iError = BNO055_SUCCESS;
  else
    iError = BNO055_ERROR;

  return (s8)iError;
}

static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
  u8 iError = BNO055_INIT_VALUE;

  i2c_master_transmit(i2c_bno055_handle, (uint8_t *)&reg_addr, 1, 10000);
  if (i2c_master_receive(i2c_bno055_handle, reg_data, cnt, 10000) == ESP_OK)
    iError = BNO055_SUCCESS;
  else
    iError = BNO055_ERROR;
  return (s8)iError;
}

static void BNO055_delay_msek(u32 msek) {
  vTaskDelay(msek / portTICK_PERIOD_MS);
}

void Imu::task() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  struct bno055_t myBNO = {};
  myBNO.bus_write = BNO055_I2C_bus_write;
  myBNO.bus_read = BNO055_I2C_bus_read;
  myBNO.dev_addr = BNO055_ADDR;
  myBNO.delay_msec = BNO055_delay_msek;
  if (bno055_init(&myBNO) != 0)
    ESP_LOGE(TAG, "BNO055 not found.");

  bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  u8 bno055_id = 0;
  bno055_read_chip_id(&bno055_id);
  ESP_LOGI(TAG, "Read BNO055 ID:0x%X, right:.0xA0", bno055_id);

  struct bno055_euler_float_t eulerData;
  bno055_convert_float_euler_hpr_deg(&eulerData);

  unsigned char accel_calib_status = 0;
  unsigned char gyro_calib_status = 0;
  unsigned char mag_calib_status = 0;
  unsigned char sys_calib_status = 0;

  bno055_get_accel_calib_stat(&accel_calib_status);
  bno055_get_mag_calib_stat(&mag_calib_status);
  bno055_get_gyro_calib_stat(&gyro_calib_status);
  bno055_get_sys_calib_stat(&sys_calib_status);

  s16 s16_yaw;
  s16 s16_pitch;
  s16 s16_roll;

  while (1) {
    bno055_read_euler_h(&s16_yaw);
    bno055_read_euler_p(&s16_pitch);
    bno055_read_euler_r(&s16_roll);

    yaw = (float)s16_yaw / 16.0;
    pitch = (float)s16_pitch / 16.0;
    roll = (float)s16_roll / 16.0;

    // ESP_LOGI(TAG, "IMU: yaw: %d pitch: %d roll: %d.",
    //          global_yaw, global_pitch, global_roll);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}