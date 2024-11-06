#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <cstring>

class Robot {
private:
  static void task_wrapper(void *pvParameters) {
    Robot *pThis = static_cast<Robot *>(pvParameters);
    pThis->task();
  }
  void task();
  void pwm_init(gpio_num_t in1_pin, gpio_num_t in2_pin, ledc_timer_t timer,
                ledc_channel_t chan, gpio_num_t pwm_pin);
  void pwm_general_init(gpio_num_t stby_pin, ledc_timer_t timer);
  void encoder_init(pcnt_unit_handle_t *pcnt_unit, int gpio_enc_a,
                    int gpio_enc_b, int limit);
  void encoder_int_init(gpio_isr_t isr_handler, gpio_num_t gpio_enc_a,
                        gpio_num_t gpio_enc_b);

  static int get_delta_pulse(pcnt_unit_handle_t pcnt_unit, int &last_pulse);
  static int right_get_delta_pulse();
  static int left_get_delta_pulse();

public:
  enum move_directions { forward, backward, left, right, breack, invalid };

  // enum { free, busy } state; // ????
  typedef struct {
    move_directions dir;
    int32_t opt;
    int32_t speed;
  } cmd_t;
  QueueHandle_t cmds_queue;

  int16_t left_pwm = 0;
  int16_t right_pwm = 0;
  uint8_t break_flag = 0;

  Robot();
  ~Robot();
  void init();
  void set_PWM(int left, int right, int left_time, int right_time);
  static void right_motor_set(int16_t right_motor_pwm);
  static void left_motor_set(int16_t left_motor_pwm);
  void send_cmd_to_queue(cmd_t cmd) {
    xQueueSend(cmds_queue, (void *)&cmd, (TickType_t)0);
  }
};

class Motor {
public:
  Motor();
  Motor(Motor &&) = default;
  Motor(const Motor &) = default;
  Motor &operator=(Motor &&) = default;
  Motor &operator=(const Motor &) = default;
  ~Motor();

  struct pid {
    float kp = 0.0;
    float ki = 0.0;
    float kd = 0.0;
  };

  void set_control_func(void (*set_pwm_func)(int16_t), int (*get_pulse_func)());
  void set_speed_pid(pid speed);
  void set_position_pid(pid position);
  void set_target(float speed, float position);
  void set_name(const char *str);
  void start();

private:
  pid speed_;
  pid position_;
  char name_[100];

  uint8_t change_target_flag_ = 0;
  uint8_t motor_working_flag_ = 0;
  float target_speed_ = 0.0;    // RPS
  float target_position_ = 0.0; // Turns
  float global_position_ = 0.0; // Turns

  static void task_wrapper(void *pvParameters) {
    Motor *pThis = static_cast<Motor *>(pvParameters);
    pThis->task();
  }
  void task();

  void (*set_pwm_)(int16_t);
  int (*get_delta_pulse_)();
};