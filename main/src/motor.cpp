#include "motor.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "imu.hpp"
#include "pid.hpp"
#include <cmath>
#include <cstdint>
#include <numbers>

// static const char *TAG = __FILENAME__;
#define PI ((float)std::numbers::pi)

extern Imu imu;

// TODO: Move it inside class Robot
pcnt_unit_handle_t pcnt_unit_left;
pcnt_unit_handle_t pcnt_unit_right;
bool pid_enabled = true;

Robot::~Robot() {}
Robot::Robot() {
  pcnt_unit_left = NULL;
  pcnt_unit_right = NULL;
}

volatile int64_t period_left = 0;
void IRAM_ATTR encoder_isr_handler_left(void *arg) {
  static int64_t last_time_left = 0;
  int64_t current_time = esp_timer_get_time();
  period_left = current_time - last_time_left;
  last_time_left = current_time;

  int level_b = gpio_get_level(MOTOR_L_ENC_B);
  period_left = level_b ? -period_left : period_left;
}

volatile int64_t period_right = 0;
void IRAM_ATTR encoder_isr_handler_right(void *arg) {
  static int64_t last_time_right = 0;
  int64_t current_time = esp_timer_get_time();
  period_right = current_time - last_time_right;
  last_time_right = current_time;

  int level_b = gpio_get_level(MOTOR_R_ENC_B);
  period_right = level_b ? -period_right : period_right;
}

void Robot::init() {
  pwm_general_init(MOTOR_STBY, LEDC_TIMER_0);
  pwm_init(MOTOR_R_AIN1, MOTOR_R_AIN2, LEDC_TIMER_0, LEDC_CHANNEL_0,
           MOTOR_R_APWM);
  pwm_init(MOTOR_L_BIN1, MOTOR_L_BIN2, LEDC_TIMER_0, LEDC_CHANNEL_1,
           MOTOR_L_BPWM);
  encoder_init(&pcnt_unit_left, MOTOR_L_ENC_A, MOTOR_L_ENC_B,
               ENCODER_CNT_LIMIT);
  encoder_init(&pcnt_unit_right, MOTOR_R_ENC_A, MOTOR_R_ENC_B,
               ENCODER_CNT_LIMIT);
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
  encoder_int_init(encoder_isr_handler_left, MOTOR_L_ENC_A, MOTOR_L_ENC_B);
  encoder_int_init(encoder_isr_handler_right, MOTOR_R_ENC_A, MOTOR_R_ENC_B);

  cmds_queue = xQueueCreate(10, sizeof(Robot::cmd_t));
  xTaskCreate(&task_wrapper, "robot_task", 2048 * 10, this, 5, NULL);
}

// --- Init function ---

void Robot::encoder_init(pcnt_unit_handle_t *pcnt_unit, int gpio_enc_a,
                         int gpio_enc_b, int limit) {
  pcnt_channel_handle_t channel_a = NULL;
  pcnt_channel_handle_t channel_b = NULL;

  pcnt_unit_config_t unit_config = {};
  unit_config.high_limit = limit;
  unit_config.low_limit = -limit;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, pcnt_unit));

  pcnt_glitch_filter_config_t filter_config = {};
  filter_config.max_glitch_ns = 1000;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*pcnt_unit, &filter_config));

  pcnt_chan_config_t chan_conf_a = {};
  chan_conf_a.edge_gpio_num = gpio_enc_a;
  chan_conf_a.level_gpio_num = gpio_enc_b;
  ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_conf_a, &channel_a));
  pcnt_chan_config_t chan_conf_b = {};
  chan_conf_b.edge_gpio_num = gpio_enc_b;
  chan_conf_b.level_gpio_num = gpio_enc_a;
  ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_conf_b, &channel_b));

  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_unit_enable(*pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(*pcnt_unit));
}

void Robot::encoder_int_init(gpio_isr_t isr_handler, gpio_num_t gpio_enc_a,
                             gpio_num_t gpio_enc_b) {
  gpio_config_t io_conf_a = {};
  io_conf_a.intr_type = GPIO_INTR_POSEDGE;
  io_conf_a.mode = GPIO_MODE_INPUT;
  io_conf_a.pin_bit_mask = (1ULL << gpio_enc_a);
  io_conf_a.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf_a.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf_a));

  gpio_config_t io_conf_b = {};
  io_conf_b.intr_type = GPIO_INTR_DISABLE;
  io_conf_b.mode = GPIO_MODE_INPUT;
  io_conf_b.pin_bit_mask = (1ULL << gpio_enc_b);
  io_conf_b.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf_b.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf_b));
  ESP_ERROR_CHECK(gpio_isr_handler_add(gpio_enc_a, isr_handler, NULL));
}

void Robot::pwm_general_init(gpio_num_t stby_pin, ledc_timer_t timer) {
  gpio_reset_pin(stby_pin);
  gpio_set_level(stby_pin, 1);

  ledc_timer_config_t motor_timer = {};
  motor_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  motor_timer.timer_num = timer;
  motor_timer.duty_resolution = LEDC_TIMER_8_BIT;
  motor_timer.freq_hz = 5000;
  motor_timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));
}

void Robot::pwm_init(gpio_num_t in1_pin, gpio_num_t in2_pin, ledc_timer_t timer,
                     ledc_channel_t chan, gpio_num_t pwm_pin) {
  gpio_reset_pin(in1_pin);
  gpio_reset_pin(in2_pin);

  gpio_set_direction(in1_pin, GPIO_MODE_OUTPUT);
  gpio_set_direction(in2_pin, GPIO_MODE_OUTPUT);

  gpio_set_level(in1_pin, 0);
  gpio_set_level(in2_pin, 0);

  ledc_channel_config_t motor_chan = {};
  motor_chan.speed_mode = LEDC_LOW_SPEED_MODE;
  motor_chan.channel = chan;
  motor_chan.timer_sel = timer;
  motor_chan.intr_type = LEDC_INTR_DISABLE;
  motor_chan.gpio_num = pwm_pin;
  motor_chan.duty = 0;
  motor_chan.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&motor_chan));
}

// --- Function for left and right motor ---

void Robot::left_motor_set(int16_t left_motor_pwm, int16_t* left_pwm_field) {
  ledc_channel_t chanel = LEDC_CHANNEL_1;
  if (left_motor_pwm > 0) {
    gpio_set_level(MOTOR_L_BIN1, 0);
    gpio_set_level(MOTOR_L_BIN2, 1);
  } else if (left_motor_pwm == 0) {
    gpio_set_level(MOTOR_L_BIN1, 0);
    gpio_set_level(MOTOR_L_BIN2, 0);
  } else {
    gpio_set_level(MOTOR_L_BIN1, 1);
    gpio_set_level(MOTOR_L_BIN2, 0);
  }
  if (left_motor_pwm < 0)
    left_motor_pwm = 0 - left_motor_pwm;

  *left_pwm_field = left_motor_pwm;

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, chanel, left_motor_pwm));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, chanel));
}

void Robot::right_motor_set(int16_t right_motor_pwm, int16_t* right_pwm_field) {
  ledc_channel_t chanel = LEDC_CHANNEL_0;
  if (right_motor_pwm > 0) {
    gpio_set_level(MOTOR_R_AIN1, 1);
    gpio_set_level(MOTOR_R_AIN2, 0);
  } else if (right_motor_pwm == 0) {
    gpio_set_level(MOTOR_R_AIN1, 0);
    gpio_set_level(MOTOR_R_AIN2, 0);
  } else {
    gpio_set_level(MOTOR_R_AIN1, 0);
    gpio_set_level(MOTOR_R_AIN2, 1);
  }
  if (right_motor_pwm < 0)
    right_motor_pwm = 0 - right_motor_pwm;

  *right_pwm_field = right_motor_pwm;

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, chanel, right_motor_pwm));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, chanel));
}

// Encoder limits -> [0..ENCODER_CNT_LIMIT]
int Robot::get_delta_pulse(pcnt_unit_handle_t pcnt_unit, int &last_pulse) {
  int pulse = 0;

  pcnt_unit_get_count(pcnt_unit, &pulse);
  int delta = pulse - last_pulse;
  if (delta > ENCODER_CNT_LIMIT / 2) delta -= ENCODER_CNT_LIMIT;   //(deprecated) = -(ENCODER_CNT_LIMIT - pulse + last_pulse);
  if (delta < -ENCODER_CNT_LIMIT / 2) delta += ENCODER_CNT_LIMIT;  //(deprecated) = ENCODER_CNT_LIMIT - last_pulse + pulse;
  last_pulse = pulse;
  return delta;
}

int Robot::right_get_delta_pulse(int16_t* right_encoder_field) {
  static int last_pulse = 0;
  int pulse = get_delta_pulse(pcnt_unit_right, last_pulse);

  *right_encoder_field += pulse;

  return pulse;
}

int Robot::left_get_delta_pulse(int16_t* left_encoder_field) {
  static int last_pulse = 0;
  int pulse = get_delta_pulse(pcnt_unit_left, last_pulse);

  *left_encoder_field += pulse;

  return pulse;
}


TimerHandle_t motor_l_timer;
TimerHandle_t motor_r_timer;

void motor_l_timer_callback(TimerHandle_t xTimer) {
  Robot *pInstance = static_cast<Robot *>(pvTimerGetTimerID(xTimer));
  if (pInstance != nullptr)
    pInstance->left_motor_set(0);
}

void motor_r_timer_callback(TimerHandle_t xTimer) {
  Robot *pInstance = static_cast<Robot *>(pvTimerGetTimerID(xTimer));
  if (pInstance != nullptr)
    pInstance->right_motor_set(0);
}

void Robot::set_PWM(int left, int right, int left_time, int right_time){
  pid_enabled = false;
  left_motor_set(left);
  right_motor_set(right);
  if (left_time == 0) {
    if (motor_l_timer != NULL)
      xTimerStop(motor_l_timer, 0);
  } else {
    if (motor_l_timer != NULL)
      xTimerChangePeriod(motor_l_timer, pdMS_TO_TICKS(left_time), 0);
    else {
      motor_l_timer = xTimerCreate("motor_l_timer", pdMS_TO_TICKS(left_time),
                                   pdFALSE, this, motor_l_timer_callback);
      xTimerStart(motor_l_timer, 0);
    }
  }
  if (right_time == 0) {
    if (motor_r_timer != NULL)
      xTimerStop(motor_r_timer, 0);
  } else {
    if (motor_r_timer != NULL)
      xTimerChangePeriod(motor_r_timer, pdMS_TO_TICKS(right_time), 0);
    else {
      motor_r_timer = xTimerCreate("motor_r_timer", pdMS_TO_TICKS(right_time),
                                   pdFALSE, this, motor_r_timer_callback);
      xTimerStart(motor_r_timer, 0);
    }
  }
}

// --- Control task ---

void Robot::task() {
  Motor left_motor;
  Motor right_motor;

  left_motor.set_name("left_motor");
  right_motor.set_name("right_motor");

  left_motor.pwm_value = &left_pwm;
  left_motor.encoder_delta_sum_value = &left_encoder_delta_sum;
  left_motor.set_control_func(left_motor_set, left_get_delta_pulse);

  right_motor.pwm_value = &right_pwm;
  right_motor.encoder_delta_sum_value = &right_encoder_delta_sum;
  right_motor.set_control_func(right_motor_set, right_get_delta_pulse);

  left_motor.set_speed_pid({
      MOTOR_SPEED_PID_KP,
      MOTOR_SPEED_PID_KI,
      MOTOR_SPEED_PID_KD,
  });
  right_motor.set_speed_pid({
      MOTOR_SPEED_PID_KP,
      MOTOR_SPEED_PID_KI,
      MOTOR_SPEED_PID_KD,
  });
  left_motor.set_position_pid({
      MOTOR_POSITION_PID_KP,
      MOTOR_POSITION_PID_KI,
      MOTOR_POSITION_PID_KD,
  });
  right_motor.set_position_pid({
      MOTOR_POSITION_PID_KP,
      MOTOR_POSITION_PID_KI,
      MOTOR_POSITION_PID_KD,
  });

  left_motor.start();
  right_motor.start();

  while (true) {
    cmd_t cmd;
    if (xQueueReceive(cmds_queue, &cmd, 0) == pdTRUE) {
      if (cmd.speed > MOTOR_MAX_SPEED)
        cmd.speed = MOTOR_MAX_SPEED;
      if (cmd.speed < -MOTOR_MAX_SPEED)
        cmd.speed = -MOTOR_MAX_SPEED;
      float target_speed =
          (float)cmd.speed / (float)WHEEL_DIAMETER; // RPS = mm/s / mm
      float target_turns = (float)cmd.opt / ((float)WHEEL_DIAMETER * PI);

      switch (cmd.dir) {
      case forward:
        left_motor.set_target(target_speed, target_turns);
        right_motor.set_target(target_speed, target_turns);
        break;
      case backward:
        left_motor.set_target(-target_speed, -target_turns);
        right_motor.set_target(-target_speed, -target_turns);
        break;
      case left:
        target_turns = target_turns * PI * WHEEL_GAP / 360.0;
        left_motor.set_target(target_speed, target_turns);
        right_motor.set_target(-target_speed, -target_turns);
        break;
      case right:
        target_turns = target_turns * PI * WHEEL_GAP / 360.0;
        left_motor.set_target(-target_speed, -target_turns);
        right_motor.set_target(target_speed, target_turns);
        break;
      case breack:
        // ???
        break;
      default:;
      }
    }

    DELAY(100);
  }
}

// --- MOTOR -------------------------------------------------------------------

Motor::Motor() {}

Motor::~Motor() {}

// --- Setters ---
void Motor::set_name(const char *str) { strcpy(name_, str); }
void Motor::set_speed_pid(pid speed) { speed_ = speed; }
void Motor::set_position_pid(pid position) { position_ = position; }
void Motor::set_target(float speed, float position) {
  pid_enabled = true;
  target_speed_ = speed;
  target_position_ = global_position_ + position;
  change_target_flag_ = 1;
  motor_working_flag_ = 1;
}
void Motor::set_control_func(void (*set_pwm_func)(int16_t, int16_t*), int (*get_delta_pulse_func)(int16_t*)) {
  set_pwm_ = set_pwm_func;
  get_delta_pulse_ = get_delta_pulse_func;
}

void Motor::start() {
  xTaskCreate(&task_wrapper, name_, 2048 * 10, this, 5, NULL);
}

void Motor::task() {

  PID speed_pid(speed_.kp, speed_.ki, speed_.kd, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
  speed_pid.set_target(0.0);
  speed_pid.setIntegralLimits(-10.0f, 10.0f);

  PID position_pid(position_.kp, position_.ki, position_.kd, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
  position_pid.set_target(0.0);
  position_pid.setIntegralLimits(-100.0f, 100.0f);

  int64_t last_time = esp_timer_get_time();

  float alpha_speed = 0.2f;
  float filtered_speed = 0.0f;

  float alpha_pwm = 0.05f;
  float filtered_pwm = 0.0f;

  float speed_limit = 0.0;
  float alpha_global_position = 0.1f;
  float filtered_global_position = 0.0f;

  while (true) {

    if(!pid_enabled){
      int delta_pulse = get_delta_pulse_();
      global_position_ += ((float)delta_pulse / (float)MOTOR_RATIO);
      target_position_ = global_position_;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // --- Change target ---
    if (change_target_flag_) {

#ifdef SPEED_PID_DEBUG
      speed_pid.set_target(target_speed_);
#else
      speed_limit = target_speed_;
      position_pid.set_target(target_position_);
#endif // SPEED_PID_DEBUG

      change_target_flag_ = 0;
    }

    // --- Get delta pulse ---
    int delta_pulse = get_delta_pulse_(encoder_delta_sum_value);

    // --- Get delta time <S> ---
    uint64_t now = esp_timer_get_time();
    float dt = (now - last_time) / 1e6f;
    last_time = now;

    // *** POSITION CONTROL ***

    // --- Update position ---
    global_position_ += ((float)delta_pulse / (float)MOTOR_RATIO);

    // --- Global position filter ---
    filtered_global_position =
        alpha_global_position * global_position_ +
        (1.0 - alpha_global_position) * filtered_global_position;

    // --- PID (position -> speed) ---
    float out_speed = position_pid.compute(filtered_global_position, dt);
    if (abs(out_speed) > abs(speed_limit))
      out_speed = speed_limit;

    if (abs(abs(global_position_) - abs(target_position_)) < 1.0 / WHEEL_DIAMETER)
      motor_working_flag_ = 0;

      // --- Speed set ---
#ifndef SPEED_PID_DEBUG
    if (out_speed > 3.0 || out_speed < -3.0)
      speed_pid.set_target(out_speed);
    else
      speed_pid.set_target(0.0);
#endif // !SPEED_PID_DEBUG

    // *** SPEED CONTROL ***

    // --- Calculate speed <RPM> ---
    float tmp_speed = ((float)(delta_pulse) / dt) / (float)MOTOR_RATIO;
    if (tmp_speed > ((float)MOTOR_MAX_SPEED * 0.05) / (float)WHEEL_DIAMETER) {
      if (name_[0] == 'r') {
        tmp_speed = ((1.0e6f / (float)period_right) / MOTOR_RATIO_ISR);
      }
      if (name_[0] == 'l') {
        tmp_speed = ((1.0e6f / (float)period_left) / MOTOR_RATIO_ISR);
      }
    }

    // --- Speed filter ---
    filtered_speed =
        alpha_speed * tmp_speed + (1.0 - alpha_speed) * filtered_speed;

    // --- PID (speed -> PWM) ---
    float out_pwm = speed_pid.compute(filtered_speed, dt);

    // --- PWM filter ---
    filtered_pwm = alpha_pwm * out_pwm + (1.0 - alpha_pwm) * filtered_pwm;

    // --- Motor set ---
    if (abs(filtered_pwm) < (float)MOTOR_DEADBAND_PWM || !motor_working_flag_)
      filtered_pwm = 0;
    set_pwm_((int16_t)filtered_pwm, pwm_value);

    if (name_[0] == 'r') {

#ifdef SPEED_PID_DEBUG
      // --- speed info ---
      printf("$%f %f %f %f;\r\n", tmp_speed, filtered_speed, target_speed_,
             filtered_pwm);
#endif // SPEED_PID_DEBUG
#ifdef POSITION_PID_DEBUG
      // --- position info ---
      printf("$%f %f %f %f;\r\n", global_position_, filtered_global_position,
             target_position_, out_speed);
#endif // POSITION_PID_DEBUG
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
