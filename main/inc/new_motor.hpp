#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class motor {
public:
    motor(void (*init_func)(), void (*set_pwm_func)(int), int (*get_pulse_func)());
    ~motor();

    void start();

private:
    static void pid_task(void *arg);

    void (*init_func_)();
    void (*set_pwm_func_)(int);
    int (*get_pulse_func_)();

    TaskHandle_t pid_task_handle_;
};

motor::motor(void (*init_func)(), void (*set_pwm_func)(int), int (*get_pulse_func)())
    : init_func_(init_func),
      set_pwm_func_(set_pwm_func),
      get_pulse_func_(get_pulse_func),
      pid_task_handle_(nullptr)
{
    if (init_func_) {
        init_func_();
    }
}

motor::~motor()
{
    if (pid_task_handle_ != nullptr) {
        vTaskDelete(pid_task_handle_);
    }
}

void motor::start()
{
    xTaskCreate(pid_task, "pid_task", 2048, this, 5, &pid_task_handle_);
}

void motor::pid_task(void *arg)
{
    motor *self = static_cast<motor *>(arg);

    while (true) {
        int pulse = self->get_pulse_func_();

        // PID controller calculations (placeholder)
        int pwm_value = /* PID calculation using 'pulse' */;

        self->set_pwm_func_(pwm_value);

        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust the delay as needed
    }
}