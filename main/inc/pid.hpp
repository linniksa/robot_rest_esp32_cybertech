#pragma once
#include <cmath>

class PID {
private:
  float kp;
  float ki;
  float kd;

  float previous_error;
  float integral;
  float setpoint;
  float output_min;
  float output_max;

  float integral_min;
  float integral_max;

public:
  PID(float kp, float ki, float kd, float output_min, float output_max)
      : kp(kp), ki(ki), kd(kd), previous_error(0.0f), integral(0.0f),
        setpoint(0.0f), output_min(output_min), output_max(output_max),
        integral_min(-INFINITY), integral_max(INFINITY) {}

  void set_target(float sp) { setpoint = sp; }
  float get_target() { return setpoint; }

  void setIntegralLimits(float min, float max) {
    integral_min = min;
    integral_max = max;
  }

  float compute(float input, float dt) {
    float error = setpoint - input;

    integral += error * dt;
    if (integral > integral_max)
      integral = integral_max;
    else if (integral < integral_min)
      integral = integral_min;

    float derivative = (error - previous_error) / dt;
    float output = kp * error + ki * integral + kd * derivative;

    if (output > output_max)
      output = output_max;
    else if (output < output_min)
      output = output_min;

    previous_error = error;
    return output;
  }

  void reset() {
    integral = 0.0f;
    previous_error = 0.0f;
  }
};