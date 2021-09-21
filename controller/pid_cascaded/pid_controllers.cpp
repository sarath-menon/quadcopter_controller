#include "pid_cascaded.h"
#include <math.h>

float PidCascadedController::altitude_pid(const float e, const float k_p,
                                          const float k_i, const float k_d,
                                          const float dt) {

  // Integral error
  static float e_i = 0;
  // Derivative erroe
  static float e_d = 0;
  // e_prev -> Previous error
  static float e_prev = 0;

  // Update derivative and integral errors
  e_i += e;
  e_d = (e - e_prev) / dt;

  float control_output = e * k_p + e_i * k_i + e_d * k_d;

  // Set current values as prev values
  e_prev = e;

  return control_output;
}

float PidCascadedController::horizontal_pid(const float e, const float k_p,
                                            const float k_i, const float k_d,
                                            const float dt) {

  // Integral error
  static float e_i = 0;
  // Derivative erroe
  static float e_d = 0;
  // e_prev -> Previous error
  static float e_prev = 0;

  // Update derivative and integral errors
  e_i += e;
  e_d = (e - e_prev) / dt;

  float control_output = e * k_p + e_i * k_i + e_d * k_d;

  // Set current values as prev values
  e_prev = e;

  return control_output;
}

float PidCascadedController::attitude_pid(const float e, const float k_p,
                                          const float k_i, const float k_d,
                                          const float dt) {

  // Integral error
  static float e_i = 0;

  // Derivative erroe
  static float e_d = 0;
  // e_prev -> Previous error
  static float e_prev = 0;

  // Update derivative and integral errors
  e_i += e;
  e_d = (e - e_prev) / dt;

  float control_output = e * k_p + e_i * k_i + e_d * k_d;

  // Set current values as prev values
  e_prev = e;

  return control_output;
}
