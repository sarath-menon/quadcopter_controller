#pragma once

constexpr static float pid(const float e, const float k_p, const float k_i,
                           const float k_d, const float dt, float &e_i,
                           float &e_d, float &e_prev) {

  // Update derivative and integral errors
  e_i += e;
  e_d = (e - e_prev) / dt;

  float control_output = e * k_p + e_i * k_i + e_d * k_d;

  // Set current values as prev values
  e_prev = e;

  return control_output;
};