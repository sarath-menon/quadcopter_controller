#pragma once

// Standard form PID controller
float pid(const float e, const float k_p, const float k_i, const float k_d,
          const float dt, float &e_i, float &e_d, float &e_prev);