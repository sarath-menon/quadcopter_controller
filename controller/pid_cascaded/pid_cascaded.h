#pragma once
#include "math_helper.h"
#include "quad_2d.h"

class PidCascadedController {

private:
  YAML::Node controller_yaml =
      YAML::LoadFile("controller/pid_cascaded/controller_parameters.yaml");

protected:
  // Altitude controller parameters
  const float k_p__z = controller_yaml["k_p__z"].as<float>(); // [constant]
  const float k_i__z = controller_yaml["k_i__z"].as<float>(); // [constant]
  const float k_d__z = controller_yaml["k_d__z"].as<float>(); // [constant]
  // Vertical controller parameters
  const float k_p__x = controller_yaml["k_p__x"].as<float>(); // [constant]
  const float k_i__x = controller_yaml["k_i__x"].as<float>(); // [constant]
  const float k_d__x = controller_yaml["k_d__x"].as<float>(); // [constant]
  // Angle controller parameters
  const float k_p__b = controller_yaml["k_p__b"].as<float>(); // [constant]
  const float k_i__b = controller_yaml["k_i__b"].as<float>(); // [constant]
  const float k_d__b = controller_yaml["k_d__b"].as<float>(); // [constant]

  // Feedforward thrust
  constexpr static float ff_thrust = 9.81;

private:
  float altitude_pid(const float e, const float k_p, const float k_i,
                     const float k_d, const float dt);
  float horizontal_pid(const float e, const float k_p, const float k_i,
                       const float k_d, const float dt);
  float attitude_pid(const float e, const float k_p, const float k_i,
                     const float k_d, const float dt);

public:
  float altitude_controller(const Quad2D &quad, const float altitude_target,
                            const float dt);
  float horizontal_controller(const Quad2D &quad, const float horizontal_target,
                              const float dt);
  float attitude_controller(const Quad2D &quad, const float attitude_target,
                            const float dt);
};