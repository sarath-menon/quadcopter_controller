#include "pid_cascaded.h"

void PidCascadedController::set_gains(std::string path) {
  // Load yaml file containing gains
  YAML::Node controller_yaml = YAML::LoadFile(path);

  // x position controller gains
  k_p__x = controller_yaml["k_p__x"].as<float>(); // [constant]
  k_i__x = controller_yaml["k_i__x"].as<float>(); // [constant]
  k_d__x = controller_yaml["k_d__x"].as<float>(); // [constant]

  // // y position controller gains
  // k_p__y = controller_yaml["k_p__y"].as<float>(); // [constant]
  // k_i__y = controller_yaml["k_i__y"].as<float>(); // [constant]
  // k_d__y = controller_yaml["k_d__y"].as<float>(); // [constant]

  // z position controller gains
  k_p__z = controller_yaml["k_p__z"].as<float>(); // [constant]
  k_i__z = controller_yaml["k_i__z"].as<float>(); // [constant]
  k_d__z = controller_yaml["k_d__z"].as<float>(); // [constant]

  // roll angle controller gains
  k_p__roll = controller_yaml["k_p__roll"].as<float>(); // [constant]
  k_i__roll = controller_yaml["k_i__roll"].as<float>(); // [constant]
  k_d__roll = controller_yaml["k_d__roll"].as<float>(); // [constant]

  // pitch angle controller gains
  k_p__pitch = controller_yaml["k_p__pitch"].as<float>(); // [constant]
  k_i__pitch = controller_yaml["k_i__pitch"].as<float>(); // [constant]
  k_d__pitch = controller_yaml["k_d__pitch"].as<float>(); // [constant]

  // simulation timestep
  dt = controller_yaml["dt"].as<float>(); // [constant]
};

void PidCascadedController::set_quad_properties(std::string path) {
  // Load yaml file containing quadcopter properties
  YAML::Node quad_yaml = YAML::LoadFile(path);

  // x position controller gains
  thrust_max = quad_yaml["motor_thrust_max"].as<float>() * 4; // [constant]
  thrust_min = quad_yaml["motor_thrust_min"].as<float>() * 4; // [constant]
}
