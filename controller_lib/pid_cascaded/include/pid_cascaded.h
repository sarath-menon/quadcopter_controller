#pragma once
#include "math_helper.h"
#include "pid.h"
#include <string>
#include <yaml-cpp/yaml.h>

class PidCascadedController {

private:
protected:
  // x position controller gains
  float k_p__x = 0; // [constant]
  float k_i__x = 0; // [constant]
  float k_d__x = 0; // [constant]
  // y position controller gains
  float k_p__y = 0; // [constant]
  float k_i__y = 0; // [constant]
  float k_d__y = 0; // [constant]
  // z position controller gains
  float k_p__z = 0; // [constant]
  float k_i__z = 0; // [constant]
  float k_d__z = 0; // [constant]
  // roll angle controller parameters
  float k_p__roll = 0; // [constant]
  float k_i__roll = 0; // [constant]
  float k_d__roll = 0; // [constant]
  // roll angle controller parameters
  float k_p__pitch = 0; // [constant]
  float k_i__pitch = 0; // [constant]
  float k_d__pitch = 0; // [constant]

  // Feedforward thrust
  constexpr static float ff_thrust = 9.81;

public:
  // Positon controllers
  float x_position_controller(const float x_position_target,
                              const float x_position_now,
                              const float roll_angle_max,
                              const float roll_angle_min, const float dt);
  // float y_position_controller(const float y_position_target,
  //                             const float y_position_now,
  //                             const float pitch_angle_max,
  //                             const float pitch_angle_min, const float dt);
  float z_position_controller(const float z_position_target,
                              const float z_position_now,
                              const float thrust_max, const float thrust_min,
                              const float dt);

  // Attitude controller
  float roll_angle_controller(const float roll_angle_target,
                              const float roll_angle_now,
                              const float roll_torque_max,
                              const float roll_torque_min, const float dt);
  // float pitch_angle_controller(const float pitch_angle_target,
  //                              const float pitch_angle_now,
  //                              const float pitch_torque_max,
  //                              const float pitch_torque_min, const float dt);
  // float yaw_angle_controller(const float yaw_angle_target,
  //                            const float yaw_angle_now,
  //                            const float yaw_torque_max, const float
  //                            yaw_torque_min, const float dt);

public:
  // To load gain vaules from yaml file
  void set_gains(std::string path);
};