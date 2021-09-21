#include "pid_cascaded.h"

float PidCascadedController::x_position_controller(
    const float x_position_target, const float x_position_now,
    const float roll_angle_max, const float roll_angle_min, const float dt) {

  // Compute error
  const float error = x_position_target - x_position_now;

  // Compute control input
  float roll_angle_command =
      pid(error, k_p__x, k_i__x, k_d__x, dt, e_i__x, e_d__x, e_prev__x);

  // Limit roll angle to near zero to respect linearization
  roll_angle_command =
      limit(roll_angle_command, roll_angle_max, roll_angle_min);

  return roll_angle_command;
};

float PidCascadedController::z_position_controller(
    const float z_position_target, const float z_position_now,
    const float thrust_max, const float thrust_min, const float dt) {

  // Compute error
  const float error = z_position_target - z_position_now;

  // Compute control input
  float thrust_command =
      pid(error, k_p__z, k_i__z, k_d__z, dt, e_i__z, e_d__z, e_prev__z);

  // Limit roll angle to near zero to respect linearization
  thrust_command = limit(ff_thrust + thrust_command, thrust_max, thrust_min);

  return thrust_command;
};

float PidCascadedController::roll_angle_controller(
    const float roll_angle_target, const float roll_angle_now,
    const float roll_torque_max, const float roll_torque_min, const float dt) {

  // Compute error
  const float error = roll_angle_target - roll_angle_now;

  // Compute control input
  float roll_torque_command = pid(error, k_p__roll, k_i__roll, k_d__roll, dt,
                                  e_i__z, e_d__z, e_prev__z);

  // Limit roll angle to near zero to respect linearization
  roll_torque_command =
      limit(roll_torque_command, roll_torque_max, roll_torque_min);

  return roll_torque_command;
};