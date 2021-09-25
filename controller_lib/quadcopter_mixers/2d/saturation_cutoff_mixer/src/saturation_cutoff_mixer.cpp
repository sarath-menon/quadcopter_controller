#include "saturation_cutoff_mixer.h"
#include <iostream>

void QuadcopterMixer::motor_mixer(float motor_commands[4],
                                  const float thrust_command,
                                  const float torque_command) {

  float f2 = (thrust_command / 4) + (torque_command / (2 * arm_length));
  float f4 = (thrust_command / 4) - (torque_command / (2 * arm_length));

  f2 = limit(f2, motor_thrust_max, motor_thrust_min);
  f4 = limit(f4, motor_thrust_max, motor_thrust_min);

  // In plane motors
  motor_commands[1] = sqrt(f2 / k_f);
  motor_commands[3] = sqrt(f4 / k_f);

  // Our of plane motor speeds set to averge of in plane
  float f1 = thrust_command / 4;
  float f3 = f1;

  motor_commands[0] = sqrt(f1 / k_f);
  motor_commands[2] = sqrt(f3 / k_f);

  // std::cout << "Actual Motor 1 command " << motor_commands[0] << std::endl;

  //   std::cout << "Controller: f1:" << f1 << "\tf2:" << f2 << "\tf3:" << f3
  //             << "\tf4:" << f4 << std::endl;
  // std::cout << "Net thrust and torque before  motor mixing:" <<
  // thrust_command
  //           << '\t' << torque_command << std::endl;
  // std::cout << "Net thrust and torque after motor mixing:" << f1 + f2 + f3 +
  // f4
  //           << '\t' << (f2 - f4) * arm_length << std::endl;
}
