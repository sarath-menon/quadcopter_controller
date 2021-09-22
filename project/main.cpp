// #include "pid.h"
#include "mocap_subscriber_callback.h"
#include "motor_mixing.h"
#include "pid_cascaded.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <math_helper.h>

// Fastdds Headers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Mocap data subscriber
#include "mocap_quadcopterPubSubTypes.h"
#include "mocap_quadcopterSubscriber.h"
// Motor commands publisher
#include "motor_commandsPubSubTypes.h"
#include "motor_commandsPublisher.h"
/////////////////////////////////////////////////////////////////////////////////

// Px4 math header
#include "matrix/math.hpp"

int main() {

  // Initialize motor commands subscriber
  motor_commandsPublisher motor_command_pub;
  bool fastdds_flag = false;

  // Initialize mocap data subscriber
  subscriber::mocap_quadcopterSubscriber mysub;
  mysub.init();

  // Initialize cascaded pid controller
  PidCascadedController controller;

  // Initialize for now
  constexpr static float z_position_target = 5;
  constexpr static float thrust_max = 25;
  constexpr static float thrust_min = 7;
  constexpr static float dt = 0.01;
  const float k_f = 6.11 * exp(-8);
  const float arm_length = 0.171;
  float motor_commands[4] = {0, 0, 0, 0};

  for (;;) {

    if (subscriber::new_data == true) {

      // Outer loop
      const float thrust_command = controller.z_position_controller(
          z_position_target, subscriber::position[2], thrust_max, thrust_min,
          dt);

      // const float attitude_command =
      //     controller.horizontal_controller(quad, horizontal_target, dt);

      // // Inner loop
      // const float torque_command =
      //     controller.attitude_controller(quad, attitude_command, dt);

      const float torque_command = 0;

      // Convert thrust, torque to motor speeds
      motor_mixing(motor_commands, thrust_command, torque_command, k_f,
                   arm_length);

      // Set flag to false after data has been processed
      subscriber::new_data = false;

      std::cout << "Index:" << subscriber::index << '\n';
      std::cout << "Thrust command:" << thrust_command << '\n';
      std::cout << "position:" << subscriber::position[2] << '\n';
    }
  }
}
