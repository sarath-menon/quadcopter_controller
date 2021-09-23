#include "include_helper.h"

int main() {

  // Initialize motor command publisher
  motor_commandsPublisher motor_command_pub;
  motor_command_pub.init();
  motor_comands msg;
  bool fastdds_flag = false;

  // Initialize mocap data subscriber
  subscriber::mocap_quadcopterSubscriber mysub;
  mysub.init();

  // Initialize cascaded pid controller
  PidCascadedController controller;
  controller.set_gains(
      "cascaded_controller_app/parameters/controller_parameters.yaml");

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

      msg.motor_command({motor_commands[0], motor_commands[1],
                         motor_commands[2], motor_commands[3]});
      // // Publish motor command msg
      motor_command_pub.run(msg);

      // Set flag to false after data has been processed
      subscriber::new_data = false;

      std::cout << "Index:" << subscriber::index << '\n';
      std::cout << "Thrust command:" << thrust_command << '\n';
      std::cout << "Position:" << subscriber::position[0] << '\t'
                << subscriber::position[1] << '\t' << subscriber::position[2]
                << '\n';
      std::cout << "Motor commands:" << motor_commands[0] << '\t'
                << motor_commands[1] << '\t' << motor_commands[2] << '\t'
                << motor_commands[3] << '\t' << std::endl;

      std::cout << std::endl;
    }
  }
}
