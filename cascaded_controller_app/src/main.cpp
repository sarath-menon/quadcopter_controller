#include "include_helper.h"

int main() {

  // Initialize motor command publisher
  motor_commandsPublisher motor_command_pub;
  motor_command_pub.init();
  actuator_commands msg;
  bool fastdds_flag = false;

  // Initialize mocap data subscriber
  subscriber::mocap_quadcopterSubscriber mysub;
  mysub.init();

  // Create cascaded pid controller
  PidCascadedController controller;

  controller.set_gains(controller_gains_yaml);
  controller.set_quad_properties(quad_yaml);

  // Create quadcopter mixer
  QuadcopterMixer mixer;
  // Set quadcopter parameters in mixer
  mixer.set_quad_properties(quad_yaml);

  // Create waypointsetter
  WaypointSetter target;
  target.set_setpoints(setpoint_yaml);

  // Initialize for now
  float motor_commands[4] = {0, 0, 0, 0};

  for (;;) {

    if (subscriber::new_data == true) {

      // Outer loop
      const float thrust_command = controller.z_position_controller(
          target.z_position(), subscriber::position[2]);

      const float attitude_command = controller.x_position_controller(
          target.x_position(), subscriber::position[0]);

      // const float attitude_command = 0.5;

      // Inner loop
      const float torque_command = controller.roll_angle_controller(
          attitude_command, -subscriber::orientation_euler[1]);

      // Convert thrust, torque to motor speeds
      mixer.motor_mixer(motor_commands, thrust_command, torque_command);

      msg.motor_commands({motor_commands[0], motor_commands[1],
                          motor_commands[2], motor_commands[3]});
      // // Publish motor command msg
      motor_command_pub.run(msg);

      // Set flag to false after data has been processed
      subscriber::new_data = false;

      std::cout << "Index:" << subscriber::index << '\n';
      std::cout << "Thrust command:" << thrust_command << '\n';
      std::cout << "Torque command:" << torque_command << '\n';
      std::cout << "Position:" << subscriber::position[0] << '\t'
                << subscriber::position[1] << '\t' << subscriber::position[2]
                << '\n';
      std::cout << "Orientation euler in degrees:"
                << subscriber::orientation_euler[0] << '\t'
                << -subscriber::orientation_euler[1] << '\t'
                << subscriber::orientation_euler[2] << '\n';
      std::cout << "Attitude command:" << attitude_command << '\n' << '\n';
      // std::cout << "Motor commands:" << motor_commands[0] << '\t'
      //           << motor_commands[1] << '\t' << motor_commands[2] << '\t'
      //           << motor_commands[3] << '\t' << std::endl;

      // std::cout << std::endl;
    }
  }
}
