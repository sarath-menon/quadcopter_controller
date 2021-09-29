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

      // Convert quaternion to euler angle
      matrix::Quatf q_nb(subscriber::orientation[1], subscriber::orientation[2],
                         subscriber::orientation[3],
                         subscriber::orientation[0]);

      matrix::Eulerf euler_orientation(q_nb);

      std::cout << "Orientation:" << euler_orientation(0)
                << euler_orientation(1) << euler_orientation(2);

      // Outer loop
      const float thrust_command = controller.z_position_controller(
          target.z_position(), subscriber::position[2]);

      const float attitude_command = controller.x_position_controller(
          target.x_position(), subscriber::position[0]);

      // Inner loop
      const float torque_command = controller.roll_angle_controller(
          attitude_command, euler_orientation(0));

      // Convert thrust, torque to motor speeds
      mixer.motor_mixer(motor_commands, thrust_command, torque_command);
      // motor_mixing(motor_commands, thrust_command, torque_command, k_f,
      //              arm_length);

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
