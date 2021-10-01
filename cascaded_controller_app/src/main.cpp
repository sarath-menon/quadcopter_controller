#include "include_helper.h"

int main() {

  // Initialize logger
  Logger logger(paths::event_log_path, paths::data_log_path);

  // Initialize motor command publisher
  motor_commandsPublisher motor_command_pub;
  actuator_commands msg;

  try {
    if (motor_command_pub.init() == true)
      logger.log_info("Initialized Motor command subscriber");
    else
      throw(motor_command_pub);
  } catch (motor_commandsPublisher motor_command_pub) {
    logger.log_error("Motor command publisher cannot be initialized");
    std::exit(EXIT_FAILURE);
  }

  // Initialize mocap data subscriber
  mocap_quadcopterSubscriber mocap_sub;

  try {
    if (mocap_sub.init() == true)
      logger.log_info("Initialized Mocap subscriber");
    else
      throw(motor_command_pub);
  } catch (motor_commandsPublisher motor_command_pub) {
    logger.log_error("Initialized Mocap subscriber cannot be");
    std::exit(EXIT_FAILURE);
  }

  // Create cascaded pid controller
  PidCascadedController controller;
  controller.set_gains(paths::controller_gains_yaml);
  controller.set_quad_properties(paths::quad_yaml);
  logger.log_info("Initialized Controller");

  // Create quadcopter mixer
  QuadcopterMixer mixer;
  // Set quadcopter parameters in mixer
  mixer.set_quad_properties(paths::quad_yaml);
  logger.log_info("Initialized Mixer");

  // Create waypointsetter
  WaypointSetter target;
  target.set_setpoints(paths::setpoint_yaml);
  logger.log_info("Initialized Waypoint setter");

  // Initialize for now
  float motor_commands[4] = {0, 0, 0, 0};

  bool session_end_flag = true;
  logger.log_info("Waiting for mocap datastream");

  for (;;) {
    // for (int i = 0; i < 10; i++) {

    if (mocap_sub::new_data == true) {

      std::cout << "Received pose data:" << mocap_sub::index << '\n';

      if (mocap_sub::index == 1) {
        session_end_flag = false;
        logger.log_info("Starting session");
      }

      // Set flag to false after data has been received
      mocap_sub::new_data = false;

      // Outer loop
      const float thrust_command = controller.z_position_controller(
          target.z_position(), mocap_sub::position[2]);

      const float attitude_command = controller.x_position_controller(
          target.x_position(), mocap_sub::position[0]);

      // const float attitude_command = 0.5;

      // Inner loop
      const float torque_command = controller.roll_angle_controller(
          attitude_command, -mocap_sub::orientation_euler[1]);

      // Convert thrust, torque to motor speeds
      mixer.motor_mixer(motor_commands, thrust_command, torque_command);

      // Send motor commands to simulator
      msg.index({mocap_sub::index});
      msg.motor_commands({motor_commands[0], motor_commands[1],
                          motor_commands[2], motor_commands[3]});
      // // Publish motor command msg
      motor_command_pub.run(msg);

      // std::cout << "Published motor commands:" << mocap_sub::index << '\n';

      // std::cout << "Thrust command:" << thrust_command << '\n';
      // std::cout << "Torque command:" << torque_command << '\n';
      // std::cout << "Position:" << mocap_sub::position[0] << '\t'
      //           << mocap_sub::position[1] << '\t' << mocap_sub::position[2]
      //           << '\n';
      // std::cout << "Orientation euler in degrees:"
      //           << mocap_sub::orientation_euler[0] << '\t'
      //           << -mocap_sub::orientation_euler[1] << '\t'
      //           << mocap_sub::orientation_euler[2] << '\n';
      // std::cout << "Attitude command:" << attitude_command << '\n' << '\n';

      // logger.log_data(mocap_sub::position[0]);
      // logger.log_data(mocap_sub::position[1]);
      // logger.log_data(mocap_sub::position[2]);
      // logger.log_data(mocap_sub::orientation_euler[0]);
      // logger.log_data(mocap_sub::orientation_euler[1]);
      // logger.log_data(mocap_sub::orientation_euler[2]);
      // logger.log_data(thrust_command);
      // logger.log_data(0);
      // logger.log_data(torque_command);
      // logger.log_data(0);
      // logger.log_data(motor_commands[0]);
      // logger.log_data(motor_commands[1]);
      // logger.log_data(motor_commands[2]);
      // logger.log_data(motor_commands[3]);

      // logger.log_next_line();
      logger.add_to_log("thrust_command", &thrust_command);
      logger.log_now();

      // std::cout << "Motor commands:" << motor_commands[0] << '\t'
      //           << motor_commands[1] << '\t' << motor_commands[2] << '\t'
      //           << motor_commands[3] << '\t' << std::endl;

      // std::cout << std::endl;
    }

    else if (session_end_flag == false && mocap_sub::matched == 0) {

      session_end_flag = true;
      logger.shutdown_data_logger();
      logger.log_info("Mocap datastreastream has closed. Data log saved");
      logger.log_info("Waiting for new mocap datastream");
    }

    else if (session_end_flag == true && mocap_sub::matched == 0) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::cout << ".";
    }
  }
}
