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
    logger.log_error("Mocap subscriber cannot be initialized");
    std::exit(EXIT_FAILURE);
  }

  // Create cascaded pid controller
  controllers_2d::BasicPidCascaded controller;
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
  matrix::Vector<float, 4> motor_commands;
  matrix::Vector<float, 4> thrust_torque_cmd;

  bool session_end_flag = true;
  logger.log_info("Waiting for mocap datastream");

  int prev_index = 0;

  for (;;) {

    if (mocap_sub::new_data == true) {

      std::cout << "Received pose data:" << mocap_sub::index << '\n';

      if (mocap_sub::index == 1) {
        session_end_flag = false;
        logger.log_info("Starting session");
      }

      // Set flag to false after data has been received
      mocap_sub::new_data = false;

      // Cascaded controller
      thrust_torque_cmd = controller.cascaded_controller(
          mocap_sub::position, mocap_sub::orientation_euler, target.position());

      // Convert thrust, torque to motor speeds
      motor_commands = mixer.motor_mixer(thrust_torque_cmd);

      // Send motor commands to simulator
      msg.index({mocap_sub::index});
      msg.motor_commands({motor_commands(0), motor_commands(1),
                          motor_commands(2), motor_commands(3)});
      // // Publish motor command msg
      motor_command_pub.run(msg);

      // std::cout << "Published motor commands:" << mocap_sub::index << '\n';

      // std::cout << "Thrust command:" << thrust_torque_cmd(0) << '\n';
      // std::cout << "Torque commands:" << thrust_torque_cmd(1) << '\t'
      //           << thrust_torque_cmd(2) << '\t' << thrust_torque_cmd(3) <<
      //           '\n';
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
      // logger.add_to_log("thrust_command", &thrust_command);
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
