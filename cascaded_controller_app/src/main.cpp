#include "include_helper.h"

int main() {

  // Initialize logger
  Logger logger(paths::event_log_path, paths::data_log_path);

  // Initialize motor command publisher
  motor_commandsPublisher motor_command_pub;
  actuator_commands msg;

  // New fastdds
  // Message
  msgs::QuadMotorCommand motor_cmd;
  // Create publisher with msg type
  DDSPublisher motor_cmd_pub(QuadMotorCommandPubSubType(), "motor_commands");

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

  bool session_end_flag = true;
  logger.log_info("Waiting for mocap datastream");

  // Global variables for now
  matrix::Vector<float, 4> motor_commands;
  matrix::Vector<float, 4> thrust_torque_cmd;

  for (;;) {

    // Lock until read and write are completed

    { // Wait until subscriber sends mocap pose
      std::unique_lock<std::mutex> lk(mocap_sub::m);
      mocap_sub::cv.wait(lk, [] { return mocap_sub::new_data; });

      // Reset flag and prroceed when mocap data available
      mocap_sub::new_data = false;

      // Run controller
      thrust_torque_cmd = controller.cascaded_controller(
          mocap_sub::position, mocap_sub::orientation_euler, target.position());
    }
    // Convert thrust, torque to motor speeds
    motor_commands = mixer.motor_mixer(thrust_torque_cmd);

    // Publish motor commands
    msg.index(mocap_sub::index);
    msg.motor_commands({motor_commands(0), motor_commands(1), motor_commands(2),
                        motor_commands(3)});
    motor_command_pub.run(msg);

    // New fastdds -> publish motor commands
    float waste[4];
    motor_cmd.header.id = "srl_quad";
    motor_cmd.header.timestamp = mocap_sub::index;
    motor_cmd.motorspeed[0] = motor_commands(0);
    // motor_cmd.speed[1] = motor_commands(1);
    // motor_cmd.speed[2] = motor_commands(2);
    // motor_cmd.speed[3] = motor_commands(3);

    // if (session_end_flag == false && mocap_sub::matched == 0) {

    //   session_end_flag = true;
    //   logger.shutdown_data_logger();
    //   logger.log_info("Mocap datastreastream has closed. Data log saved");
    //   logger.log_info("Waiting for new mocap datastream");
    // }
  }
}
