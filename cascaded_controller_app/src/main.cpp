#include "geometry_msgs/Pose.h"
#include "include_helper.h"

// Global variables for now
matrix::Vector<float, 4> motor_commands;

void controller_thread(controllers_2d::BasicPidCascaded &controller,
                       QuadcopterMixer &mixer, WaypointSetter &target) {

  // Wait until subscriber sends mocap pose
  std::unique_lock<std::mutex> lk(mocap_sub::m);
  mocap_sub::cv.wait(lk, [] { return mocap_sub::new_data; });

  // Proceed when mocap data availablw
  // First, set new data flag to false
  mocap_sub::new_data = false;

  // RUn controller
  matrix::Vector<float, 4> thrust_torque_cmd = controller.cascaded_controller(
      mocap_sub::position, mocap_sub::orientation_euler, target.position());

  // Convert thrust, torque to motor speeds
  motor_commands = mixer.motor_mixer(thrust_torque_cmd);
}

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

  bool session_end_flag = true;
  logger.log_info("Waiting for mocap datastream");

  // msgs::Pose pose_msg;
  // pose_msg.header.name = "selva";

  for (;;) {

    // Start controller thread
    std::thread ctrl(std::ref(controller_thread), std::ref(controller),
                     std::ref(mixer), std::ref(target));

    // Wait for controller thread to finish
    ctrl.join();

    // Publish motor commands
    msg.index({mocap_sub::index});
    msg.motor_commands({motor_commands(0), motor_commands(1), motor_commands(2),
                        motor_commands(3)});
    motor_command_pub.run(msg);

    // if (session_end_flag == false && mocap_sub::matched == 0) {

    //   session_end_flag = true;
    //   logger.shutdown_data_logger();
    //   logger.log_info("Mocap datastreastream has closed. Data log saved");
    //   logger.log_info("Waiting for new mocap datastream");
    // }
  }
}
