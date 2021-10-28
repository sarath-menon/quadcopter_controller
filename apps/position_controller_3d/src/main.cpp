#include "include_helper.h"

int main() {

  // Initialize logger
  Logger logger(paths::event_log_path, paths::data_log_path);

  // Create participant. Argument-> Domain id, QOS name
  DefaultParticipant dp(0, "quadcopter_controller_qos");

  // Create fastdds publisher
  DDSPublisher attitude_cmd_pub(idl_msg::AttitudeCommandPubSubType(),
                                "attitude_command", dp.participant());

  // Initialize publisher with check
  if (attitude_cmd_pub.init() == true) {
    logger.log_info("Initialized Motor command subscriber");
  } else {
    logger.log_error("Motor command publisher could be initialized");
    std::exit(EXIT_FAILURE);
  }

  // Create mocap subscriber
  DDSSubscriber mocap_sub(idl_msg::MocapPubSubType(), &sub::mocap_msg,
                          "mocap_pose", dp.participant());
  // Initialize subscriber with check
  if (mocap_sub.init() == true) {
    logger.log_info("Initialized Mocap subscriber");
  } else {
    logger.log_error("Mocap subscriber could be initialized");
    std::exit(EXIT_FAILURE);
  }

  // Create reference cmd subscriber
  DDSSubscriber cmd_sub(idl_msg::QuadPositionCmdPubSubType(), &sub::pos_cmd,
                        "pos_cmd", dp.participant());

  // Initialize subscriber
  if (cmd_sub.init() == true) {
    logger.log_info("Initialized Reference command subscriber");
  } else {
    logger.log_error("Reference command subscriber could be initialized");
    std::exit(EXIT_FAILURE);
  }
  // Intiailize fastdds subscriber

  // Create cascaded pid controller
  controllers_3d::PositionPid controller;
  controller.set_gains(paths::controller_gains_yaml);
  controller.set_quad_properties(paths::quad_yaml);
  logger.log_info("Initialized Controller");

  // Create quadcopter mixer
  QuadcopterMixer mixer;
  // Set quadcopter parameters in mixer
  mixer.set_quad_properties(paths::quad_yaml);
  logger.log_info("Initialized Mixer");

  // bool session_end_flag = true;
  logger.log_info("Waiting for mocap datastream");

  // Needed in main
  cpp_msg::AttitudeCommand attitude_cmd{};

  for (;;) {
    // // Wait until new reference cmd is available
    // mocap_sub.listener->wait_for_data();

    // Run controller
    attitude_cmd =
        controller.position_controller(sub::mocap_msg.pose, sub::pos_cmd);

    // Add quad name, timestamp and publish
    attitude_cmd.header.id = "srl_quad";
    attitude_cmd.header.timestamp = sub::mocap_msg.header.timestamp;
    attitude_cmd_pub.publish(attitude_cmd);

    // // Print info
    // std::cout << "Cmd: " << thrust_torque_cmd.thrust << '\t'
    //           << thrust_torque_cmd.roll_torque << '\t'
    //           << thrust_torque_cmd.pitch_torque << '\t'
    //           << thrust_torque_cmd.yaw_torque << std::endl;

    // if (mocap_sub.listener.matched() == 0) {

    //   session_end_flag = true;
    //   logger.shutdown_data_logger();
    //   logger.log_info("Mocap datastreastream has closed. Data log
    //   saved"); logger.log_info("Waiting for new mocap datastream");
    // }
  }
}
