#include "include_helper.h"

int main() {

  // Initialize logger
  Logger logger(paths::event_log_path, paths::data_log_path);

  // Create participant. Argument-> Domain id, QOS name
  DefaultParticipant dp(0, "quadcopter_controller_qos");

  // Create fastdds publisher
  DDSPublisher motor_cmd_pub(idl_msg::QuadMotorCommandPubSubType(),
                             "motor_commands", dp.participant());

  // Initialize publisher with check
  if (motor_cmd_pub.init() == true) {
    logger.log_info("Initialized Motor command subscriber");
  } else {
    logger.log_error("Motor command publisher could be initialized");
    std::exit(EXIT_FAILURE);
  }

  // Create fastdds subscriber
  DDSSubscriber mocap_sub(idl_msg::MocapPubSubType(), "mocap_pose",
                          dp.participant());
  // Initialize subscriber with check
  if (mocap_sub.init() == true) {
    logger.log_info("Initialized Mocap subscriber");
  } else {
    logger.log_error("Mocap subscriber could be initialized");
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

  // bool session_end_flag = true;
  logger.log_info("Waiting for mocap datastream");

  // Needed in main
  cpp_msg::QuadMotorCommand motor_cmd{};
  cpp_msg::ThrustTorqueCommand thrust_torque_cmd{};

  for (;;) {
    // Lock until read and write are completed

    { // Wait until subscriber sends mocap pose
      std::unique_lock<std::mutex> lk(mocap_sub.listener.m);
      mocap_sub.listener.cv.wait(lk, [] { return sub::new_data_flag; });

      // Reset flag when data received
      sub::new_data_flag = false;

      // Run controller
      thrust_torque_cmd =
          controller.cascaded_controller(sub::mocap_msg.pose, target.pose());
    }

    // Convert thrust, torque to motor speeds
    motor_cmd = mixer.motor_mixer(thrust_torque_cmd);

    // Add quad name, timestamp and publish
    motor_cmd.header.id = "srl_quad";
    motor_cmd.header.timestamp = sub::mocap_msg.header.timestamp;
    motor_cmd_pub.publish(motor_cmd);

    // if (mocap_sub.listener.matched() == 0) {

    //   session_end_flag = true;
    //   logger.shutdown_data_logger();
    //   logger.log_info("Mocap datastreastream has closed. Data log
    //   saved"); logger.log_info("Waiting for new mocap datastream");
    // }
  }
}
