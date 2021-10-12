#include "include_helper.h"

int main() {

  // Initialize logger
  Logger logger(paths::event_log_path, paths::data_log_path);

  // Initialize motor command publisher
  motor_commandsPublisher motor_command_pub;
  actuator_commands msg;

  // New fastdds publisher
  // Message
  msgs::QuadMotorCommand motor_cmd;
  // Create publisher with msg type
  DDSPublisher motor_cmd_pub(QuadMotorCommandPubSubType(), "motor_commands");
  motor_cmd_pub.init();

  // New fastdds subscriber
  DDSSubscriber mocap_sub_new(MocapPubSubType(), "mocap_pose");
  mocap_sub_new.init();

  // try {
  //   if (motor_command_pub.init() == true)
  //     logger.log_info("Initialized Motor command subscriber");
  //   else
  //     throw(motor_command_pub);
  // } catch (motor_commandsPublisher motor_command_pub) {
  //   logger.log_error("Motor command publisher cannot be initialized");
  //   std::exit(EXIT_FAILURE);
  // }

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

      // wait for the subscriber
      std::unique_lock<std::mutex> lk(mocap_sub_new.listener.m);
      mocap_sub_new.listener.cv.wait(lk, [] { return sub::new_data_flag; });

      sub::new_data_flag = false;

      sub::st.pose.position.x /= 1000.0;
      sub::st.pose.position.y /= 1000.0;
      sub::st.pose.position.z /= 1000.0;

      // Run controller
      thrust_torque_cmd =
          controller.cascaded_controller(sub::st.pose, target.position());
    }

    // Convert thrust, torque to motor speeds
    motor_commands = mixer.motor_mixer(thrust_torque_cmd);

    // New fastdds -> publish motor commands
    motor_cmd.header.id = "srl_quad";
    motor_cmd.header.timestamp = sub::st.header.timestamp;
    motor_cmd.motorspeed[0] = motor_commands(0);
    motor_cmd.motorspeed[1] = motor_commands(1);
    motor_cmd.motorspeed[2] = motor_commands(2);
    motor_cmd.motorspeed[3] = motor_commands(3);
    motor_cmd_pub.publish(motor_cmd);

    // if (session_end_flag == false && mocap_sub::matched == 0) {

    //   session_end_flag = true;
    //   logger.shutdown_data_logger();
    //   logger.log_info("Mocap datastreastream has closed. Data log saved");
    //   logger.log_info("Waiting for new mocap datastream");
    // }
  }
}
