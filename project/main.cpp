// #include "pid.h"
#include "motor_mixing.h"
#include "pid_cascaded.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <math_helper.h>

// Fastdds Headers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Mocap data subscriber
#include "mocap_quadcopterPubSubTypes.h"
#include "mocap_quadcopterSubscriber.h"
// Motor commands publisher
#include "motor_commandsPubSubTypes.h"
#include "motor_commandsPublisher.h"
/////////////////////////////////////////////////////////////////////////////////

// Px4 math header
#include "matrix/math.hpp"

int main() {

  // Initialize motor commands subscriber
  motor_commandsPublisher motor_command_pub;
  bool fastdds_flag = false;

  // Initialize mocap data subscriber
  subscriber::mocap_quadcopterSubscriber mysub;
  mysub.init();

  // Initialize cascaded pid controller
  PidCascadedController controller;

  // Initialize for now
  constexpr static float z_position_target = 5;
  constexpr static float thrust_max = 25;
  constexpr static float thrust_min = 7;
  constexpr static float dt = 0.01;

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

      // Set flag to false after data has been processed
      subscriber::new_data = false;

      std::cout << "Thrust command:" << thrust_command;
    }
  }
}

namespace subscriber {
void mocap_quadcopterSubscriber::SubListener::on_data_available(
    eprosima::fastdds::dds::DataReader *reader) {
  // Take data
  mocap_quadcopter st;
  eprosima::fastdds::dds::SampleInfo info;

  if (reader->take_next_sample(&st, &info) == ReturnCode_t::RETCODE_OK) {
    if (info.valid_data) {
      // Print your structure data here.
      ++samples;
      new_data = true;

      // std::cout << "\nSample received, count=" << samples << std::endl;
      // std::cout << "Index=" << st.index() << std::endl;
      // std::cout << "Object Name:" << st.object_name() << std::endl;

      object_name = st.object_name();
      frame_number = st.index();

      position[0] = st.position().at(0);
      position[1] = st.position().at(1);
      position[2] = st.position().at(2);

      orientation[0] = st.orientation_quaternion().at(0);
      orientation[1] = st.orientation_quaternion().at(1);
      orientation[2] = st.orientation_quaternion().at(2);
      orientation[3] = st.orientation_quaternion().at(3);

      latency = st.delay();

      // Sleep for 1 millisecond
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

} // namespace subscriber