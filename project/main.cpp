// #include "pid.h"
#include "motor_mixing.h"
#include "pid_cascaded.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <math_helper.h>

// Fastdds Headers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
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

  for (;;) {

    if (subscriber::new_data == true) {
      std::cout << "Position      :" << subscriber::position[0] << '\t'
                << subscriber::position[1] << '\t' << subscriber::position[2]
                << std::endl;

      std::cout << "Orientation:" << subscriber::orientation[0] << '\t'
                << subscriber::orientation[1] << '\t'
                << subscriber::orientation[2] << '\t'
                << subscriber::orientation[3] << std::endl
                << std::endl;

      // Set flag to false after data has been processed
      subscriber::new_data = false;
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

      // Sleep for 500 microseconds
    }
  }
}
} // namespace subscriber