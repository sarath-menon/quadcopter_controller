#pragma once

// Mocap data subscriber
#include "mocap_quadcopterPubSubTypes.h"
#include "mocap_quadcopterSubscriber.h"

void mocap_quadcopterSubscriber::SubListener::on_data_available(
    eprosima::fastdds::dds::DataReader *reader) {
  // Take data
  mocap_quadcopter st;
  eprosima::fastdds::dds::SampleInfo info;

  if (reader->take_next_sample(&st, &info) == ReturnCode_t::RETCODE_OK) {
    if (info.valid_data) {
      // Print your structure data here.
      ++samples;
      mocap_sub::new_data = true;

      // std::cout << "\nSample received, count=" << samples << std::endl;
      // std::cout << "Index=" << st.index() << std::endl;
      // std::cout << "Object Name:" << st.object_name() << std::endl;

      mocap_sub::object_name = st.object_name();
      mocap_sub::index = st.index();

      mocap_sub::position[0] = st.position().at(0) / 1000;
      mocap_sub::position[1] = st.position().at(1) / 1000;
      mocap_sub::position[2] = st.position().at(2) / 1000;

      mocap_sub::orientation[0] = st.orientation_quaternion().at(0);
      mocap_sub::orientation[1] = st.orientation_quaternion().at(1);
      mocap_sub::orientation[2] = st.orientation_quaternion().at(2);
      mocap_sub::orientation[3] = st.orientation_quaternion().at(3);

      mocap_sub::orientation_euler[0] = st.orientation_euler().at(0);
      mocap_sub::orientation_euler[1] = st.orientation_euler().at(1);
      mocap_sub::orientation_euler[2] = st.orientation_euler().at(2);

      mocap_sub::latency = st.delay();

      // Sleep for 1 millisecond
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}
