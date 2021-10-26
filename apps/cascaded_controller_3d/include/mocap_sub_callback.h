#pragma once
#include "MocapPubSubTypes.h"
#include "default_subscriber.h"
#include "sensor_msgs/msgs/Mocap.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::Mocap mocap_msg{};
bool new_data_flag{false};
} // namespace sub

// Subscriber callback - gets executed when a sample is received
inline void DDSSubscriber::SubListener::on_data_available(
    eprosima::fastdds::dds::DataReader *reader) {
  eprosima::fastdds::dds::SampleInfo info;

  if (reader->take_next_sample(&sub::mocap_msg, &info) ==
      ReturnCode_t::RETCODE_OK) {
    if (info.valid_data) {
      // std::cout << "Sample received, count=" << samples << std::endl;
      {
        // Protection against race condition using mutex
        std::lock_guard lock(m);
        sub::new_data_flag = true;
      }
      cv.notify_one();
    }
  }
}
