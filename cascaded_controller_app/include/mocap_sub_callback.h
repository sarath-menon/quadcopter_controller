#pragma once
#include "MocapPubSubTypes.h"
#include "default_subscriber.h"
#include "sensor_msgs/msgs/Mocap.h"

// Subscriber data that needs to be accessed in main
namespace sub {
msgs::Mocap msg;
bool new_data_flag{false};
} // namespace sub

// Subscriber callback - gets executed when a sample is received
inline void DDSSubscriber::SubListener::on_data_available(DataReader *reader) {
  SampleInfo info;

  if (reader->take_next_sample(&sub::msg, &info) == ReturnCode_t::RETCODE_OK) {
    if (info.valid_data) {
      { // Protection against race condition using mutex
        std::unique_lock<std::mutex> lock(m);

        // std::cout << "Sample received, count=" << samples << std::endl;

        sub::new_data_flag = true;
        // std::cout << "subscriber signals data ready for processing\n";
        lock.unlock();
      }
      cv.notify_one();
    }
  }
}