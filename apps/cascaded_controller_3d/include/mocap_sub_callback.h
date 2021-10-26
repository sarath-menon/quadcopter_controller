#pragma once
#include "MocapPubSubTypes.h"
#include "sensor_msgs/msgs/Mocap.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::Mocap mocap_msg{};
} // namespace sub
