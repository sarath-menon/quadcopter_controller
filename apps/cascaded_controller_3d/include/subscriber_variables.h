#pragma once
#include "MocapPubSubTypes.h"
#include "QuadPositionCmdPubSubTypes.h"
#include "quadcopter_msgs/msgs/QuadPositionCmd.h"
#include "sensor_msgs/msgs/Mocap.h"

// Subscriber data that needs to be accessed in main
namespace sub {
cpp_msg::Mocap mocap_msg{};
cpp_msg::QuadPositionCmd pos_cmd{};
} // namespace sub
