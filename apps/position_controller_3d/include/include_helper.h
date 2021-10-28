#pragma once

#include "matrix/math.hpp" // Px4 math header
#include "saturation_cutoff_mixer.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <math_helper.h>

// Utilities
#include "waypoint_setter.h"

// File containing ymal paths
#include "paths.h"

// spdlog headers
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/spdlog.h"

// Logger
#include "logger.h"

// Basic cascaded controller
#include "pid_position.h"

// Fastdds
#include "AttitudeCommandPubSubTypes.h"
#include "default_participant.h"
#include "default_publisher.h"
#include "default_subscriber.h"
#include "mocap_sub_callback.h"
#include "quadcopter_msgs/msgs/AttitudeCommand.h"
#include "reference_sub_callback.h"