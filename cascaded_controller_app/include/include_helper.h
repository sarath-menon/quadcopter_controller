#pragma once

#include "matrix/math.hpp" // Px4 math header
#include "pid_cascaded.h"
#include "saturation_cutoff_mixer.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <math_helper.h>

// Fastdds  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Mocap data subscriber
#include "mocap_quadcopterPubSubTypes.h"
#include "mocap_quadcopterSubscriber.h"
// Motor commands publisher
#include "motor_commandsPubSubTypes.h"
#include "motor_commandsPublisher.h"

/////////////////////////////////////////////////////////////////////////////////

// Utilities  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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