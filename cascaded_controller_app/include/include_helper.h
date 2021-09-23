#pragma once

#include "matrix/math.hpp" // Px4 math header
#include "motor_mixing.h"
#include "pid_cascaded.h"
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

// Subscriber callbacks
#include "mocap_sub_callback.h"
/////////////////////////////////////////////////////////////////////////////////
