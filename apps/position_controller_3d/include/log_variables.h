#pragma once
#include "safety_checks.h"
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace log_variables {

// Yaml paths
std::string quad_yaml =
    "cascaded_controller_app/parameters/quadcopter_parameters.yaml";

std::string controller_gains_yaml =
    "cascaded_controller_app/parameters/controller_gains.yaml";

std::string controller_timescales_yaml =
    "cascaded_controller_app/parameters/controller_timescales.yaml";

std::string setpoint_yaml = "cascaded_controller_app/parameters/setpoint.yaml";

// Log file paths
std::string event_log_path = "logs/event_logs/";
std::string data_log_path = "logs/data_logs/";

} // namespace log_variables