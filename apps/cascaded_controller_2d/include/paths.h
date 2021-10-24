#pragma once
#include "safety_checks.h"
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace paths {

// Yaml paths

std::string folder_path = "apps/cascaded_controller_2d/parameters/";

std::string quad_yaml = folder_path + "quadcopter_parameters.yaml";

std::string controller_gains_yaml = folder_path + "controller_gains.yaml";

std::string controller_timescales_yaml =
    folder_path + "controller_timescales.yaml";

std::string setpoint_yaml = folder_path + "setpoint.yaml";

// Vector of yaml files for safety check
std::vector<std::string> files_to_check = {quad_yaml, controller_gains_yaml,
                                           controller_timescales_yaml,
                                           setpoint_yaml};

// Text file paths

std::string event_log_path = "logs/event_logs/";
std::string data_log_path = "logs/data_logs/";

} // namespace paths