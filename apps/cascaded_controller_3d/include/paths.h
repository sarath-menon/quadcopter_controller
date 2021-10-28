#pragma once
#include "safety_checks.h"
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace paths {

// Yaml paths

const std::string folder_path = "apps/cascaded_controller_3d/parameters/";

const std::string quad_yaml = folder_path + "quadcopter_parameters.yaml";

const std::string controller_gains_yaml = folder_path + "controller_gains.yaml";

const std::string controller_timescales_yaml =
    folder_path + "controller_timescales.yaml";

const std::string setpoint_yaml = folder_path + "setpoint.yaml";

// Text file paths

const std::string event_log_path = "logs/event_logs/";
const std::string data_log_path = "logs/data_logs/";

} // namespace paths