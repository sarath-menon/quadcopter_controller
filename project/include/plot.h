#pragma once

#include "set_values.h"
#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>

using namespace mahi::gui;
using namespace mahi::util;

namespace plot_var {
const int euler_timesteps = 300;

// Variables to be plotted
float z_plot[euler_timesteps], x_plot[euler_timesteps],
    thrust_plot[euler_timesteps], torque_plot[euler_timesteps],
    beta_plot[euler_timesteps], t_plot[euler_timesteps];

// Plot axes limits
const int x_min = 0;
const int x_max = 5;
const int y_min = 0;
const int y_max = 20;

} // namespace plot_var

// Inherit from Application
class MyApp : public Application {
public:
  // 640x480 px window
  MyApp() : Application(1080, 720, "My App") {}
  // Override update (called once per frame)

  void update() override {
    // App logic and/or ImGui code goes here
    ImGui::Begin("Quadcopter plots");

    static float alpha = 0.25f;
    ImPlot::PushColormap(ImPlotColormap_Pastel);

    // Altitude plot
    if (altitude_plot_flag) {
      ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, 0, 8);
      if (ImPlot::BeginPlot("Altitude vs Time", "time", "altitude [m]",
                            ImVec2(-1, 200))) {
        ImPlot::PlotLine("altitude", plot_var::t_plot, plot_var::z_plot,
                         plot_var::euler_timesteps);
        ImPlot::EndPlot();
      }
    }

    if (translation_plot_flag) {
      // Translation plot
      ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, 0, 5);
      if (ImPlot::BeginPlot("Vertical vs Time", "time", "x distance [m]",
                            ImVec2(-1, 200))) {
        ImPlot::PlotLine("vertical distance", plot_var::t_plot,
                         plot_var::x_plot, plot_var::euler_timesteps);
        ImPlot::EndPlot();
      }
    }

    // Thrust input plot
    if (thrust_plot_flag) {
      ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, 0, 25);
      ImPlot::PushColormap(ImPlotColormap_Pastel);
      if (ImPlot::BeginPlot("Thrust input vs Time", "time",
                            "thrust input [N/kg]", ImVec2(-1, 200))) {
        ImPlot::PlotLine("thrust input", plot_var::t_plot,
                         plot_var::thrust_plot, plot_var::euler_timesteps);
        ImPlot::EndPlot();
      }
    }

    // Torque input plot
    if (torque_plot_flag) {
      ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, -5, 5);
      ImPlot::PushColormap(ImPlotColormap_Pastel);
      if (ImPlot::BeginPlot("Torque input vs Time", "time",
                            "Torque Input [Nm/kg]", ImVec2(-1, 200))) {
        ImPlot::PlotLine("torque input", plot_var::t_plot,
                         plot_var::torque_plot, plot_var::euler_timesteps);
        ImPlot::EndPlot();
      }
    }

    // Roll angle plot
    if (roll_angle_flag) {
      ImPlot::SetNextPlotLimits(plot_var::x_min, plot_var::x_max, -50, 50);
      ImPlot::PushColormap(ImPlotColormap_Pastel);
      if (ImPlot::BeginPlot("Roll angle vs Time", "time", "roll angle [deg]",
                            ImVec2(-1, 200))) {
        ImPlot::PlotLine("torque input", plot_var::t_plot, plot_var::beta_plot,
                         plot_var::euler_timesteps);
        ImPlot::EndPlot();
      }
    }

    ImGui::End();
  }
};
