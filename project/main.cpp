// #include "pid.h"
#include "motor_mixing.h"
#include "pid_cascaded.h"
#include "plot.h"
#include "quad_2d.h"
#include "set_values.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <math_helper.h>
// Fastdds Headers
#include "mocap_quadcopterPubSubTypes.h"
#include "mocap_quadcopterPublisher.h"
// Px4 math header
#include "matrix/math.hpp"

int main() {
  Quad2D quad;
  PidCascadedController controller;

  // Set quadcopter parameters
  quad.set_initial_conditions("project/parameters/initial_conditions.yaml");
  float motor_commands[4] = {0, 0, 0, 0};

  // // Fastdds publisher and message initialization
  mocap_quadcopterPublisher pose_pub;
  bool fastdds_flag = false;

  if (pose_pub_flag) {
    fastdds_flag = pose_pub.init();
  }

  // Outer Loop: Position Control
  for (int i = 0; i < euler_steps; i++) {
    std::cout << "Timestep: " << i + 1 << '\n';

    // Get system state
    quad.sensor_read();

    // Outer loop
    const float thrust_command =
        controller.altitude_controller(quad, altitude_target, dt);

    const float attitude_command =
        controller.horizontal_controller(quad, horizontal_target, dt);

    // Inner loop
    const float torque_command =
        controller.attitude_controller(quad, attitude_command, dt);

    // Convert thrust, torque to motor speeds
    motor_mixing(motor_commands, thrust_command, torque_command, quad.k_f(),
                 quad.arm_length());

    // Dynamics function that accepts motor commands instead of thrusts
    quad.new_dynamics(motor_commands);

    // quad.dynamics(ff_thrust, torque_command);
    quad.euler_step(dt);

    // // Ony for tuning inner angle loop
    // quad.inner_loop_tuning_euler_step(dt);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // For plotting and logging
    ///////////////////////////////////////////////////////////////////////////////////////////

    // Diplay the control input and error
    // std::cout << "Thrust command:" << thrust_command << std::endl;
    // std::cout << "Altitude error:" << altitude_error << std::endl;
    // std::cout << "Angle Command:" << angle_command << std::endl;
    // std::cout << "Angle error:" << angle_error << std::endl;
    // std::cout << "Torque Command:" << torque_command << std::endl;
    // std::cout << "Vertical error:" << vertical_error << std::endl;
    // std::cout << "Motor commands:" << motor_commands[0] << std::endl;
    std::cout << std::endl;

    if (plot_flag) {
      // Set variables for plotting
      plot_var::z_plot[i] = quad.true_z();
      plot_var::x_plot[i] = quad.true_x();
      plot_var::thrust_plot[i] = thrust_command;
      plot_var::torque_plot[i] = torque_command;
      plot_var::beta_plot[i] = quad.true_beta() * (180 / M_PI);
      plot_var::t_plot[i] = i * dt;
    }

    if (pose_pub_flag && fastdds_flag) {
      // Publish mocap msg
      mocap_quadcopter msg;

      matrix::Eulerf euler(0, -quad.true_beta(), 0);
      matrix::Quatf q_nb(euler);
      // std::cout << "q_w" << q_nb(0);

      msg.index({(uint32_t)i + 1});
      msg.position({quad.true_x() * 1000, 0, quad.true_z() * 1000});
      // msg.orientation_quaternion({0, 0, 0, 1});
      msg.orientation_quaternion({q_nb(1), q_nb(2), q_nb(3), q_nb(0)});
      pose_pub.run(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(sim_time));
    }
  }

  // Plot the results

  if (plot_flag) {
    // Initialize visualizer
    MyApp app;
    app.run();
  }
}
