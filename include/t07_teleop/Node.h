/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/t07_teleop/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <mp-units/systems/si/si.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::s;

namespace t07_teleop
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
   Node();
  ~Node();

private:
  rclcpp::QoS _motor_left_qos_profile;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _motor_left_pub;
  quantity<m/s> _motor_left_target;
  void init_pub_motor_left();

  rclcpp::QoS _motor_right_qos_profile;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _motor_right_pub;
  quantity<m/s> _motor_right_target;
  void init_pub_motor_right();

  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _keyboard_loop_timer;
  void keyboard_loop();

  std::mutex _keyboard_mtx;
  std::thread _keyboard_thread;
  std::atomic<bool> _keyboard_thread_active;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07_teleop */
