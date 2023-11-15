/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/t07_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <unistd.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <mp-units/systems/si/si.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::s;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static int getch_timeout(int const timeout_ms)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 0;  // Set to 0 for non-blocking
  newt.c_cc[VTIME] = 0; // Set to 0 for non-blocking
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Use select for a timeout
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  int ready = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);

  if (ready > 0) {
    // Input is available, read the character
    ch = getchar();
  } else {
    // Timeout occurred
    ch = EOF;
  }

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("t07_teleop_node");


  node->declare_parameter("motor_left_topic", "/motor/left/target");
  node->declare_parameter("motor_left_topic_deadline_ms", 100);
  node->declare_parameter("motor_left_topic_liveliness_lease_duration", 1000);

  auto const motor_left_topic = node->get_parameter("motor_left_topic").as_string();
  auto const motor_left_topic_deadline = std::chrono::milliseconds(node->get_parameter("motor_left_topic_deadline_ms").as_int());
  auto const motor_left_topic_liveliness_lease_duration = std::chrono::milliseconds(node->get_parameter("motor_left_topic_liveliness_lease_duration").as_int());

  rclcpp::QoS motor_left_qos_profile{rclcpp::KeepLast(1), rmw_qos_profile_sensor_data};
  motor_left_qos_profile.deadline(motor_left_topic_deadline);
  motor_left_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  motor_left_qos_profile.liveliness_lease_duration(motor_left_topic_liveliness_lease_duration);

  auto const motor_left_pub = node->create_publisher<std_msgs::msg::Float32>(motor_left_topic, motor_left_qos_profile);


  node->declare_parameter("motor_right_topic", "/motor/right/target");
  node->declare_parameter("motor_right_topic_deadline_ms", 100);
  node->declare_parameter("motor_right_topic_liveliness_lease_duration", 1000);

  auto const motor_right_topic = node->get_parameter("motor_right_topic").as_string();
  auto const motor_right_topic_deadline = std::chrono::milliseconds(node->get_parameter("motor_right_topic_deadline_ms").as_int());
  auto const motor_right_topic_liveliness_lease_duration = std::chrono::milliseconds(node->get_parameter("motor_right_topic_liveliness_lease_duration").as_int());

  rclcpp::QoS motor_right_qos_profile{rclcpp::KeepLast(1), rmw_qos_profile_sensor_data};
  motor_right_qos_profile.deadline(motor_right_topic_deadline);
  motor_right_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  motor_right_qos_profile.liveliness_lease_duration(motor_right_topic_liveliness_lease_duration);

  auto const motor_right_pub = node->create_publisher<std_msgs::msg::Float32>(motor_right_topic, motor_right_qos_profile);

  try
  {
    quantity<m/s> motor_left_target  = 0. * m/s;
    quantity<m/s> motor_right_target = 0. * m/s;

    while(rclcpp::ok())
    {
      /* Wait for a key to be pressed. */
      int const ch = getch_timeout(100);

      /* Change target velocities depending on keyboard input. */
      if (ch != EOF)
      {
        if (tolower(ch) == 'w')
        {
          motor_left_target  += 0.1 * m/s;
          motor_right_target += 0.1 * m/s;
        }
        else if (tolower(ch) == 's')
        {
          motor_left_target  -= 0.1 * m/s;
          motor_right_target -= 0.1 * m/s;
        }
        else if (tolower(ch) == 'a')
        {
          motor_left_target  += 0.05 * m/s;
          motor_right_target -= 0.05 * m/s;
        }
        else if (tolower(ch) == 'd')
        {
          motor_left_target  -= 0.05 * m/s;
          motor_right_target += 0.05 * m/s;
        }
        else if (tolower(ch) == ' ')
        {
          motor_left_target  = 0. * m/s;
          motor_right_target = 0. * m/s;
        }
      }

      /* Publish the target velocities. */
      {
        std_msgs::msg::Float32 motor_left_target_msg;
        motor_left_target_msg.data = motor_left_target.numerical_value_in(m/s);
        motor_left_pub->publish(motor_left_target_msg);
      }
      {
        std_msgs::msg::Float32 motor_right_target_msg;
        motor_right_target_msg.data = motor_right_target.numerical_value_in(m/s);
        motor_right_pub->publish(motor_right_target_msg);
      }

      /* Let ROS do its things. */
      rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  }
  catch (std::runtime_error const & err)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Exception (std::runtime_error) caught: %s\nTerminating ...", err.what());
    return EXIT_FAILURE;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Unhandled exception caught.\nTerminating ...");
    return EXIT_FAILURE;
  }
}
