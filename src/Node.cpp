/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/t07_ros/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <t07_teleop/Node.h>

#include <unistd.h>
#include <termios.h>
#include <curses.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace t07_teleop
{

/**************************************************************************************
 * PROTOTYPE DECLARATION
 **************************************************************************************/

//static char getch();

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

Node::Node()
: rclcpp::Node("t07_teleop_node")
, _motor_left_qos_profile
{
  rclcpp::KeepLast(1),
  rmw_qos_profile_sensor_data
}
, _motor_left_target{0. * m/s}
, _motor_right_qos_profile
{
  rclcpp::KeepLast(1),
  rmw_qos_profile_sensor_data
}
, _motor_right_target{0. * m/s}
, _keyboard_mtx{}
, _keyboard_thread{}
, _keyboard_thread_active{false}
{
  init_pub_motor_left();
  init_pub_motor_right();

  _keyboard_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->keyboard_loop(); });

  _keyboard_thread = std::thread([this]()
  {
    _keyboard_thread_active = true;

    while(_keyboard_thread_active)
    {
      char const ch = getch();
      RCLCPP_INFO(get_logger(), "ch = %c", ch);

      /* Change target velocities depending on the pressed key. */
      if (tolower(ch) == 'w')
      {
        _motor_left_target  += 0.1 * m/s;
        _motor_right_target += 0.1 * m/s;
      }
      else if (tolower(ch) == 's')
      {
        _motor_left_target  -= 0.1 * m/s;
        _motor_right_target -= 0.1 * m/s;
      }
    }
  });


  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  _keyboard_thread_active = false;
  _keyboard_thread.join();

  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_pub_motor_left()
{
  declare_parameter("motor_left_topic", "/motor/left/target");
  declare_parameter("motor_left_topic_deadline_ms", 100);
  declare_parameter("motor_left_topic_liveliness_lease_duration", 1000);

  auto const motor_left_topic = get_parameter("motor_left_topic").as_string();
  auto const motor_left_topic_deadline = std::chrono::milliseconds(get_parameter("motor_left_topic_deadline_ms").as_int());
  auto const motor_left_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("motor_left_topic_liveliness_lease_duration").as_int());

  _motor_left_qos_profile.deadline(motor_left_topic_deadline);
  _motor_left_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _motor_left_qos_profile.liveliness_lease_duration(motor_left_topic_liveliness_lease_duration);

  _motor_left_pub = create_publisher<std_msgs::msg::Float32>(motor_left_topic, _motor_left_qos_profile);
}

void Node::init_pub_motor_right()
{
  declare_parameter("motor_right_topic", "/motor/right/target");
  declare_parameter("motor_right_topic_deadline_ms", 100);
  declare_parameter("motor_right_topic_liveliness_lease_duration", 1000);

  auto const motor_right_topic = get_parameter("motor_right_topic").as_string();
  auto const motor_right_topic_deadline = std::chrono::milliseconds(get_parameter("motor_right_topic_deadline_ms").as_int());
  auto const motor_right_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("motor_right_topic_liveliness_lease_duration").as_int());

  _motor_right_qos_profile.deadline(motor_right_topic_deadline);
  _motor_right_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _motor_right_qos_profile.liveliness_lease_duration(motor_right_topic_liveliness_lease_duration);

  _motor_right_pub = create_publisher<std_msgs::msg::Float32>(motor_right_topic, _motor_right_qos_profile);
}

void Node::keyboard_loop()
{
  /* Publish the target velocities. */
  {
    std_msgs::msg::Float32 motor_left_target_msg;
    motor_left_target_msg.data = _motor_left_target.numerical_value_in(m/s);
    _motor_left_pub->publish(motor_left_target_msg);
  }
}

/**************************************************************************************
 * PROTOTYPE DEFINITION
 **************************************************************************************/

/* https://stackoverflow.com/questions/7469139/what-is-the-equivalent-to-getch-getche-in-linux */
//char getch()
//{
//  char buf = 0;
//  struct termios old;
//  fflush(stdout);
//  if(tcgetattr(0, &old) < 0)
//    perror("tcsetattr()");
//  old.c_lflag &= ~ICANON;
//  old.c_lflag &= ~ECHO;
//  old.c_cc[VMIN] = 1;
//  old.c_cc[VTIME] = 0;
//  if(tcsetattr(0, TCSANOW, &old) < 0)
//    perror("tcsetattr ICANON");
//  if(read(0, &buf, 1) < 0)
//    perror("read()");
//  old.c_lflag |= ICANON;
//  old.c_lflag |= ECHO;
//  if(tcsetattr(0, TCSADRAIN, &old) < 0)
//    perror("tcsetattr ~ICANON");
//  return buf;
//}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07_teleop */
