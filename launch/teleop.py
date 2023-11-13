from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='t07_teleop',
      executable='t07_teleop_node',
      name='t07_teleop',
      namespace='t07',
      output='screen',
      emulate_tty=True,
      prefix="xterm -e",
      parameters=[
        {'motor_left_topic': '/motor/left/target'},
        {'motor_left_topic_deadline_ms': 100},
        {'motor_left_topic_liveliness_lease_duration': 1000},
        {'motor_right_topic': '/motor/right/target'},
        {'motor_right_topic_deadline_ms': 100},
        {'motor_right_topic_liveliness_lease_duration': 1000},
      ]
    )
  ])
