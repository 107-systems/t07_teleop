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
      parameters=[
        {'can_iface' : 'can0'},
        {'motor_left_topic': '/motor/left/target'},
        {'motor_right_topic': '/motor/right/target'},
      ]
    )
  ])
