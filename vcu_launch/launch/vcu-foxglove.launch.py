from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    foxgloveBridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[
	    {
                "name": "vcu_bridge",
                "capabilities": ["clientPublish", "services"]
            }
	],
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        foxgloveBridge
    ])
