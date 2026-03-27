import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def get_config_file_path(name, dir='launch/vcu/params'):
    ret = os.path.join(
        get_package_share_directory('vcu_launch'),
        dir,
        name
    )
    return ret

def generate_launch_description():

    config_inverter = get_config_file_path("inverterParams.yaml")
    config_vcu = get_config_file_path('vcuParams.yaml')
    config_cooling = get_config_file_path('coolingParams.yaml')
    config_stateMachine = get_config_file_path('stateMachineParams.yaml')
    config_watchdog = get_config_file_path("watchdogParams.yaml")


    nodeVCU = Node(
        package='vcu',
        executable='vcu',
        parameters=[config_inverter, config_vcu],
        remappings=[
            # TODO: remove these remappings once the other repositories are updated
            ('/vcu/data/motorSpeeds', '/sensors/motor_speeds'),
            ('/vcu/data/motorTorques', '/sensors/motor_torques'),
        ],
        output="screen",
        emulate_tty=True)

    nodeStateMachine = Node(
        package='state_machine',
        executable='state_machine_node',
        parameters=[config_stateMachine],
        output="screen",
        emulate_tty=True)

    nodeMailman = Node(
        package='mailman',
        executable='mailman',
        parameters=[],
        output="screen",
        emulate_tty=True)

    nodeWatchdog = Node(
        package='vcu_watchdog',
        executable='vcu_watchdog',
        parameters=[config_watchdog],
        output="screen",
        emulate_tty=True)

    nodeCooling = Node(
        package='cooling',
        executable='cooling_vcu',
        parameters=[config_cooling],
        output="screen",
        emulate_tty=True)

    statistics = Node(
        package='vcu_statistics',
        executable='vcu_statistics',
        parameters=[],
        output="screen",
        emulate_tty=True)

    analytics = Node(
        package='vcu_analytics',
        executable='vcu_analytics',
        parameters=[],
        output="screen",
        emulate_tty=True)

    return LaunchDescription([
        nodeVCU, nodeMailman, nodeStateMachine, nodeWatchdog, nodeCooling, 
        statistics, analytics])
