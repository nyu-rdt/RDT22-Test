from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    machine_node = Node(
        package='nyurdt_state_test',
        executable='single-machine',
        output='screen',
    )

    ld.add_action(machine_node)

    return ld