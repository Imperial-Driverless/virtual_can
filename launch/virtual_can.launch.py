from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package="virtual_can",
        executable="virtual_can_node",
        name="virtual_can",
        parameters=[
            # {"use_sim_time": True},
            {"can_debug": 0},
            {"can_simulate": 0},
            {"can_interface": "vcan0"},
            {"debug_logging": False},
            {"loop_rate": 100},
            # {"rpm_limit": 100},
            # {"max_acc": 5.0},
            # {"max_braking": 5.0},
            # {"cmd_timeout": 0.5}
        ],
    )

    ld = LaunchDescription()
    ld.add_action(node)
    return ld
