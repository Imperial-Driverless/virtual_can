from launch import LaunchDescription
from launch_ros.actions import Node

AMI_NOT_SELECTED = 0
AMI_ACCELERATION = 1
AMI_SKIDPAD = 2
AMI_AUTOCROSS = 3
AMI_TRACK_DRIVE = 4
AMI_STATIC_INSPECTION_A = 5
AMI_STATIC_INSPECTION_B = 6
AMI_AUTONOMOUS_DEMO = 7

def generate_launch_description():
    node = Node(
        package="virtual_can",
        executable="virtual_can_node",
        name="virtual_can",
        parameters=[
            {"use_sim_time": True},
            {"can_debug": 0},
            {"can_simulate": 0},
            {"can_interface": "vcan0"},
            {"debug_logging": False},
            {"loop_rate": 100},
            {"mission": AMI_TRACK_DRIVE},
            # {"rpm_limit": 100},
            # {"max_acc": 5.0},
            # {"max_braking": 5.0},
            # {"cmd_timeout": 0.5}
        ],
        # arguments=['--ros-args', '--log-level', 'debug'],
    )

    ld = LaunchDescription()
    ld.add_action(node)
    return ld
