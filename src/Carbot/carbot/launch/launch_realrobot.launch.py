import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "carbot"

    # ===== EDIT THIS PATH IF YOUR XACRO FILE NAME / FOLDER IS DIFFERENT =====
    # Example common path: carbot/description/robot.urdf.xacro
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare(package_name),
            "description",
            "robot.urdf.xacro"
        ]),
        " ",
        "use_ros2_control:=true"
    ])

    robot_description = {
        "robot_description": robot_description_content
    }

    controller_params_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "my_controllers.yaml"
    ])

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False}
        ]
    )

    # ros2_control controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controller_params_file
        ],
        remappings=[
            ("/diff_cont/cmd_vel", "/cmd_vel"),
        ]
    )

    # Delay a little to let robot_state_publisher come up first
    delayed_controller_manager = TimerAction(
        period=2.0,
        actions=[controller_manager]
    )

    # Spawn joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_broad",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Spawn diff drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_cont",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Opening the Arduino serial port resets the board.  The hardware plugin
    # waits for it to boot, so give controller_manager time to finish hardware
    # activation before the spawners call its services.
    controller_startup_delay = 5.0

    delayed_joint_broad_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(
                period=controller_startup_delay,
                actions=[joint_broad_spawner]
            )]
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(
                period=controller_startup_delay,
                actions=[diff_drive_spawner]
            )]
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
    ])
