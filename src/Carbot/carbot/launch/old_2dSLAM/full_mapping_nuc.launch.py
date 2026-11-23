from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    pointcloud_to_laserscan_params_file = LaunchConfiguration(
        'pointcloud_to_laserscan_params_file'
    )

    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio')
    carbot_share = FindPackageShare('carbot')
    fl_nav2_helper_share = FindPackageShare('fl_nav2_helper')
    slam_toolbox_share = FindPackageShare('slam_toolbox')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time. For real NUC robot, keep false.'
        ),

        DeclareLaunchArgument(
            'fast_lio_config_file',
            default_value='mid360.yaml',
            description='FAST-LIO config file'
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                fl_nav2_helper_share,
                'config',
                'mapper_params_online_async.yaml'
            ]),
            description='slam_toolbox mapper params file'
        ),

        DeclareLaunchArgument(
            'pointcloud_to_laserscan_params_file',
            default_value=PathJoinSubstitution([
                fl_nav2_helper_share,
                'config',
                'pointcloud_to_laserscan.yaml'
            ]),
            description='pointcloud_to_laserscan params file'
        ),

        DeclareLaunchArgument(
            'cloud_in_topic',
            default_value='/cloud_registered_body',
            description='Input point cloud topic from FAST-LIO'
        ),

        DeclareLaunchArgument(
            'scan_topic',
            default_value='/lidar_scan',
            description='Output LaserScan topic for slam_toolbox'
        ),

        # 1. Livox MID360 driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    livox_share,
                    'launch_ROS2',
                    'msg_MID360_launch.py'
                ])
            )
        ),

        # 2. FAST-LIO mapping / odometry
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            fast_lio_share,
                            'launch',
                            'mapping.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'config_file': fast_lio_config_file,
                    }.items()
                )
            ]
        ),

        # 3. Convert FAST-LIO point cloud to LaserScan
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    output='screen',
                    remappings=[
                        ('cloud_in', cloud_in_topic),
                        ('scan', scan_topic),
                    ],
                    parameters=[
                        pointcloud_to_laserscan_params_file
                    ]
                )
            ]
        ),

        # 4. Real robot bringup
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            carbot_share,
                            'launch',
                            'launch_realrobot.launch.py'
                        ])
                    )
                )
            ]
        ),

        # 5. Relay FAST-LIO odometry / TF topics
        TimerAction(
            period=13.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            fl_nav2_helper_share,
                            'launch',
                            'relay_topics.launch.py'
                        ])
                    )
                )
            ]
        ),

        # 6. slam_toolbox online mapping
        TimerAction(
            period=16.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            slam_toolbox_share,
                            'launch',
                            'online_async_launch.py'
                        ])
                    ),
                    launch_arguments={
                        'slam_params_file': slam_params_file,
                        'use_sim_time': use_sim_time,
                    }.items()
                )
            ]
        ),
    ])