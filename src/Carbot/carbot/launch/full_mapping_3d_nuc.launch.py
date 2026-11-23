from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')
    sam_config_file = LaunchConfiguration('sam_config_file')
    odom_topic = LaunchConfiguration('odom_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    rviz = LaunchConfiguration('rviz')
    start_livox = LaunchConfiguration('start_livox')
    start_robot = LaunchConfiguration('start_robot')
    start_relay = LaunchConfiguration('start_relay')

    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio2_share = FindPackageShare('fast_lio2')
    fast_lio_sam_share = FindPackageShare('fast_lio_sam_qn')
    carbot_share = FindPackageShare('carbot')
    fl_nav2_helper_share = FindPackageShare('fl_nav2_helper')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time. For real NUC robot, keep false.'
        ),
        DeclareLaunchArgument(
            'fast_lio_config_file',
            default_value='mid360.yaml',
            description='FAST-LIO2 config file name from fast_lio2/config.'
        ),
        DeclareLaunchArgument(
            'sam_config_file',
            default_value=PathJoinSubstitution([
                fast_lio_sam_share,
                'config',
                'config.yaml'
            ]),
            description='FAST-LIO-SAM-QN2 params file.'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description='FAST-LIO2 odometry topic consumed by FAST-LIO-SAM-QN2.'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/cloud_registered',
            description='FAST-LIO2 registered point cloud topic consumed by FAST-LIO-SAM-QN2.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start FAST-LIO-SAM-QN2 RViz view.'
        ),
        DeclareLaunchArgument(
            'start_livox',
            default_value='true',
            description='Start the Livox MID360 driver.'
        ),
        DeclareLaunchArgument(
            'start_robot',
            default_value='true',
            description='Start robot_state_publisher and ros2_control.'
        ),
        DeclareLaunchArgument(
            'start_relay',
            default_value='true',
            description='Start helper relay for odometry/TF topics.'
        ),

        # 1. Livox MID360 driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    livox_share,
                    'launch_ROS2',
                    'msg_MID360_launch.py'
                ])
            ),
            condition=IfCondition(start_livox)
        ),

        # 2. FAST-LIO2 odometry, using the FAST-LIO2 copy bundled with SAM-QN2.
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            fast_lio2_share,
                            'launch',
                            'mapping.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'config_file': fast_lio_config_file,
                        'rviz': 'false',
                    }.items()
                )
            ]
        ),

        # 3. FAST-LIO-SAM-QN2 pose graph / loop-closure 3D SLAM.
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='fast_lio_sam_qn',
                    executable='fast_lio_sam_qn_node',
                    name='fast_lio_sam_qn_node',
                    output='screen',
                    parameters=[
                        sam_config_file,
                        {'use_sim_time': use_sim_time}
                    ],
                    remappings=[
                        ('/Odometry', odom_topic),
                        ('/cloud_registered', lidar_topic),
                    ]
                )
            ]
        ),

        # 4. Real robot bringup for driving while mapping.
        TimerAction(
            period=10.0,
            condition=IfCondition(start_robot),
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

        # 5. Relay FAST-LIO odometry / TF topics for the rest of the robot stack.
        TimerAction(
            period=13.0,
            condition=IfCondition(start_relay),
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

        # 6. FAST-LIO-SAM-QN2 visualization.
        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_sam',
                    arguments=['-d', PathJoinSubstitution([
                        fast_lio_sam_share,
                        'rviz',
                        'sam_rviz.rviz'
                    ])],
                    condition=IfCondition(rviz),
                    prefix='nice',
                    output='screen'
                )
            ]
        ),
    ])
