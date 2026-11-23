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
    localization_config_file = LaunchConfiguration('localization_config_file')
    saved_map_bag = LaunchConfiguration('saved_map_bag')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    twist_mux_params_file = LaunchConfiguration('twist_mux_params_file')
    collision_monitor_params_file = LaunchConfiguration('collision_monitor_params_file')
    pointcloud_to_laserscan_params_file = LaunchConfiguration(
        'pointcloud_to_laserscan_params_file'
    )
    odom_topic = LaunchConfiguration('odom_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    start_livox = LaunchConfiguration('start_livox')
    start_robot = LaunchConfiguration('start_robot')
    start_relay = LaunchConfiguration('start_relay')
    start_twist_mux = LaunchConfiguration('start_twist_mux')
    start_collision_monitor = LaunchConfiguration('start_collision_monitor')

    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio2_share = FindPackageShare('fast_lio2')
    fast_lio_localization_share = FindPackageShare('fast_lio_localization_sc_qn')
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
            'localization_config_file',
            default_value=PathJoinSubstitution([
                fast_lio_localization_share,
                'config',
                'config.yaml'
            ]),
            description='FAST-LIO-Localization-SC-QN2 params file.'
        ),
        DeclareLaunchArgument(
            'saved_map_bag',
            default_value='/home/beinglab-kilted/carbot_ws/install/fast_lio_sam_qn/share/fast_lio_sam_qn/result.bag',
            description='3D keyframe map bag produced by FAST-LIO-SAM-QN2.'
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='/home/beinglab-kilted/carbot_ws/maps/map_grid.yaml',
            description='2D occupancy grid YAML produced from the 3D PCD map.'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                fl_nav2_helper_share,
                'config',
                'nav2_params.yaml'
            ]),
            description='Nav2 params file.'
        ),
        DeclareLaunchArgument(
            'collision_monitor_params_file',
            default_value=PathJoinSubstitution([
                fl_nav2_helper_share,
                'config',
                'collision_monitor.yaml'
            ]),
            description='Nav2 collision monitor params file.'
        ),
        DeclareLaunchArgument(
            'twist_mux_params_file',
            default_value=PathJoinSubstitution([
                carbot_share,
                'config',
                'twist_mux.yaml'
            ]),
            description='twist_mux params file.'
        ),
        DeclareLaunchArgument(
            'pointcloud_to_laserscan_params_file',
            default_value=PathJoinSubstitution([
                fl_nav2_helper_share,
                'config',
                'pointcloud_to_laserscan.yaml'
            ]),
            description='pointcloud_to_laserscan params file for Nav2 obstacle input.'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description='FAST-LIO2 odometry topic consumed by localization.'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/cloud_registered',
            description='FAST-LIO2 registered cloud topic consumed by localization.'
        ),
        DeclareLaunchArgument(
            'cloud_in_topic',
            default_value='/cloud_registered_body',
            description='Point cloud topic converted to LaserScan for Nav2 obstacle layers.'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/lidar_scan',
            description='Intermediate LaserScan topic relayed to /scan.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz for 3D localization and Nav2 control.'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                carbot_share,
                'rviz',
                'navigation_3d_control.rviz'
            ]),
            description='RViz config file for 3D localization and Nav2 navigation control.'
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
            description='Start helper relay for odometry/TF and /scan topics.'
        ),
        DeclareLaunchArgument(
            'start_twist_mux',
            default_value='true',
            description='Start twist_mux for Nav2 and teleop velocity arbitration.'
        ),
        DeclareLaunchArgument(
            'start_collision_monitor',
            default_value='true',
            description='Start Nav2 collision monitor.'
        ),

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
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='odom_to_camera_init_tf',
                    arguments=[
                        '--x', '0', '--y', '0', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'odom',
                        '--child-frame-id', 'camera_init',
                    ],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=10.0,
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

        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='fast_lio_localization_sc_qn',
                    executable='fast_lio_localization_sc_qn_node',
                    name='fast_lio_localization_sc_qn_node',
                    output='screen',
                    parameters=[
                        localization_config_file,
                        {
                            'use_sim_time': use_sim_time,
                            'basic.saved_map': saved_map_bag,
                        }
                    ],
                    remappings=[
                        ('/Odometry', odom_topic),
                        ('/cloud_registered', lidar_topic),
                    ]
                )
            ]
        ),

        TimerAction(
            period=6.0,
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

        TimerAction(
            period=17.0,
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

        TimerAction(
            period=19.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'yaml_filename': map_yaml_file},
                    ]
                ),
            ]
        ),

        TimerAction(
            period=21.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map_server',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']},
                    ]
                ),
            ]
        ),

        TimerAction(
            period=24.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            carbot_share,
                            'launch',
                            'navigation_launch.py'
                        ])
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params_file,
                    }.items()
                )
            ]
        ),

        TimerAction(
            period=21.5,
            condition=IfCondition(start_collision_monitor),
            actions=[
                Node(
                    package='nav2_collision_monitor',
                    executable='collision_monitor',
                    name='collision_monitor',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        collision_monitor_params_file,
                        {'use_sim_time': use_sim_time},
                    ],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                    ],
                )
            ]
        ),

        TimerAction(
            period=23.0,
            condition=IfCondition(start_collision_monitor),
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_collision_monitor',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['collision_monitor']},
                        {'bond_timeout': 10.0},
                    ],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                    ],
                )
            ]
        ),

        TimerAction(
            period=26.0,
            condition=IfCondition(start_twist_mux),
            actions=[
                Node(
                    package='twist_mux',
                    executable='twist_mux',
                    name='twist_mux',
                    output='screen',
                    parameters=[twist_mux_params_file],
                    remappings=[
                        ('cmd_vel_out', '/cmd_vel'),
                    ]
                )
            ]
        ),

        TimerAction(
            period=34.0,
            actions=[
                Node(
                    package='carbot',
                    executable='route_mission_client',
                    name='route_mission_client',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([
                            carbot_share,
                            'config',
                            'voice_node_aliases.yaml'
                        ]),
                        {'use_sim_time': use_sim_time},
                    ]
                )
            ]
        ),

        TimerAction(
            period=36.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_navigation_3d',
                    arguments=['-d', rviz_config_file],
                    condition=IfCondition(rviz),
                    output='screen'
                )
            ]
        ),
    ])
