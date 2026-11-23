from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensor_offset = LaunchConfiguration('sensor_offset')
    do_offset = LaunchConfiguration('do_offset')
    pub_odom_tf = LaunchConfiguration('pub_odom_tf')

    declare_sensor_offset = DeclareLaunchArgument(
        'sensor_offset',
        default_value='0.0',
        description='Offset from base_link to lidar in x direction'
    )

    declare_do_offset = DeclareLaunchArgument(
        'do_offset',
        default_value='True',
        description='Apply sensor offset compensation'
    )

    declare_pub_odom_tf = DeclareLaunchArgument(
        'pub_odom_tf',
        default_value='True',
        description='Publish odom to base_link TF'
    )

    relay_topics_node = Node(
        package='fl_nav2_helper',
        executable='relay_topics',
        name='relay_topics',
        output='screen',
        parameters=[{
            'sensor_offset': sensor_offset,
            'do_offset': do_offset,
            'pub_odom_tf': pub_odom_tf
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_sensor_offset)
    ld.add_action(declare_do_offset)
    ld.add_action(declare_pub_odom_tf)
    ld.add_action(relay_topics_node)

    return ld
