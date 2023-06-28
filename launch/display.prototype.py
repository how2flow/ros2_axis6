import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    # files
    urdf_file = os.path.join(
        get_package_share_directory('axis6'),
        'urdf',
        'prototype.urdf')
    rviz_file = os.path.join(
        get_package_share_directory('axis6'),
        'rviz',
        'ros2_axis6.rviz')

    # arguments
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    use_sim_arg = DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                        description='Use simulation (Gazebo) clock if true')
    urdf_arg = DeclareLaunchArgument(name='urdf', default_value=urdf_file,
                                     description='Absolute path to urdf file')
    rviz_arg = DeclareLaunchArgument(name='rviz', default_value=rviz_file,
                                     description='Absolute path to rviz config file')

    # descriptions
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'robot_description': robot_description}],
        arguments=[LaunchConfiguration('urdf')]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz')],
    )

    return LaunchDescription([
        gui_arg,
        use_sim_arg,
        urdf_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
