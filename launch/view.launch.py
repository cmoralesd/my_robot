import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node




def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_robot'))
    xacro_file = os.path.join(pkg_path,'description/my_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a node for robot_state_publisher
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a node for rviz2
    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_path, 'config', 'my_config.rviz')]]

    )

    # Create a node for join_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
        
    )

    # Create a node for my_robot_controller
    my_robot_controller_node = Node(
        package='my_robot',
        executable='my_robot_controller',
        name='my_robot_controller'
        
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation time if true'),

        robot_state_publisher_node,
        rviz2_node,
        #joint_state_publisher_gui_node,
        my_robot_controller_node

    ])