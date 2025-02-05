import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    # xacro_file = os.path.join(pkg_path,'description','new_car_master.urdf.xacro')
    xacro_file = os.path.join(pkg_path,'description','building_robot.urdf')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    # robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    # Process Xacro
    robot_description_raw = xacro.process_file(xacro_file, mappings={
        "use_ros2_control": "true",
        "sim_mode": "false"
    }).toxml()

    # Define node parameters
    params = {
        'robot_description': ParameterValue(robot_description_raw, value_type=str),
        'use_sim_time': use_sim_time
    }

    # Create a robot_state_publisher node
    # params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher
    ])