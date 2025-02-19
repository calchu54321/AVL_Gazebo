import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_name = 'robot_description' 
    ros_gz_sim = get_package_share_directory('ros_gz_sim') 

    #share directory to read stl file
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join('/home/ubuntu/AGV_ws/src')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','robot_state_publisher.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()

    )

    world = os.path.join(
        get_package_share_directory('robot_description'),
        'worlds',
        'empty_world.world'
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_description"),
            "config",
            "controllers.yaml",
        ]
    )

    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    ) 

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        launch_arguments={'gz_args': '-g -v4 '}.items()
    ) 

    gz_spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'car', '-allow_renaming', 'true',
                   '-x', '0.0', '-y', '0.0', '-z', '0.323'], 
    ) 

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad",]
    )

    robot_ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont", "--param-file", robot_controllers],
        # condition=UnlessCondition(remap_odometry_tf),
    )


    teleop_twist_keyboard = Node(
        package ='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/keyboard/cmd_vel', '/ack_cont/reference_unstamped')] #remap topic
    )

    return LaunchDescription([
        rsp,
        gazebo_server, 
        gazebo_client,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        robot_ackermann_controller_spawner,
        gz_spawn_entity,
        teleop_twist_keyboard,
    ]) 
