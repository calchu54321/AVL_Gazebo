#WORKING
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time') 
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    robot_description_path = get_package_share_directory('robot_description') 
     
    world_file_name = 'building_robot_debug.sdf' 
    world_path = os.path.join(robot_description_path, 'worlds', world_file_name) 

    if not os.path.exists(world_path): 
        raise FileNotFoundError(f"World file not found :{world_path}") 

    pkg_share = get_package_share_directory('robot_description')
    default_model_path = os.path.join(pkg_share, 'description', 'building_robot.urdf.xacro') #xacro
    robot_description_config = Command(['xacro ', default_model_path, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])


    rviz_config_file = '/home/ubuntu/AGV_ws/src/robot_description/rviz/robot_description.rviz'

    urdf_file = '/home/ubuntu/AGV_ws/src/robot_description/urdf/building_robot.urdf'
    with open(urdf_file) as infp: 
        robot_dec = infp.read()

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(robot_description_path, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )


    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', str(world_path)], 
            output='screen' 
        ), 

        # Argument to load the URDF model
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        node_robot_state_publisher,

        # Node to visualize the robot in rviz
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),
        # Joint State Publisher GUI node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ), 
        Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher', 
            output='screen', 
            parameters=[{'robot_description': robot_dec}], 
            arguments=[urdf_file]
        ),
        bridge
            

    ]) 
