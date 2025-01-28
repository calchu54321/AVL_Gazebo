#launches both rviz and gazebo together but no sdf and urdf
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to SDF and URDF files
    pkg_share = get_package_share_directory('robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sdf_path = os.path.join(pkg_share, 'worlds', 'new_car_world.sdf')
    urdf_path = os.path.join(pkg_share, 'description', 'building_robot.urdf.xacro')
    
    # Path to RViz config file
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot_description.rviz')
    
    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'world': '/home/ubuntu/AGV_ws/src/robot_description/worlds/world.sdf'
        }.items()
    )
    
    # # Spawn model in Gazebo
    # spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='spawn_entity.py',
    #     arguments=['-file', sdf_path, '-entity', 'robot'],
    #     output='screen'
    # )
    
    # Robot State Publisher for RViz
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}],
        output='screen'
    )
    
    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        #spawn_entity,
        robot_state_publisher,
        rviz
    ])
