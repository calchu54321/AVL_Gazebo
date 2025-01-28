from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    gazebo_world = '/path/to/your/world.sdf'
    urdf_file = '/path/to/your/robot.urdf'
    
    # Load robot description from URDF file
    with open(urdf_file, 'r') as urdf:
        robot_description = urdf.read()
    
    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', gazebo_world],
            output='screen'
        ),
        
        # Start robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/path/to/your/rviz_config_file.rviz']
        ),
        
        # Joint state publisher bridge (from Gazebo to robot_state_publisher)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0'],
            output='screen'
        )
    ])
