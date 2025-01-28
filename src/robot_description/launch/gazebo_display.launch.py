#BROKEN
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the robot_description package
    robot_description_path = get_package_share_directory('robot_description')
    
    # Specify the SDF world file
    world_file_name = 'building_robot_debug.sdf'  # Replace with your actual world file name
    world_path = os.path.join(robot_description_path, 'worlds', world_file_name)
    
    # Check if the world file exists
    if not os.path.exists(world_path):
        raise FileNotFoundError(f"World file not found: {world_path}")

    return LaunchDescription([
        # Launch Gazebo with the specified world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '--verbose', world_path],
            output='screen'
        )
    ])
