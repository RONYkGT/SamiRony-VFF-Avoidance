import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the turtlebot3_world.py launch file
    turtlebot3_gazebo_launch_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_world_launch_file = os.path.join(turtlebot3_gazebo_launch_dir, 'launch', 'turtlebot3_world.launch.py')

    # Define the path to the parameter file
    avoidance_node_config = os.path.join(
        get_package_share_directory('vff_avoidance'), 'config', 'AvoidanceNodeConfig.yaml'
    )
    
    return launch.LaunchDescription([
        # Include the turtlebot3_world.py launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_world_launch_file)
        ),
        # Launch the Robot Driver Node
        launch_ros.actions.Node(
            package='vff_avoidance',
            executable='avoidance_node',
            name='avoidance_node',
            output='screen',
            parameters=[avoidance_node_config]
        ),
    ])
