import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
 
 
def generate_launch_description():
    pkg_share = get_package_share_directory('loop_phase_one_pkg')
    gz_sim_share = get_package_share_directory('ros_gz_sim')
 
    world_path = os.path.join(pkg_share, 'worlds', 'course.sdf')
    cube_model_path = os.path.join(pkg_share, 'models', 'red_cube', 'model.sdf')
 
    # Optional: `ros2 launch loop_phase_one_pkg sim.launch.py seed:=42`
    # 0 = new random waypoints every run
    seed_arg = DeclareLaunchArgument('seed', default_value='0',
                                     description='Waypoint random seed (0 = random)')
 
    # 1. Gazebo with GUI (-r starts the simulation unpaused)
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )
 
    # 2. Spawn the cube (center at z=0.25 so it sits on the ground)
    spawn_cube = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'red_cube', '-file', cube_model_path,
                   '-x', '0.0', '-y', '0.0', '-z', '0.25'],
        output='screen',
    )
 
    # 3. Bridge Gazebo <-> ROS 2
    #    ]  = ROS -> Gazebo      [  = Gazebo -> ROS
    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )
 
    # 4. Autonomous navigation node
    navigator = Node(
        package='loop_phase_one_pkg', executable='navigator.py',
        parameters=[{
            'use_sim_time': True,
            'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
        }],
        output='screen',
    )
 
    return LaunchDescription([
        seed_arg,
        launch_gazebo,
        spawn_cube,
        ros_gz_bridge,
        navigator,
    ])
