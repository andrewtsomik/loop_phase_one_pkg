
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
 
 
def generate_launch_description():
    pkg = get_package_share_directory('loop_phase_one_pkg')
    world = os.path.join(pkg, 'worlds', 'course.sdf')
    cube = os.path.join(pkg, 'models', 'red_cube', 'model.sdf')
 
    return LaunchDescription([
        # Gazebo
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen'),
 
        # Put the cube in the world
        Node(package='ros_gz_sim', executable='create', arguments=['-name', 'red_cube', '-file', cube, '-z', '0.25']),
 
        # Connect Gazebo topics to ROS (] = ROS->Gazebo, [ = Gazebo->ROS)
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
 
        # The driving logic
        Node(package='loop_phase_one_pkg', executable='navigator.py', parameters=[{'use_sim_time': True}], output='screen')
    ])
