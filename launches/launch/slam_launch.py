import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    navpkg = 'ngeeann_av_nav'
    gzpkg = 'ngeeann_av_gazebo'
    descpkg = 'ngeeann_av_description'
    mappkg = 'ngeeann_av_map'
    slampkg = 'sloam'

    world = os.path.join(get_package_share_directory(gzpkg), 'worlds', 'ngeeann_av.world')
    urdf = os.path.join(get_package_share_directory(descpkg),'urdf', 'ngeeann_av.urdf')
    rviz = os.path.join(get_package_share_directory(descpkg), 'rviz', 'lidar_view.rviz')
    ekf = os.path.join(get_package_share_directory(navpkg), 'config', 'ekf.yaml')

    slamconfig = os.path.join(get_package_share_directory(slampkg), 'config', 'ros_param.yaml')
    navconfig = os.path.join(get_package_share_directory(navpkg), 'config', 'navigation_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    subprocess.run(['killall', 'gzserver'])
    subprocess.run(['killall', 'gzclient'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),

        # ExecuteProcess(
        #     cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        # ),

        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world], output='screen'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),


        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output={'both': 'log'}
        ),

        Node(
            package = navpkg,
            name = 'localisation',
            executable = 'localisation.py',
            parameters = [navconfig]
        ),

        

        # Node(
        #     package = navpkg,
        #     name = 'global_planner',
        #     executable = 'globalplanner.py',
        #     parameters = [navconfig]
        # ),

        # Node(
        #     package = navpkg,
        #     name = 'local_planner',
        #     executable = 'localplanner.py',
        #     parameters = [navconfig]
        # ),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[ekf, {'use_sim_time': use_sim_time}]
        # ),


        Node(
            package = mappkg,
            name = 'bof_lidar',
            executable = 'bof_lidar',
        ),

        Node(
            package = slampkg,
            executable="frameFeature",
            parameters=[slamconfig]
        ),
    
        Node(
            package = slampkg,
            executable="lidarOdometry"
        ),


        # Node(
        #     package = navpkg,
        #     name = 'path_tracker',
        #     executable = 'tracker.py',
        #     parameters = [navconfig]
        # )
    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
