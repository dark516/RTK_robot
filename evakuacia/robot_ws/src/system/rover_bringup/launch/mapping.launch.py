from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('rover_bringup'))
    profile = LaunchConfiguration('profile')
    use_base = LaunchConfiguration('use_base')
    use_odometry = LaunchConfiguration('use_odometry')
    use_description = LaunchConfiguration('use_description')
    use_localization = LaunchConfiguration('use_localization')
    use_imu = LaunchConfiguration('use_imu')
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_vision = LaunchConfiguration('use_vision')
    use_display = LaunchConfiguration('use_display')
    use_led_strip = LaunchConfiguration('use_led_strip')
    use_octoliner = LaunchConfiguration('use_octoliner')
    use_waveshare_audio = LaunchConfiguration('use_waveshare_audio')
    use_web = LaunchConfiguration('use_web')
    use_rosboard = LaunchConfiguration('use_rosboard')
    use_agent = LaunchConfiguration('use_agent')
    use_fleet_bridge = LaunchConfiguration('use_fleet_bridge')
    use_twist_mux = LaunchConfiguration('use_twist_mux')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_slam = LaunchConfiguration('use_slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    discovery_mode = LaunchConfiguration('discovery_mode')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rover_bringup'), 'launch', 'robot.launch.py'
        ])),
        launch_arguments={
            'profile': profile,
            'use_base': use_base,
            'use_odometry': use_odometry,
            'use_description': use_description,
            'use_localization': use_localization,
            'use_imu': use_imu,
            'use_lidar': use_lidar,
            'use_camera': use_camera,
            'use_vision': use_vision,
            'use_display': use_display,
            'use_led_strip': use_led_strip,
            'use_octoliner': use_octoliner,
            'use_waveshare_audio': use_waveshare_audio,
            'use_web': use_web,
            'use_rosboard': use_rosboard,
            'use_agent': use_agent,
            'use_fleet_bridge': use_fleet_bridge,
            'use_twist_mux': use_twist_mux,
            'use_nav2': use_nav2,
            'use_slam': use_slam,
            'use_sim_time': use_sim_time,
            'discovery_mode': discovery_mode,
            'use_rviz': use_rviz,
            'slam_params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='mapping'),
        DeclareLaunchArgument('use_base', default_value=''),
        DeclareLaunchArgument('use_odometry', default_value=''),
        DeclareLaunchArgument('use_description', default_value=''),
        DeclareLaunchArgument('use_localization', default_value=''),
        DeclareLaunchArgument('use_imu', default_value=''),
        DeclareLaunchArgument('use_lidar', default_value=''),
        DeclareLaunchArgument('use_camera', default_value=''),
        DeclareLaunchArgument('use_vision', default_value=''),
        DeclareLaunchArgument('use_display', default_value=''),
        DeclareLaunchArgument('use_led_strip', default_value=''),
        DeclareLaunchArgument('use_octoliner', default_value=''),
        DeclareLaunchArgument('use_waveshare_audio', default_value=''),
        DeclareLaunchArgument('use_web', default_value=''),
        DeclareLaunchArgument('use_rosboard', default_value=''),
        DeclareLaunchArgument('use_agent', default_value=''),
        DeclareLaunchArgument('use_fleet_bridge', default_value=''),
        DeclareLaunchArgument('use_twist_mux', default_value=''),
        DeclareLaunchArgument('use_nav2', default_value=''),
        DeclareLaunchArgument('use_slam', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('discovery_mode', default_value='configured'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(
                bringup_share / 'config' / 'navigation' / 'slam_toolbox_params.yaml'
            ),
        ),
        robot_launch,
    ])
