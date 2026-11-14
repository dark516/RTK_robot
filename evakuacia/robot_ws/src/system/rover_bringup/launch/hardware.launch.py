from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    empty_default = ''
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
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='hardware'),
        DeclareLaunchArgument('use_base', default_value=empty_default),
        DeclareLaunchArgument('use_odometry', default_value=empty_default),
        DeclareLaunchArgument('use_description', default_value=empty_default),
        DeclareLaunchArgument('use_localization', default_value=empty_default),
        DeclareLaunchArgument('use_imu', default_value=empty_default),
        DeclareLaunchArgument('use_lidar', default_value=empty_default),
        DeclareLaunchArgument('use_camera', default_value=empty_default),
        DeclareLaunchArgument('use_vision', default_value=empty_default),
        DeclareLaunchArgument('use_display', default_value=empty_default),
        DeclareLaunchArgument('use_led_strip', default_value=empty_default),
        DeclareLaunchArgument('use_octoliner', default_value=empty_default),
        DeclareLaunchArgument('use_waveshare_audio', default_value=empty_default),
        DeclareLaunchArgument('use_web', default_value=empty_default),
        DeclareLaunchArgument('use_rosboard', default_value=empty_default),
        DeclareLaunchArgument('use_agent', default_value=empty_default),
        DeclareLaunchArgument('use_fleet_bridge', default_value=empty_default),
        DeclareLaunchArgument('use_twist_mux', default_value=empty_default),
        IncludeLaunchDescription(
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
            }.items(),
        ),
    ])
