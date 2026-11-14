from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "launch" / "city_robot_test.launch.py").read_text(
    encoding="utf-8"
)


def test_integrated_launch_starts_core_and_web_without_motion_bypass():
    assert 'city_autonomy.launch.py' in SOURCE
    assert 'package="frob_city_web"' in SOURCE
    assert '"control_service": _city(namespace, "control_mission")' in SOURCE
    assert 'create_publisher' not in SOURCE
    assert 'NavigateToPose' not in SOURCE


def test_hardware_includes_are_opt_in():
    for argument in (
        "include_robot_bringup",
        "include_localization",
        "include_nav2",
        "include_detector",
    ):
        declaration = f'DeclareLaunchArgument("{argument}", default_value="false")'
        assert declaration in SOURCE

    assert 'DeclareLaunchArgument("include_detection_adapter", default_value="true")' in SOURCE
