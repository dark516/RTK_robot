from pathlib import Path


ROOT = Path(__file__).parents[2]
ROBOT = (ROOT / 'frob_city_web' / 'launch' / 'city_competition_robot.launch.py').read_text()
OFFBOARD = (ROOT / 'traffic_sign_detector' / 'launch' / 'city_competition_offboard.launch.py').read_text()


def test_robot_launch_uses_competition_map_profiles_and_guarded_command_chain():
    assert 'andrey_map.yaml' in ROBOT
    assert 'andrey_competition_field.yaml' in ROBOT
    assert 'andrey_competition_motion_guard.yaml' in ROBOT
    assert "'odom_topic': '/odometry/filtered'" in ROBOT
    assert "'include_nav2': 'false'" in ROBOT
    assert 'city_robot_test.launch.py' in ROBOT
    assert "'node_names': ['controller_server', 'velocity_smoother', 'collision_monitor']" in ROBOT
    assert 'bt_navigator' not in ROBOT
    assert 'opennav_docking' not in ROBOT
    assert 'ControlMission' not in ROBOT  # launch cannot auto-START the attempt


def test_offboard_launch_has_one_distinct_detector_output_and_tcp_relay():
    assert '/traffic_sign/detections_offboard' in OFFBOARD
    assert 'traffic_sign_detection_tcp_server' in OFFBOARD
    assert 'ultralytics_vendor_path' in OFFBOARD
    assert 'rqt' not in OFFBOARD.lower()
