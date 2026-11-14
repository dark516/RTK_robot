import math
from types import SimpleNamespace

from frob_city_navigation.map_planner_node import MapPlannerNode
from frob_city_navigation.models import Pose2D, PoseSample


class _RotatedField:
    def observation_region(self, observation_id):
        assert observation_id == "OBS_B2_FROM_A2"
        points = [
            SimpleNamespace(x_m=0.30, y_m=1.10),
            SimpleNamespace(x_m=0.50, y_m=1.10),
            SimpleNamespace(x_m=0.50, y_m=1.30),
            SimpleNamespace(x_m=0.30, y_m=1.30),
        ]
        return SimpleNamespace(
            polygon=SimpleNamespace(points=points),
            nominal_heading_rad=0.0,
            heading_tolerance_rad=0.35,
        )

    def map_to_field_pose(self, pose):
        # The laboratory overlay maps field (0.4, 1.2, 0) to approximately
        # map (0.8298, 0.0171, -pi/2).
        assert math.isclose(pose.x_m, 0.8298055284, abs_tol=1e-6)
        return Pose2D(x_m=0.4, y_m=1.2, yaw_rad=0.0)


def test_observation_zone_converts_map_pose_to_field_frame():
    node = object.__new__(MapPlannerNode)
    node._field = _RotatedField()
    node._config = SimpleNamespace(
        navigation=SimpleNamespace(
            stationary_linear_mps=0.01,
            stationary_angular_radps=0.03,
        )
    )
    sample = PoseSample(
        timestamp_s=1.0,
        pose=Pose2D(
            x_m=0.8298055284,
            y_m=0.0171449951,
            yaw_rad=-math.pi / 2.0,
        ),
        linear_speed_mps=0.0,
        angular_speed_radps=0.0,
    )

    assert node._is_in_observation_zone("OBS_B2_FROM_A2", sample)
