import json
import math
from types import SimpleNamespace as NS

import pytest

from frob_city_web.serializers import occupancy_json


def grid(data, width=3, height=2, yaw=0.25):
    return NS(
        header=NS(stamp=NS(sec=2, nanosec=0), frame_id="map"),
        info=NS(
            width=width,
            height=height,
            resolution=0.05,
            map_load_time=NS(sec=1, nanosec=0),
            origin=NS(
                position=NS(x=-1.0, y=-2.0, z=0.0),
                orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        ),
        data=data,
    )


def test_occupancy_grid_is_run_length_encoded():
    result = occupancy_json(grid([-1, -1, 0, 0, 0, 100]))
    assert result["width"] == 3
    assert result["height"] == 2
    assert result["rle"] == [[-1, 2], [0, 3], [100, 1]]
    assert result["origin"]["yaw_rad"] == pytest.approx(0.25)
    json.dumps(result, allow_nan=False)


def test_occupancy_grid_rejects_bad_dimensions_or_values():
    with pytest.raises(ValueError, match="data length"):
        occupancy_json(grid([0], width=2, height=2))
    with pytest.raises(ValueError, match="range"):
        occupancy_json(grid([101], width=1, height=1))
