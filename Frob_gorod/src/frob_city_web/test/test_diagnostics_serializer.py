from types import SimpleNamespace

from frob_city_web.serializers import diagnostics_json


def test_diagnostic_uint8_bytes_are_serialized() -> None:
    status = SimpleNamespace(
        OK=b"\x00",
        WARN=b"\x01",
        ERROR=b"\x02",
        STALE=b"\x03",
        level=b"\x01",
        name="map planner",
        message="test",
        hardware_id="test",
        values=[],
    )
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=0), frame_id=""
        ),
        status=[status],
    )

    result = diagnostics_json(message)

    assert result["status"][0]["level"] == {"value": 1, "label": "WARN"}
