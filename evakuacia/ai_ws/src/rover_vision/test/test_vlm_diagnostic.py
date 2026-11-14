import json

import pytest

from rover_vision.vlm_diagnostic import (
    make_schema,
    parse_vlm_response,
)


def response(payload):
    return {"choices": [{"message": {"content": json.dumps(payload)}}]}


def test_schema_requires_independent_qr_state():
    schema = make_schema()
    assert schema["properties"]["qr"]["enum"] == ["absent", "present", "unknown"]
    assert "qr" in schema["required"]


def test_parser_accepts_object_and_qr_as_independent_fields():
    value = parse_vlm_response(
        response({"event_type": "fallen_tree", "confidence": 0.8, "qr": "absent"}),
        make_schema(),
    )
    assert value["event_type"] == "fallen_tree"
    assert value["qr"] == "absent"


def test_parser_rejects_missing_or_invalid_qr_state():
    with pytest.raises(ValueError):
        parse_vlm_response(
            response({"event_type": "none", "confidence": 1.0}), make_schema()
        )
    with pytest.raises(ValueError):
        parse_vlm_response(
            response({"event_type": "none", "confidence": 1.0, "qr": "maybe"}),
            make_schema(),
        )
