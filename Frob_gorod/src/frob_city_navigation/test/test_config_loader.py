from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest
import yaml

from frob_city_navigation.config_loader import (
    ConfigValidationError,
    canonical_field_digest,
    load_project_config,
    validate_project_config,
)
from frob_city_navigation.models import ConfigPaths, SignClass


FIXTURE = Path(__file__).parent / "fixtures" / "valid"


def paths(root: Path = FIXTURE) -> ConfigPaths:
    return ConfigPaths(
        field=root / "field.yaml",
        event=root / "event.yaml",
        perception=root / "perception.yaml",
        navigation=root / "navigation.yaml",
    )


def load_yaml(name: str):
    return yaml.safe_load((FIXTURE / name).read_text())


def write_set(root: Path, *, field=None, event=None, perception=None, navigation=None):
    root.mkdir(parents=True, exist_ok=True)
    values = {
        "field.yaml": field if field is not None else load_yaml("field.yaml"),
        "event.yaml": event if event is not None else load_yaml("event.yaml"),
        "perception.yaml": perception if perception is not None else load_yaml("perception.yaml"),
        "navigation.yaml": navigation if navigation is not None else load_yaml("navigation.yaml"),
    }
    for name, value in values.items():
        (root / name).write_text(yaml.safe_dump(value, sort_keys=False))
    return paths(root)


def assert_invalid(config_paths: ConfigPaths, code: str):
    with pytest.raises(ConfigValidationError) as exc:
        load_project_config(config_paths)
    assert any(issue.code == code for issue in exc.value.issues), exc.value.issues


def test_valid_fixture_loads_and_is_ros_independent():
    cfg = load_project_config(paths())
    assert cfg.contract_version.startswith("frob-city-v2-core-")
    assert cfg.field.width_m == 4.0
    assert dict(cfg.perception.label_map)["parking"] == SignClass.PARKING
    assert validate_project_config(cfg) == ()


def test_nan_is_rejected(tmp_path):
    field = load_yaml("field.yaml")
    field["field_to_map"]["x_m"] = math.nan
    assert_invalid(write_set(tmp_path, field=field), "NON_FINITE")


def test_duplicate_id_is_rejected(tmp_path):
    field = load_yaml("field.yaml")
    field["anchors"].append(copy.deepcopy(field["anchors"][0]))
    assert_invalid(write_set(tmp_path, field=field), "DUPLICATE_ID")


def test_bad_enum_is_rejected(tmp_path):
    field = load_yaml("field.yaml")
    field["edges"][0]["movement"] = "UTURN"
    assert_invalid(write_set(tmp_path, field=field), "BAD_ENUM")


def test_unknown_yaml_key_is_rejected(tmp_path):
    navigation = load_yaml("navigation.yaml")
    navigation["surprise_key"] = 123
    assert_invalid(write_set(tmp_path, navigation=navigation), "UNKNOWN_KEY")


def test_negative_timeout_is_rejected(tmp_path):
    navigation = load_yaml("navigation.yaml")
    navigation["timeouts"]["tf_lookup_s"] = -0.1
    assert_invalid(write_set(tmp_path, navigation=navigation), "RANGE")


def test_unsupported_label_mapping_is_rejected(tmp_path):
    perception = load_yaml("perception.yaml")
    perception["label_map"]["mystery"] = "FLY"
    assert_invalid(write_set(tmp_path, perception=perception), "BAD_ENUM")


def test_mapping_to_unknown_is_rejected(tmp_path):
    perception = load_yaml("perception.yaml")
    perception["label_map"]["mystery"] = "UNKNOWN"
    assert_invalid(write_set(tmp_path, perception=perception), "UNSUPPORTED_LABEL")


def test_malformed_polygon_is_rejected(tmp_path):
    field = load_yaml("field.yaml")
    field["footprint"] = [[0.0, 0.0], [1.0, 0.0]]
    assert_invalid(write_set(tmp_path, field=field), "MALFORMED_POLYGON")


def test_inconsistent_start_edge_is_rejected(tmp_path):
    event = load_yaml("event.yaml")
    event["start"]["cell_id"] = "C4_4"
    assert_invalid(write_set(tmp_path, event=event), "START_EDGE")


def test_confirmed_start_requires_complete_state(tmp_path):
    event = load_yaml("event.yaml")
    event["event_inputs_confirmed"] = True
    event["start"]["outgoing_edge_id"] = None
    assert_invalid(write_set(tmp_path, event=event), "START_STATE")


def test_deterministic_digest():
    cfg_a = load_project_config(paths())
    cfg_b = load_project_config(paths())
    digest_a = canonical_field_digest(cfg_a.field)
    digest_b = canonical_field_digest(cfg_b.field)
    assert digest_a == digest_b
    assert len(digest_a) == 64


def test_digest_changes_after_geometry_change(tmp_path):
    original = load_project_config(paths())
    field = load_yaml("field.yaml")
    field["anchors"][1]["pose"]["x_m"] = 1.25
    changed = load_project_config(write_set(tmp_path, field=field))
    assert canonical_field_digest(original.field) != canonical_field_digest(changed.field)


def test_missing_threshold_for_mapped_class_is_rejected(tmp_path):
    perception = load_yaml("perception.yaml")
    del perception["confidence_thresholds"]["PARKING"]
    assert_invalid(write_set(tmp_path, perception=perception), "THRESHOLD")


def test_bus_dwell_cannot_be_below_two_seconds(tmp_path):
    event = load_yaml("event.yaml")
    event["bus_stop_dwell_s"] = 1.99
    assert_invalid(write_set(tmp_path, event=event), "RULE")
