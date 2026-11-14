import pytest

from frob_city_web.calibration_model import new_calibration
from frob_city_web.calibration_store import (
    ActivationRejected,
    CalibrationStore,
    RevisionConflict,
)


def draft(name="x"):
    return new_calibration(name, {"kind": "file", "id": "map", "fingerprint": "a" * 64})


def test_create_and_update_use_server_ids_and_monotonic_revisions(tmp_path):
    store = CalibrationStore(tmp_path)

    created = store.create(draft("first"))
    replacement = {**created, "name": "renamed"}
    updated = store.update(created["id"], created["revision"], replacement)

    assert len(created["id"]) == 32
    assert created["revision"] == 1
    assert updated["id"] == created["id"]
    assert updated["revision"] == 2
    assert store.get(created["id"])["name"] == "renamed"


def test_update_requires_current_revision(tmp_path):
    store = CalibrationStore(tmp_path)
    created = store.create(draft())
    store.update(created["id"], created["revision"], created)

    with pytest.raises(RevisionConflict):
        store.update(created["id"], created["revision"], created)


def test_invalid_identifier_cannot_escape_drafts_directory(tmp_path):
    store = CalibrationStore(tmp_path)

    with pytest.raises(KeyError):
        store.get("../../outside")


def test_activation_rejects_validation_errors_or_readiness_blockers(tmp_path):
    store = CalibrationStore(tmp_path)
    created = store.create(draft())

    with pytest.raises(ActivationRejected):
        store.activate(created["id"], created["revision"], {
            "valid": False, "activation_ready": False, "errors": [{"code": "BAD"}], "blockers": []
        })
    with pytest.raises(ActivationRejected):
        store.activate(created["id"], created["revision"], {
            "valid": True, "activation_ready": False, "errors": [], "blockers": [{"code": "MISSING"}]
        })


def test_activation_backs_up_previous_active_and_keeps_it_on_replace_failure(tmp_path, monkeypatch):
    store = CalibrationStore(tmp_path)
    validation = {"valid": True, "activation_ready": True, "errors": [], "blockers": []}
    first = store.create(draft("first"))
    store.activate(first["id"], first["revision"], validation)
    second = store.create(draft("second"))

    def fail_replace(_document):
        raise OSError("disk")

    monkeypatch.setattr(store, "_replace_active", fail_replace)
    with pytest.raises(OSError, match="disk"):
        store.activate(second["id"], second["revision"], validation)

    assert store.active()["id"] == first["id"]
    assert list((tmp_path / "backups").glob("*.json"))


def test_atomic_write_does_not_leave_temporary_file(tmp_path):
    store = CalibrationStore(tmp_path)
    created = store.create(draft())

    assert store.get(created["id"])["id"] == created["id"]
    assert not list(tmp_path.rglob("*.tmp"))


def test_list_summaries_reports_active_draft(tmp_path):
    store = CalibrationStore(tmp_path)
    validation = {"valid": True, "activation_ready": True, "errors": [], "blockers": []}
    created = store.create(draft("cal"))
    store.activate(created["id"], created["revision"], validation)

    result = store.list_summaries()

    assert result["active"]["id"] == created["id"]
    assert result["drafts"][0]["name"] == "cal"


def test_activation_swaps_a_complete_artifact_bundle(tmp_path):
    store = CalibrationStore(tmp_path)
    validation = {"valid": True, "activation_ready": True, "errors": [], "blockers": []}
    created = store.create(draft("bundle"))

    store.activate(
        created["id"], created["revision"], validation,
        artifacts={"field.yaml": "schema_version: 2\n"},
    )

    assert store.active()["name"] == "bundle"
    assert (tmp_path / "active" / "current" / "field.yaml").read_text() == "schema_version: 2\n"
    assert (tmp_path / "active" / "field.yaml").read_text() == "schema_version: 2\n"
