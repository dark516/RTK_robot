"""Application service joining map sources, validation, and calibration storage."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from typing import Any, Callable, Mapping

from .calibration_export import export_field_yaml
from .calibration_model import new_calibration
from .calibration_store import CalibrationStore, RevisionConflict
from .calibration_validation import validate_calibration
from .map_catalog import MapCatalog, MapCatalogError


class CalibrationRequestError(ValueError):
    """Raised when an API request does not match the calibration contract."""


class CalibrationService:
    def __init__(
        self,
        catalog: MapCatalog,
        store: CalibrationStore,
        live_map_snapshot: Callable[[], Mapping[str, Any] | None],
        live_map_max_age_s: float,
    ) -> None:
        self.catalog = catalog
        self.store = store
        self.live_map_snapshot = live_map_snapshot
        self.live_map_max_age_s = float(live_map_max_age_s)
        if self.live_map_max_age_s <= 0.0 or not math.isfinite(self.live_map_max_age_s):
            raise ValueError("live map stale timeout must be positive and finite")

    @staticmethod
    def _fingerprint(map_data: Mapping[str, Any]) -> str:
        content = {
            key: map_data[key]
            for key in ("width", "height", "resolution_m", "origin", "rle")
            if key in map_data
        }
        encoded = json.dumps(content, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return hashlib.sha256(encoded).hexdigest()

    def _live(self) -> tuple[dict[str, Any] | None, dict[str, Any]]:
        snapshot = self.live_map_snapshot()
        age = None if snapshot is None else snapshot.get("age_s")
        map_data = None if snapshot is None else snapshot.get("map")
        fresh = (
            isinstance(map_data, Mapping)
            and isinstance(age, (int, float))
            and math.isfinite(float(age))
            and 0.0 <= float(age) <= self.live_map_max_age_s
        )
        status = {
            "id": "live",
            "name": "/map",
            "available": bool(fresh),
            "stale": bool(map_data is not None and not fresh),
            "age_s": float(age) if isinstance(age, (int, float)) and math.isfinite(float(age)) else None,
            "max_age_s": self.live_map_max_age_s,
        }
        if not fresh:
            return None, status
        preview = deepcopy(dict(map_data))
        preview["source"] = {"kind": "live", "id": "live", "name": "/map", "path": None}
        preview["fingerprint"] = self._fingerprint(preview)
        status["fingerprint"] = preview["fingerprint"]
        return preview, status

    def maps(self) -> dict[str, Any]:
        _, live_status = self._live()
        return {"live": live_status, "files": self.catalog.list_maps()}

    def map_preview(self, map_id: str) -> dict[str, Any]:
        if map_id == "live":
            preview, _ = self._live()
            if preview is None:
                raise MapCatalogError("live map is unavailable or stale")
            return preview
        return self.catalog.preview(map_id)

    def _preview_for_source(self, source: Mapping[str, Any]) -> dict[str, Any]:
        source_id = source.get("id")
        kind = source.get("kind")
        if kind not in {"live", "file"} or not isinstance(source_id, str):
            raise CalibrationRequestError("map_source requires kind and id")
        if kind == "live" and source_id != "live":
            raise CalibrationRequestError("live map id must be 'live'")
        if kind == "file" and source_id == "live":
            raise CalibrationRequestError("file map id is invalid")
        return self.map_preview(source_id)

    def list_drafts(self) -> dict[str, Any]:
        return self.store.list_summaries()

    def get_draft(self, draft_id: str) -> dict[str, Any]:
        return self.store.get(draft_id)

    def create(self, body: Mapping[str, Any]) -> dict[str, Any]:
        name = body.get("name", "Новая калибровка")
        source = body.get("map_source")
        if not isinstance(name, str) or not isinstance(source, Mapping):
            raise CalibrationRequestError("name and map_source are required")
        preview = self._preview_for_source(source)
        map_source = deepcopy(preview["source"])
        map_source["fingerprint"] = preview["fingerprint"]
        document = new_calibration(name, map_source)
        origin = preview["origin"]
        yaw = float(origin.get("yaw_rad", 0.0))
        local_centre = [
            float(preview["width"]) * float(preview["resolution_m"]) / 2.0,
            float(preview["height"]) * float(preview["resolution_m"]) / 2.0,
        ]
        map_centre = [
            float(origin["x_m"]) + math.cos(yaw) * local_centre[0] - math.sin(yaw) * local_centre[1],
            float(origin["y_m"]) + math.sin(yaw) * local_centre[0] + math.cos(yaw) * local_centre[1],
        ]
        field_half = 2.0
        document["field"]["field_to_map"] = {
            "x_m": map_centre[0] - math.cos(yaw) * field_half + math.sin(yaw) * field_half,
            "y_m": map_centre[1] - math.sin(yaw) * field_half - math.cos(yaw) * field_half,
            "yaw_rad": yaw,
        }
        return self.store.create(document)

    def update(self, draft_id: str, body: Mapping[str, Any]) -> dict[str, Any]:
        revision = body.get("revision")
        document = body.get("document")
        if isinstance(revision, bool) or not isinstance(revision, int) or not isinstance(document, Mapping):
            raise CalibrationRequestError("integer revision and document are required")
        return self.store.update(draft_id, revision, document)

    def _document_from_body(self, body: Mapping[str, Any]) -> dict[str, Any]:
        supplied = body.get("document")
        if isinstance(supplied, Mapping):
            return deepcopy(dict(supplied))
        draft_id, revision = body.get("id"), body.get("revision")
        if not isinstance(draft_id, str) or isinstance(revision, bool) or not isinstance(revision, int):
            raise CalibrationRequestError("document or id with integer revision is required")
        document = self.store.get(draft_id)
        if int(document.get("revision", 0)) != revision:
            raise RevisionConflict("draft revision is stale")
        return document

    def validate(self, body: Mapping[str, Any]) -> dict[str, Any]:
        document = self._document_from_body(body)
        preview = self._preview_for_source(document.get("map_source", {}))
        return validate_calibration(document, preview)

    def activate(self, body: Mapping[str, Any]) -> dict[str, Any]:
        document = self._document_from_body(body)
        validation = self.validate({"document": document})
        if not validation.get("activation_ready"):
            return self.store.activate(document["id"], document["revision"], validation)
        field_yaml = export_field_yaml(document)
        return self.store.activate(
            document["id"], document["revision"], validation,
            artifacts={"field.yaml": field_yaml},
        )
