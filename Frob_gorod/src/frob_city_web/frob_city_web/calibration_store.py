"""Atomic versioned storage for calibration drafts and active state."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import tempfile
import threading
import uuid
from typing import Any, Mapping


_ID = re.compile(r"^[0-9a-f]{32}$")


class RevisionConflict(RuntimeError):
    """The caller attempted to overwrite a newer draft revision."""


class ActivationRejected(RuntimeError):
    """The draft is not valid and physically ready for activation."""


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class CalibrationStore:
    def __init__(self, root: Path | str):
        self.root = Path(root)
        self.drafts_dir = self.root / "drafts"
        self.active_dir = self.root / "active"
        self.backups_dir = self.root / "backups"
        for directory in (self.drafts_dir, self.active_dir, self.backups_dir):
            directory.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()

    @staticmethod
    def _valid_id(draft_id: str) -> str:
        value = str(draft_id)
        if not _ID.fullmatch(value):
            raise KeyError("unknown calibration draft")
        return value

    def _draft_path(self, draft_id: str) -> Path:
        return self.drafts_dir / f"{self._valid_id(draft_id)}.json"

    @staticmethod
    def _read(path: Path) -> dict[str, Any]:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise KeyError("unknown calibration draft") from exc
        if not isinstance(value, dict):
            raise ValueError(f"invalid calibration file: {path}")
        return value

    def _atomic_write(self, path: Path, document: Mapping[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        handle, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
        try:
            with os.fdopen(handle, "w", encoding="utf-8") as stream:
                json.dump(document, stream, ensure_ascii=False, allow_nan=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_name, path)
        except BaseException:
            try:
                Path(temp_name).unlink(missing_ok=True)
            finally:
                raise

    def create(self, document: Mapping[str, Any]) -> dict[str, Any]:
        with self._lock:
            result = deepcopy(dict(document))
            now = _now_iso()
            result["id"] = uuid.uuid4().hex
            result["revision"] = 1
            result["lifecycle"] = "draft"
            result["created_at"] = now
            result["updated_at"] = now
            self._atomic_write(self._draft_path(result["id"]), result)
            return deepcopy(result)

    def get(self, draft_id: str) -> dict[str, Any]:
        with self._lock:
            return deepcopy(self._read(self._draft_path(draft_id)))

    def update(
        self,
        draft_id: str,
        revision: int,
        document: Mapping[str, Any],
    ) -> dict[str, Any]:
        with self._lock:
            current = self._read(self._draft_path(draft_id))
            if int(revision) != int(current.get("revision", 0)):
                raise RevisionConflict("draft revision is stale")
            result = deepcopy(dict(document))
            result["id"] = current["id"]
            result["revision"] = int(current["revision"]) + 1
            result["lifecycle"] = "draft"
            result["created_at"] = current["created_at"]
            result["updated_at"] = _now_iso()
            self._atomic_write(self._draft_path(draft_id), result)
            return deepcopy(result)

    def active(self) -> dict[str, Any] | None:
        current_path = self.active_dir / "current" / "calibration.json"
        legacy_path = self.active_dir / "calibration.json"
        with self._lock:
            path = current_path if current_path.exists() else legacy_path
            if not path.exists():
                return None
            return deepcopy(self._read(path))

    def _replace_active(
        self,
        document: Mapping[str, Any],
        artifacts: Mapping[str, str | bytes] | None = None,
    ) -> None:
        revision_name = f"{document['id']}_{document['revision']}_{uuid.uuid4().hex[:8]}"
        snapshot_dir = self.active_dir / "revisions" / revision_name
        snapshot_dir.mkdir(parents=True, exist_ok=False)
        self._atomic_write(snapshot_dir / "calibration.json", document)
        for name, content in (artifacts or {}).items():
            if Path(name).name != name:
                raise ValueError("active artifact name must not contain a path")
            if isinstance(content, bytes):
                text = content.decode("utf-8")
            else:
                text = str(content)
            path = snapshot_dir / name
            handle, temp_name = tempfile.mkstemp(prefix=f".{name}.", suffix=".tmp", dir=snapshot_dir)
            try:
                with os.fdopen(handle, "w", encoding="utf-8") as stream:
                    stream.write(text)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temp_name, path)
            except BaseException:
                Path(temp_name).unlink(missing_ok=True)
                raise

        pointer_temp = self.active_dir / f".current.{uuid.uuid4().hex}.tmp"
        os.symlink(Path("revisions") / revision_name, pointer_temp)
        os.replace(pointer_temp, self.active_dir / "current")
        aliases = ["calibration.json", *(artifacts or {}).keys()]
        for name in aliases:
            alias_temp = self.active_dir / f".{name}.{uuid.uuid4().hex}.tmp"
            os.symlink(Path("current") / name, alias_temp)
            os.replace(alias_temp, self.active_dir / name)

    def activate(
        self,
        draft_id: str,
        revision: int,
        validation: Mapping[str, Any],
        artifacts: Mapping[str, str | bytes] | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if not validation.get("valid") or not validation.get("activation_ready"):
                raise ActivationRejected("calibration has validation errors or readiness blockers")
            current = self._read(self._draft_path(draft_id))
            if int(revision) != int(current.get("revision", 0)):
                raise RevisionConflict("draft revision is stale")
            previous = self.active()
            if previous is not None:
                timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
                self._atomic_write(
                    self.backups_dir / f"{timestamp}_{previous['id']}.json",
                    previous,
                )
            activated = deepcopy(current)
            activated["lifecycle"] = "active"
            activated["validation"] = deepcopy(dict(validation))
            activated["activation"] = {"activated_at": _now_iso(), "revision": current["revision"]}
            if artifacts:
                self._replace_active(activated, artifacts)
            else:
                self._replace_active(activated)
            return deepcopy(activated)

    @staticmethod
    def _summary(document: Mapping[str, Any]) -> dict[str, Any]:
        return {
            "id": document.get("id"),
            "revision": document.get("revision"),
            "name": document.get("name"),
            "lifecycle": document.get("lifecycle", "draft"),
            "updated_at": document.get("updated_at"),
            "map_source": document.get("map_source"),
        }

    def list_summaries(self) -> dict[str, Any]:
        with self._lock:
            drafts = []
            for path in self.drafts_dir.glob("*.json"):
                try:
                    drafts.append(self._summary(self._read(path)))
                except (KeyError, ValueError, OSError, json.JSONDecodeError):
                    continue
            drafts.sort(key=lambda item: str(item.get("updated_at") or ""), reverse=True)
            active = self.active()
            return {"drafts": drafts, "active": self._summary(active) if active else None}
