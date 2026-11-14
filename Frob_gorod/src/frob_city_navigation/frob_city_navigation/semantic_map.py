"""Authoritative ROS-independent semantic map for Frob City.

This module owns runtime world facts and semantic dependency versions.  It
consumes already-fused :class:`EvidenceEvent` objects; it never imports detector
messages, ROS, Nav2, or mission state.
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Dict, Iterable, Optional, Tuple
from uuid import uuid4

from .field_model import FieldModel
from .pose_projection import construct_service_geometry
from .geometry import pose_at_station, project_point_to_path

from .models import (
    ApproachRecord,
    ApproachResolution,
    CoverageRecord,
    CoverageResolution,
    DependencyCheck,
    EdgeBlockState,
    EdgeRecord,
    EvidenceEvent,
    EvidenceKind,
    ExecutionEvent,
    ExecutionEventKind,
    FieldConfig,
    MapSnapshot,
    MapUpdate,
    MovementMask,
    Pose2D,
    SignClass,
    KnownSignRecord,
    TargetRecord,
    TargetResolution,
    TrafficPolicy,
)
from .traffic_rules import allowed_movements


_DIRECTIONAL = frozenset(
    {
        SignClass.STRAIGHT_ONLY,
        SignClass.LEFT_ONLY,
        SignClass.RIGHT_ONLY,
        SignClass.NO_LEFT,
        SignClass.NO_RIGHT,
    }
)
_TARGET_CLASSES = frozenset({SignClass.BUS_STOP, SignClass.PARKING})


def _dep(prefix: str, entity_id: str) -> str:
    return f"{prefix}:{entity_id}"


class SemanticMap:
    """Mutable owner with immutable snapshots at its public boundary.

    `map_instance_id` is created once per process/object. `begin_run()` clears
    runtime semantics while retaining the validated static field model.
    """

    def __init__(self, field: FieldConfig, policy: TrafficPolicy | None = None):
        self._field = field
        self._field_model = FieldModel.from_config(field)
        self._policy = policy or TrafficPolicy()
        self._map_instance_id = str(uuid4())

        self._run_id = ""
        self._field_id = ""
        self._revision = 0
        self._ready = False
        self._perception_healthy = True
        self._localization_healthy = True
        self._reason = "not initialized"

        self._approaches: Dict[str, ApproachRecord] = {}
        self._targets: Dict[str, TargetRecord] = {}
        self._known_signs: Dict[str, KnownSignRecord] = {}
        self._edges: Dict[str, EdgeRecord] = {}
        self._coverage: Dict[str, CoverageRecord] = {}
        self._approach_signs: Dict[str, frozenset[SignClass]] = {}
        self._holds: set[str] = set()

        self._seen_evidence_ids: set[str] = set()
        self._seen_execution_keys: set[tuple[str, int]] = set()

        self._available_by_approach = self._build_available_movement_masks()
        self._site_faces = self._build_site_face_index()
        self._approach_edges = self._build_approach_edge_index()

    @property
    def map_instance_id(self) -> str:
        return self._map_instance_id

    def begin_run(self, run_id: str, field_id: str) -> MapSnapshot:
        if not run_id:
            raise ValueError("run_id must be non-empty")
        if not field_id:
            raise ValueError("field_id must be non-empty")

        self._run_id = run_id
        self._field_id = field_id
        self._revision = 0
        self._ready = True
        self._perception_healthy = True
        self._localization_healthy = True
        self._reason = ""

        self._approaches = {
            approach_id: ApproachRecord(
                approach_id=approach_id,
                version=0,
                state=ApproachResolution.UNKNOWN,
                confirmed_signs=frozenset(),
                allowed_movements=MovementMask.NONE,
                last_qualified_observation_s=None,
                precautionary_hold=False,
                reason="",
            )
            for approach_id in sorted(self._available_by_approach)
        }
        self._approach_signs = {
            approach_id: frozenset() for approach_id in self._approaches
        }
        self._targets = {}
        self._known_signs = {}
        self._edges = {
            edge.edge_id: EdgeRecord(
                edge_id=edge.edge_id,
                version=0,
                state=EdgeBlockState.OPEN,
                completed_traversals=0,
                next_recheck_after_s=None,
                reason="",
            )
            for edge in sorted(self._field.edges, key=lambda item: item.edge_id)
        }
        self._coverage = {
            region.observation_id: CoverageRecord(
                coverage_id=region.observation_id,
                version=0,
                state=CoverageResolution.UNKNOWN,
                failed_attempts=0,
                last_qualified_observation_s=None,
                reason="",
            )
            for region in sorted(
                self._field.observation_regions,
                key=lambda item: item.observation_id,
            )
        }
        self._holds.clear()
        self._seen_evidence_ids.clear()
        self._seen_execution_keys.clear()
        return self.snapshot()

    def apply(self, event: EvidenceEvent | ExecutionEvent) -> MapUpdate:
        if not self._ready:
            return self._update(False, False, False, (), "map not initialized")
        if event.run_id != self._run_id:
            return self._update(False, False, False, (), "stale run_id")

        if isinstance(event, EvidenceEvent):
            if not event.event_id:
                return self._update(False, False, False, (), "empty evidence event_id")
            if event.event_id in self._seen_evidence_ids:
                return self._update(True, True, False, (), "duplicate evidence event")
            update = self._apply_evidence(event)
            if update.accepted:
                self._seen_evidence_ids.add(event.event_id)
            return update

        if isinstance(event, ExecutionEvent):
            return self.apply_execution_with_edges(event, ())

        return self._update(False, False, False, (), "unsupported event type")

    def apply_execution_with_edges(
        self, event: ExecutionEvent, traversed_edge_ids: tuple[str, ...]
    ) -> MapUpdate:
        """Apply a cached-segment execution report with resolved edge identity.

        WP07's frozen ``ExecutionEvent`` intentionally carries only the opaque
        segment id.  WP11 owns the immutable route cache, so it resolves the
        cached segment first and supplies the actually traversed edge ids here.
        Traversal counters are observational bookkeeping: they change the map
        snapshot/global revision, but not the edge dependency version because
        legal/executable meaning did not change.
        """
        if not self._ready:
            return self._update(False, False, False, (), "map not initialized")
        if event.run_id != self._run_id:
            return self._update(False, False, False, (), "stale run_id")
        key = (event.navigator_instance_id, event.report_sequence)
        if key in self._seen_execution_keys:
            return self._update(True, True, False, (), "duplicate execution event")
        if len(set(traversed_edge_ids)) != len(traversed_edge_ids):
            return self._update(False, False, False, (), "duplicate traversed edge id")
        unknown = [edge_id for edge_id in traversed_edge_ids if edge_id not in self._edges]
        if unknown:
            return self._update(False, False, False, (), f"unknown traversed edge: {unknown[0]}")

        update = self._apply_execution(event)
        if not update.accepted:
            return update

        changed_ids: list[str] = []
        if event.kind is ExecutionEventKind.SEGMENT_COMPLETED:
            for edge_id in traversed_edge_ids:
                record = self._edges[edge_id]
                self._edges[edge_id] = replace(
                    record, completed_traversals=record.completed_traversals + 1
                )
                changed_ids.append(_dep("edge", edge_id))
            if changed_ids:
                self._bump_revision()

        self._seen_execution_keys.add(key)
        return self._update(
            True,
            False,
            bool(changed_ids),
            tuple(changed_ids),
            update.reason,
        )

    def snapshot(self) -> MapSnapshot:
        return MapSnapshot(
            run_id=self._run_id,
            map_instance_id=self._map_instance_id,
            field_id=self._field_id,
            revision=self._revision,
            ready=self._ready,
            perception_healthy=self._perception_healthy,
            localization_healthy=self._localization_healthy,
            precautionary_hold_ids=tuple(sorted(self._holds)),
            approaches=tuple(self._approaches[key] for key in sorted(self._approaches)),
            targets=tuple(self._targets[key] for key in sorted(self._targets)),
            edges=tuple(self._edges[key] for key in sorted(self._edges)),
            coverage=tuple(self._coverage[key] for key in sorted(self._coverage)),
            reason=self._reason,
            known_signs=tuple(self._known_signs[key] for key in sorted(self._known_signs)),
        )

    def version_of(self, dependency_id: str) -> int | None:
        if ":" not in dependency_id:
            return None
        prefix, entity_id = dependency_id.split(":", 1)
        if prefix == "approach":
            record = self._approaches.get(entity_id)
        elif prefix == "target":
            record = self._targets.get(entity_id)
        elif prefix == "edge":
            record = self._edges.get(entity_id)
        elif prefix == "coverage":
            record = self._coverage.get(entity_id)
        else:
            return None
        return None if record is None else record.version

    def dependencies_match(
        self, ids: tuple[str, ...], versions: tuple[int, ...]
    ) -> DependencyCheck:
        if len(ids) != len(versions):
            return DependencyCheck(
                matches=False,
                mismatched_ids=(),
                unknown_ids=(),
                reason="dependency ids/versions length mismatch",
            )
        if len(set(ids)) != len(ids):
            return DependencyCheck(
                matches=False,
                mismatched_ids=(),
                unknown_ids=(),
                reason="duplicate dependency id",
            )

        mismatched = []
        unknown = []
        for dep_id, expected in zip(ids, versions):
            current = self.version_of(dep_id)
            if current is None:
                unknown.append(dep_id)
            elif current != expected:
                mismatched.append(dep_id)
        return DependencyCheck(
            matches=not mismatched and not unknown,
            mismatched_ids=tuple(mismatched),
            unknown_ids=tuple(unknown),
            reason="" if not mismatched and not unknown else "dependency mismatch",
        )

    # ------------------------------------------------------------------
    # Evidence handling
    # ------------------------------------------------------------------

    def _apply_evidence(self, event: EvidenceEvent) -> MapUpdate:
        if event.kind is EvidenceKind.PRECAUTIONARY_HOLD:
            hold_id = self._hold_id(event)
            if not hold_id:
                return self._update(False, False, False, (), "hold has no entity id")
            if hold_id in self._holds:
                return self._update(True, False, False, (), "hold already active")
            self._holds.add(hold_id)
            self._refresh_approach_hold_flag(event.approach_id)
            self._bump_revision()
            return self._update(True, False, True, (), "precautionary hold added")

        if event.kind is EvidenceKind.INCONCLUSIVE:
            changed_ids = self._apply_inconclusive(event)
            hold_removed = self._clear_related_holds(event)
            if changed_ids or hold_removed:
                self._bump_revision()
            return self._update(
                True,
                False,
                bool(changed_ids or hold_removed),
                tuple(changed_ids),
                event.reason or "inconclusive observation",
            )

        if event.kind is EvidenceKind.QUALIFIED_CLEAR:
            changed_ids = self._apply_qualified_clear(event)
            hold_removed = self._clear_related_holds(event)
            if changed_ids or hold_removed:
                self._bump_revision()
            return self._update(
                True,
                False,
                bool(changed_ids or hold_removed),
                tuple(changed_ids),
                event.reason or "qualified clear",
            )

        if event.kind is EvidenceKind.CONFLICT:
            changed_ids = self._apply_conflict(event)
            self._clear_related_holds(event)
            if changed_ids:
                self._bump_revision()
            return self._update(
                True,
                False,
                bool(changed_ids),
                tuple(changed_ids),
                event.reason or "conflicting evidence",
            )

        if event.kind is EvidenceKind.POSITIVE:
            changed_ids = self._apply_positive(event)
            if changed_ids is None:
                return self._update(False, False, False, (), "invalid positive evidence")
            hold_removed = self._clear_related_holds(event)
            if changed_ids or hold_removed:
                self._bump_revision()
            return self._update(
                True,
                False,
                bool(changed_ids or hold_removed),
                tuple(changed_ids),
                event.reason or "positive evidence applied",
            )

        return self._update(False, False, False, (), "unsupported evidence kind")

    def _apply_positive(self, event: EvidenceEvent) -> list[str] | None:
        if event.sign_class is SignClass.UNKNOWN:
            return None

        changed: list[str] = []
        placement_change = self._record_known_sign(event)
        if placement_change:
            changed.append(placement_change)

        if event.sign_class in _DIRECTIONAL:
            if event.approach_id not in self._approaches:
                # Exact physical placement is useful even before a calibrated
                # sign face is connected to route semantics.
                return changed if changed else None
            current_signs = self._approach_signs[event.approach_id]
            next_signs = frozenset((*current_signs, event.sign_class))
            self._approach_signs[event.approach_id] = next_signs

            record = self._approaches[event.approach_id]
            # POSITIVE reaches the semantic map only after temporal confirmation
            # and calibrated site/face association in SignFusion.  That evidence
            # is strong enough to apply the sign while the robot is moving; a
            # stopped Observe action remains required only to prove absence.
            decision = allowed_movements(
                self._available_by_approach[event.approach_id],
                next_signs,
                ApproachResolution.RESOLVED_SIGNED,
                self._policy,
            )
            next_state = (
                ApproachResolution.RESOLVED_SIGNED
                if decision.allowed != MovementMask.NONE
                else ApproachResolution.CONFLICT
            )
            next_allowed = decision.allowed
            next_last_qualified = (
                event.timestamp_s
                if event.observation_id
                else record.last_qualified_observation_s
            )
            next_reason = (
                decision.reason
                if event.observation_id
                else f"moving sign confirmed: {decision.reason}"
            )
            executable_changed = (
                record.state != next_state
                or record.confirmed_signs != next_signs
                or record.allowed_movements != next_allowed
            )
            next_record = replace(
                record,
                version=record.version + (1 if executable_changed else 0),
                state=next_state,
                confirmed_signs=next_signs,
                allowed_movements=next_allowed,
                last_qualified_observation_s=next_last_qualified,
                reason=next_reason,
            )
            self._approaches[event.approach_id] = next_record
            if executable_changed:
                changed.append(_dep("approach", event.approach_id))

        elif event.sign_class in _TARGET_CLASSES:
            target_change = self._apply_target_positive(event)
            if target_change is None and not changed:
                return None
            if target_change:
                changed.append(target_change)

        elif event.sign_class is SignClass.HAZARD_MARKER:
            hold_id = self._hold_id(event)
            if hold_id and hold_id not in self._holds:
                self._holds.add(hold_id)
                self._refresh_approach_hold_flag(event.approach_id)

        if event.observation_id:
            coverage_change = self._mark_coverage_complete(
                event.observation_id, event.timestamp_s, event.reason
            )
            if coverage_change:
                changed.append(coverage_change)

        return changed

    def _record_known_sign(self, event: EvidenceEvent) -> str:
        """Persist the exact calibrated fixture selected by sign fusion."""
        if not event.site_id:
            return ""
        site_id, separator, face_id = event.site_id.partition("/")
        if not separator or site_id not in self._site_faces:
            return ""
        if face_id not in self._site_faces[site_id]:
            return ""
        key = f"{site_id}/{face_id}/{int(event.sign_class)}"
        current = self._known_signs.get(key)
        if current is None:
            self._known_signs[key] = KnownSignRecord(
                site_id=site_id,
                face_id=face_id,
                approach_id=event.approach_id,
                sign_class=event.sign_class,
                version=1,
                last_seen_s=event.timestamp_s,
                confidence=event.confidence,
            )
            return _dep("known_sign", key)
        self._known_signs[key] = replace(
            current,
            last_seen_s=event.timestamp_s,
            confidence=max(current.confidence, event.confidence),
        )
        return ""

    def _apply_target_positive(self, event: EvidenceEvent) -> str | None:
        if not event.site_id:
            return None
        physical_site_id, separator, explicit_face_id = event.site_id.partition("/")
        if physical_site_id not in self._site_faces:
            return None
        face_id = (
            explicit_face_id
            if separator and explicit_face_id in self._site_faces[physical_site_id]
            else self._resolve_face_id(physical_site_id, event.approach_id)
        )
        if face_id is None:
            return None

        existing_at_site = [
            record for record in self._targets.values()
            if record.site_id == physical_site_id and record.face_id == face_id
        ]
        if existing_at_site:
            record = existing_at_site[0]
            if record.kind != event.sign_class:
                if record.state is not TargetResolution.CONFLICT:
                    self._targets[record.target_id] = replace(
                        record,
                        version=record.version + 1,
                        state=TargetResolution.CONFLICT,
                        last_seen_s=event.timestamp_s,
                    )
                    return _dep("target", record.target_id)
                self._targets[record.target_id] = replace(
                    record, last_seen_s=event.timestamp_s
                )
                return ""

            # Supporting evidence renews freshness but not semantic version.
            self._targets[record.target_id] = replace(
                record, last_seen_s=event.timestamp_s
            )
            return ""

        site = next(site for site in self._field.sign_sites if site.site_id == physical_site_id)
        face = next(face for face in site.faces if face.face_id == face_id)
        target_id = f"{event.sign_class.name}:{physical_site_id}:{face_id}"
        controlled_approach_id = event.approach_id or face.controlled_approach_id
        directed_road_id = self._unique_edge_for_approach(controlled_approach_id)
        site_model = self._field_model.sign_site(physical_site_id)
        service = construct_service_geometry(
            self._field_model,
            site_model,
            directed_road_id,
        )
        if not service.valid:
            directed_road_id = self._service_edge_for_face(
                site_model, face.yaw_rad
            )
            service = construct_service_geometry(
                self._field_model,
                site_model,
                directed_road_id,
            )
        sign_pose = Pose2D(site.position.x_m, site.position.y_m, face.yaw_rad)
        self._targets[target_id] = TargetRecord(
            target_id=target_id,
            version=1,
            kind=event.sign_class,
            state=TargetResolution.CONFIRMED,
            site_id=physical_site_id,
            face_id=face_id,
            directed_road_id=directed_road_id,
            service_anchor_id=f"service:{target_id}" if service.valid else "",
            sign_pose=sign_pose,
            geometry_valid=service.valid,
            service_pose=service.service_pose,
            terminal_limit=service.terminal_limit,
            xy_tolerance_m=0.05 if service.valid else 0.0,
            yaw_tolerance_rad=0.10 if service.valid else 0.0,
            front_clearance_min_m=service.front_extent_m if service.valid else 0.0,
            last_seen_s=event.timestamp_s,
        )
        return _dep("target", target_id)

    def _apply_qualified_clear(self, event: EvidenceEvent) -> list[str]:
        changed: list[str] = []
        if event.approach_id:
            record = self._approaches.get(event.approach_id)
            if record is None:
                return changed
            existing_signs = self._approach_signs[event.approach_id]
            next_state = (
                ApproachResolution.RESOLVED_SIGNED
                if existing_signs
                else ApproachResolution.RESOLVED_CLEAR
            )
            decision = allowed_movements(
                self._available_by_approach[event.approach_id],
                existing_signs,
                next_state,
                self._policy,
            )
            executable_changed = (
                record.state != next_state
                or record.allowed_movements != decision.allowed
                or record.confirmed_signs != existing_signs
            )
            self._approaches[event.approach_id] = replace(
                record,
                version=record.version + (1 if executable_changed else 0),
                state=next_state,
                confirmed_signs=existing_signs,
                allowed_movements=decision.allowed,
                last_qualified_observation_s=event.timestamp_s,
                reason=decision.reason,
            )
            if executable_changed:
                changed.append(_dep("approach", event.approach_id))

        if event.observation_id:
            coverage_change = self._mark_coverage_complete(
                event.observation_id, event.timestamp_s, event.reason
            )
            if coverage_change:
                changed.append(coverage_change)
        return changed

    def _apply_inconclusive(self, event: EvidenceEvent) -> list[str]:
        if not event.observation_id:
            return []
        record = self._coverage.get(event.observation_id)
        if record is None:
            return []
        self._coverage[event.observation_id] = replace(
            record,
            version=record.version + 1,
            state=CoverageResolution.INCONCLUSIVE,
            failed_attempts=record.failed_attempts + 1,
            reason=event.reason or "inconclusive",
        )
        return [_dep("coverage", event.observation_id)]

    def _apply_conflict(self, event: EvidenceEvent) -> list[str]:
        changed: list[str] = []
        if event.approach_id and event.approach_id in self._approaches:
            record = self._approaches[event.approach_id]
            if record.state is not ApproachResolution.CONFLICT:
                self._approaches[event.approach_id] = replace(
                    record,
                    version=record.version + 1,
                    state=ApproachResolution.CONFLICT,
                    allowed_movements=MovementMask.NONE,
                    last_qualified_observation_s=event.timestamp_s,
                    reason=event.reason or "conflict",
                )
                changed.append(_dep("approach", event.approach_id))

        if event.site_id:
            for target_id, record in list(self._targets.items()):
                if record.site_id != event.site_id:
                    continue
                if record.state is not TargetResolution.CONFLICT:
                    self._targets[target_id] = replace(
                        record,
                        version=record.version + 1,
                        state=TargetResolution.CONFLICT,
                        last_seen_s=event.timestamp_s,
                    )
                    changed.append(_dep("target", target_id))

        if event.observation_id and event.observation_id in self._coverage:
            record = self._coverage[event.observation_id]
            self._coverage[event.observation_id] = replace(
                record,
                version=record.version + 1,
                state=CoverageResolution.INCONCLUSIVE,
                failed_attempts=record.failed_attempts + 1,
                reason=event.reason or "conflict",
            )
            changed.append(_dep("coverage", event.observation_id))
        return changed

    # ------------------------------------------------------------------
    # Execution handling
    # ------------------------------------------------------------------

    def _apply_execution(self, event: ExecutionEvent) -> MapUpdate:
        # WP07 owns idempotency and the edge records, but a bare ExecutionEvent
        # does not carry RouteSegment.traversed_edge_ids.  The future WP11 ROS
        # wrapper will resolve the immutable cached segment before producing
        # traversal updates.  We therefore accept/deduplicate the report here
        # without guessing an edge from the opaque segment_id.
        if event.report_sequence < 0 or not event.navigator_instance_id:
            return self._update(False, False, False, (), "invalid execution identity")
        if event.kind not in (
            ExecutionEventKind.SEGMENT_COMPLETED,
            ExecutionEventKind.SEGMENT_FAILED,
            ExecutionEventKind.SEGMENT_CANCELED,
        ):
            return self._update(False, False, False, (), "invalid execution kind")
        return self._update(True, False, False, (), "execution report accepted")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _mark_coverage_complete(
        self, observation_id: str, timestamp_s: float, reason: str
    ) -> str:
        record = self._coverage.get(observation_id)
        if record is None:
            return ""
        semantic_changed = record.state is not CoverageResolution.COMPLETE
        self._coverage[observation_id] = replace(
            record,
            version=record.version + (1 if semantic_changed else 0),
            state=CoverageResolution.COMPLETE,
            last_qualified_observation_s=timestamp_s,
            reason=reason,
        )
        return _dep("coverage", observation_id) if semantic_changed else ""

    def _hold_id(self, event: EvidenceEvent) -> str:
        if event.approach_id:
            return _dep("approach", event.approach_id)
        if event.site_id:
            return f"site:{event.site_id}"
        if event.observation_id:
            return _dep("coverage", event.observation_id)
        return ""

    def _clear_related_holds(self, event: EvidenceEvent) -> bool:
        candidates = set()
        if event.approach_id:
            candidates.add(_dep("approach", event.approach_id))
        if event.site_id:
            candidates.add(f"site:{event.site_id}")
        if event.observation_id:
            candidates.add(_dep("coverage", event.observation_id))
        before = len(self._holds)
        self._holds.difference_update(candidates)
        self._refresh_approach_hold_flag(event.approach_id)
        return len(self._holds) != before

    def _refresh_approach_hold_flag(self, approach_id: str) -> None:
        if not approach_id or approach_id not in self._approaches:
            return
        record = self._approaches[approach_id]
        active = _dep("approach", approach_id) in self._holds
        if record.precautionary_hold != active:
            self._approaches[approach_id] = replace(
                record, precautionary_hold=active
            )

    def _build_available_movement_masks(self) -> dict[str, MovementMask]:
        result: dict[str, MovementMask] = {}
        for edge in self._field.edges:
            if not edge.approach_id:
                continue
            result[edge.approach_id] = (
                result.get(edge.approach_id, MovementMask.NONE) | edge.movement
            )
        return result

    def _build_site_face_index(self) -> dict[str, tuple[str, ...]]:
        return {
            site.site_id: tuple(face.face_id for face in site.faces)
            for site in self._field.sign_sites
        }

    def _build_approach_edge_index(self) -> dict[str, tuple[str, ...]]:
        result: dict[str, list[str]] = {}
        for edge in self._field.edges:
            if edge.approach_id:
                result.setdefault(edge.approach_id, []).append(edge.edge_id)
        return {key: tuple(sorted(value)) for key, value in result.items()}

    def _resolve_face_id(self, site_id: str, approach_id: str) -> Optional[str]:
        site = next(site for site in self._field.sign_sites if site.site_id == site_id)
        if approach_id:
            matches = [
                face.face_id for face in site.faces
                if face.controlled_approach_id == approach_id
            ]
            if len(matches) == 1:
                return matches[0]
        if len(site.faces) == 1:
            return site.faces[0].face_id
        return None

    def _unique_edge_for_approach(self, approach_id: str) -> str:
        edges = self._approach_edges.get(approach_id, ())
        return edges[0] if len(edges) == 1 else ""

    def _service_edge_for_face(self, site, face_yaw_rad: float) -> str:
        """Select a surveyed forward path that passes the visible sign face.

        A target may be detected on either half of a road cell, whereas an
        atomic junction approach starts at the road midpoint.  Selecting from
        all directed primitives avoids invalid service geometry for signs just
        behind that midpoint while retaining the required right-roadside and
        face-direction checks.
        """
        desired_yaw = face_yaw_rad - math.pi / 2.0
        front_extent = max(point.x_m for point in self._field_model.footprint.points)
        candidates = []
        for movement_id in self._field_model.movement_ids:
            movement = self._field_model.movement_primitive(movement_id)
            projection = project_point_to_path(site.position, movement.path)
            if projection.station_m + 1e-9 < front_extent:
                continue
            path_pose = pose_at_station(movement.path, projection.station_m)
            heading_error = abs(math.atan2(
                math.sin(path_pose.yaw_rad - desired_yaw),
                math.cos(path_pose.yaw_rad - desired_yaw),
            ))
            if heading_error > 0.30 or projection.distance_m > 0.80:
                continue
            dx = site.position.x_m - path_pose.x_m
            dy = site.position.y_m - path_pose.y_m
            left_m = -math.sin(path_pose.yaw_rad) * dx + math.cos(path_pose.yaw_rad) * dy
            if left_m >= -0.10:
                continue
            movement_rank = 0 if movement.movement is MovementMask.STRAIGHT else 1
            candidates.append((projection.distance_m, movement_rank, movement_id))
        return min(candidates)[2] if candidates else ""

    def _bump_revision(self) -> None:
        self._revision += 1

    def _update(
        self,
        accepted: bool,
        duplicate: bool,
        changed: bool,
        changed_dependency_ids: tuple[str, ...],
        reason: str,
    ) -> MapUpdate:
        return MapUpdate(
            accepted=accepted,
            duplicate=duplicate,
            changed=changed,
            revision=self._revision,
            changed_dependency_ids=tuple(sorted(set(changed_dependency_ids))),
            reason=reason,
        )
