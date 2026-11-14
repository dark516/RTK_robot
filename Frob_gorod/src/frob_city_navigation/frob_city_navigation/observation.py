"""WP06 fail-closed observation-session core using frozen Agent-1 semantics.

The session consumes canonical ``EvidenceEvent`` / ``EvidenceKind`` objects from
``frob_city_navigation.models``.  It does not define a second private semantic
evidence enum.

Safety rule: unknown, unhealthy, ambiguous, stale, conflicting, or incomplete
observation never becomes QUALIFIED_CLEAR.
"""

from __future__ import annotations

from frob_city_navigation.models import PoseSample

from dataclasses import dataclass
import math
from typing import Optional

from .models import EvidenceEvent, EvidenceKind


@dataclass(frozen=True)
class ObservationFrame:
    run_id: str
    observation_id: str
    sequence: int
    capture_timestamp_s: float
    healthy: bool
    covered_sectors: frozenset[str]
    evidence: tuple[EvidenceEvent, ...] = ()
    validity_reason: str = 'ok'




@dataclass(frozen=True)
class ObservationConfig:
    required_distinct_frames: int = 8
    required_sectors: frozenset[str] = frozenset(
        {'left', 'center', 'right'}
    )
    min_hits_per_sector: int = 2
    min_temporal_span_s: float = 1.0
    max_capture_age_s: float = 0.75
    max_pose_timestamp_error_s: float = 1e-6


@dataclass(frozen=True)
class ObservationProgress:
    accepted: bool
    duplicate: bool
    reason: str
    distinct_healthy_frames: int
    temporal_span_s: float
    has_positive: bool
    has_conflict: bool
    has_hold_or_inconclusive: bool
    camera_failure: bool


@dataclass(frozen=True)
class ObservationOutcome:
    """Pure outcome using the frozen canonical EvidenceKind."""

    kind: EvidenceKind
    reason: str
    distinct_healthy_frames: int
    temporal_span_s: float
    site_ids: tuple[str, ...] = ()


class ObservationSession:
    def __init__(self, config: ObservationConfig | None = None) -> None:
        self._config = config or ObservationConfig()
        if self._config.required_distinct_frames < 1:
            raise ValueError('required_distinct_frames must be >= 1')
        if self._config.min_hits_per_sector < 1:
            raise ValueError('min_hits_per_sector must be >= 1')
        if self._config.min_temporal_span_s < 0:
            raise ValueError('min_temporal_span_s must be >= 0')
        if not self._config.required_sectors:
            raise ValueError('required_sectors must not be empty')

        self._run_id: Optional[str] = None
        self._observation_id: Optional[str] = None
        self._seen_sequences: set[int] = set()
        self._healthy_timestamps: dict[int, float] = {}
        self._sector_hits = {
            sector: 0
            for sector in sorted(self._config.required_sectors)
        }
        self._positive_sites: set[str] = set()
        self._has_conflict = False
        self._has_hold_or_inconclusive = False
        self._camera_failure = False
        self._latest_capture_timestamp_s: Optional[float] = None

    def begin(self, run_id: str, observation_id: str) -> None:
        if not run_id or not observation_id:
            raise ValueError('run_id and observation_id are required')
        self.__init__(self._config)
        self._run_id = run_id
        self._observation_id = observation_id

    def ingest(
        self,
        frame: ObservationFrame,
        pose: PoseSample,
        now: float,
    ) -> ObservationProgress:
        if self._run_id is None:
            self.begin(frame.run_id, frame.observation_id)

        if (
            frame.run_id != self._run_id
            or frame.observation_id != self._observation_id
        ):
            self._has_hold_or_inconclusive = True
            return self._progress(
                False,
                False,
                'observation_identity_mismatch',
            )

        if frame.sequence in self._seen_sequences:
            return self._progress(
                False,
                True,
                'duplicate_sequence',
            )
        self._seen_sequences.add(frame.sequence)

        error = self._validate(frame, pose, now)
        if error is not None:
            if error in {'camera_unhealthy', 'frame_invalid'}:
                self._camera_failure = True
            else:
                self._has_hold_or_inconclusive = True
            return self._progress(False, False, error)

        self._healthy_timestamps[
            frame.sequence
        ] = frame.capture_timestamp_s
        self._latest_capture_timestamp_s = frame.capture_timestamp_s

        for sector in frame.covered_sectors:
            if sector in self._sector_hits:
                self._sector_hits[sector] += 1

        for event in frame.evidence:
            self._consume_event(event)

        return self._progress(True, False, 'accepted')

    def finish(self, now: float) -> ObservationOutcome:
        if self._camera_failure:
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'camera_or_inference_failure_seen',
            )
        if self._has_conflict:
            return self._outcome(
                EvidenceKind.CONFLICT,
                'conflicting_evidence',
            )
        if self._positive_sites:
            return self._outcome(
                EvidenceKind.POSITIVE,
                'confirmed_face_observed',
                self._positive_sites,
            )
        if self._has_hold_or_inconclusive:
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'hold_or_inconclusive_input_seen',
            )
        if not self._healthy_timestamps:
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'no_valid_observation',
            )
        if not math.isfinite(float(now)):
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'finish_time_non_finite',
            )

        latest = self._latest_capture_timestamp_s
        assert latest is not None
        age = float(now) - latest
        if age < 0:
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'finish_before_latest_capture',
            )
        if age > self._config.max_capture_age_s:
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'observation_stale',
            )

        if (
            len(self._healthy_timestamps)
            < self._config.required_distinct_frames
        ):
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'insufficient_distinct_healthy_frames',
            )

        if self._temporal_span_s() < self._config.min_temporal_span_s:
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'insufficient_temporal_span',
            )

        incomplete = [
            sector
            for sector, hits in sorted(self._sector_hits.items())
            if hits < self._config.min_hits_per_sector
        ]
        if incomplete:
            return self._outcome(
                EvidenceKind.INCONCLUSIVE,
                'incomplete_coverage:' + ','.join(incomplete),
            )

        return self._outcome(
            EvidenceKind.QUALIFIED_CLEAR,
            'qualified_absence',
        )

    def reset(self) -> None:
        self.__init__(self._config)

    def _consume_event(self, event: EvidenceEvent) -> None:
        # Strict identity matching prevents stale events from changing the
        # observation result.
        if (
            event.run_id != self._run_id
            or event.observation_id != self._observation_id
        ):
            self._has_hold_or_inconclusive = True
            return

        if event.kind is EvidenceKind.CONFLICT:
            self._has_conflict = True
            return

        if event.kind is EvidenceKind.POSITIVE:
            if event.site_id:
                self._positive_sites.add(event.site_id)
            else:
                self._has_hold_or_inconclusive = True
            return

        if event.kind in (
            EvidenceKind.PRECAUTIONARY_HOLD,
            EvidenceKind.INCONCLUSIVE,
        ):
            self._has_hold_or_inconclusive = True
            return

        # QUALIFIED_CLEAR belongs to a completed observation result, not a
        # per-frame fusion input. Treat an unexpected one conservatively.
        if event.kind is EvidenceKind.QUALIFIED_CLEAR:
            self._has_hold_or_inconclusive = True

    def _validate(
        self,
        frame: ObservationFrame,
        pose: PoseSample,
        now: float,
    ) -> Optional[str]:
        if not frame.healthy:
            return 'camera_unhealthy'
        if frame.validity_reason != 'ok':
            return 'frame_invalid'
        if not math.isfinite(float(frame.capture_timestamp_s)):
            return 'capture_timestamp_non_finite'
        if not math.isfinite(float(pose.timestamp_s)):
            return 'pose_timestamp_non_finite'
        if (
            abs(
                pose.timestamp_s
                - frame.capture_timestamp_s
            )
            > self._config.max_pose_timestamp_error_s
        ):
            return 'pose_not_at_capture_time'
        if not math.isfinite(float(now)):
            return 'now_non_finite'
        if float(now) < frame.capture_timestamp_s:
            return 'capture_from_future'
        if frame.covered_sectors.difference(
            self._config.required_sectors
        ):
            return 'unknown_coverage_sector'
        return None

    def _temporal_span_s(self) -> float:
        if len(self._healthy_timestamps) < 2:
            return 0.0
        values = self._healthy_timestamps.values()
        return max(values) - min(values)

    def _progress(
        self,
        accepted: bool,
        duplicate: bool,
        reason: str,
    ) -> ObservationProgress:
        return ObservationProgress(
            accepted=accepted,
            duplicate=duplicate,
            reason=reason,
            distinct_healthy_frames=len(self._healthy_timestamps),
            temporal_span_s=self._temporal_span_s(),
            has_positive=bool(self._positive_sites),
            has_conflict=self._has_conflict,
            has_hold_or_inconclusive=self._has_hold_or_inconclusive,
            camera_failure=self._camera_failure,
        )

    def _outcome(
        self,
        kind: EvidenceKind,
        reason: str,
        sites: set[str] | None = None,
    ) -> ObservationOutcome:
        return ObservationOutcome(
            kind=kind,
            reason=reason,
            distinct_healthy_frames=len(self._healthy_timestamps),
            temporal_span_s=self._temporal_span_s(),
            site_ids=tuple(sorted(sites or set())),
        )
