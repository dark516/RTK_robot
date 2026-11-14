"""WP06 sign fusion finalized against Agent 1 shared semantic types.

Owned by Agent 2.  This module intentionally imports the frozen semantic event
types from ``frob_city_navigation.models`` and does not define a second private
EvidenceEvent/EvidenceKind/SignClass model.

The only private detector records here form the legacy detector conversion
boundary.  They preserve range convention and uncertainty while converting
centimeters to meters exactly once.
"""

from __future__ import annotations

from frob_city_navigation.models import PoseSample

from collections import defaultdict, deque
from dataclasses import MISSING, dataclass, fields
import math
from typing import Deque, Mapping, Optional, Protocol, Sequence

from .models import EvidenceEvent, EvidenceKind, RangeConvention, SignClass


@dataclass(frozen=True)
class LegacyRawDetection:
    label: str
    confidence: float
    bbox_xyxy_px: tuple[float, float, float, float]
    range_value: Optional[float] = None
    range_unit: str = 'm'
    range_convention: RangeConvention = RangeConvention.RANGE_UNKNOWN
    range_stddev_value: Optional[float] = None
    track_id: Optional[str] = None
    valid: bool = True


@dataclass(frozen=True)
class LegacyRawDetectionFrame:
    sequence: int
    capture_timestamp_s: float
    frame_id: str
    healthy: bool
    detections: tuple[LegacyRawDetection, ...] = ()
    calibration_valid: bool = True
    validity_reason: str = 'ok'


@dataclass(frozen=True)
class CanonicalDetection:
    sign_class: SignClass
    confidence: float
    bbox_xyxy_px: tuple[float, float, float, float]
    range_m: Optional[float]
    range_convention: RangeConvention
    range_stddev_m: Optional[float]
    track_id: Optional[str]


@dataclass(frozen=True)
class CanonicalDetectionFrame:
    sequence: int
    capture_timestamp_s: float
    frame_id: str
    detections: tuple[CanonicalDetection, ...]


@dataclass(frozen=True)
class DetectorBoundaryConfig:
    label_to_sign_class: Mapping[str, SignClass]
    min_confidence: float = 0.0
    max_confidence: float = 1.0
    image_width_px: Optional[int] = None
    image_height_px: Optional[int] = None
    require_calibration: bool = True


@dataclass(frozen=True)
class DetectorBoundaryResult:
    accepted: bool
    duplicate: bool
    reason: str
    frame: Optional[CanonicalDetectionFrame] = None


class LegacyDetectorAdapter:
    """Convert a legacy detector record to the internal canonical boundary.

    Conversion from centimeters to meters happens here exactly once for both
    range and its standard deviation.
    """

    def __init__(self, config: DetectorBoundaryConfig) -> None:
        if not config.label_to_sign_class:
            raise ValueError('label_to_sign_class must not be empty')
        if not (0.0 <= config.min_confidence <= config.max_confidence <= 1.0):
            raise ValueError('invalid confidence range')
        self._config = config
        self._seen_sequences: set[int] = set()

    def reset_sequences(self) -> None:
        self._seen_sequences.clear()

    def convert(self, raw: LegacyRawDetectionFrame) -> DetectorBoundaryResult:
        error = self._validate_frame(raw)
        if error is not None:
            return DetectorBoundaryResult(False, False, error)

        if raw.sequence in self._seen_sequences:
            return DetectorBoundaryResult(
                False,
                True,
                'duplicate_sequence',
            )

        detections: list[CanonicalDetection] = []
        for index, item in enumerate(raw.detections):
            error = self._validate_detection(item)
            if error is not None:
                return DetectorBoundaryResult(
                    False,
                    False,
                    f'detection[{index}]:{error}',
                )
            detections.append(self._convert_detection(item))

        self._seen_sequences.add(raw.sequence)
        return DetectorBoundaryResult(
            True,
            False,
            'ok',
            CanonicalDetectionFrame(
                sequence=raw.sequence,
                capture_timestamp_s=float(raw.capture_timestamp_s),
                frame_id=raw.frame_id,
                detections=tuple(detections),
            ),
        )

    def _validate_frame(
        self,
        raw: LegacyRawDetectionFrame,
    ) -> Optional[str]:
        if not isinstance(raw.sequence, int) or isinstance(raw.sequence, bool):
            return 'sequence_not_integer'
        if raw.sequence < 0:
            return 'sequence_negative'
        if not math.isfinite(float(raw.capture_timestamp_s)):
            return 'capture_timestamp_non_finite'
        if raw.capture_timestamp_s < 0:
            return 'capture_timestamp_negative'
        if not raw.frame_id or not raw.frame_id.strip():
            return 'frame_id_empty'
        if not raw.healthy:
            return 'detector_unhealthy'
        if self._config.require_calibration and not raw.calibration_valid:
            return 'calibration_invalid'
        if raw.validity_reason != 'ok':
            return f'frame_invalid:{raw.validity_reason}'
        return None

    def _validate_detection(
        self,
        item: LegacyRawDetection,
    ) -> Optional[str]:
        if not item.valid:
            return 'marked_invalid'
        if item.label not in self._config.label_to_sign_class:
            return 'label_not_mapped'

        confidence = float(item.confidence)
        if not math.isfinite(confidence):
            return 'confidence_non_finite'
        if not (
            self._config.min_confidence
            <= confidence
            <= self._config.max_confidence
        ):
            return 'confidence_out_of_range'

        bbox_error = self._validate_bbox(item.bbox_xyxy_px)
        if bbox_error is not None:
            return bbox_error

        if item.range_value is not None:
            value = float(item.range_value)
            if not math.isfinite(value) or value <= 0:
                return 'range_invalid'
            if item.range_convention is RangeConvention.RANGE_UNKNOWN:
                return 'range_convention_unknown'

        if item.range_stddev_value is not None:
            stddev = float(item.range_stddev_value)
            if not math.isfinite(stddev) or stddev < 0:
                return 'range_stddev_invalid'
            if item.range_value is None:
                return 'range_stddev_without_range'

        if item.range_unit not in ('m', 'cm'):
            return 'range_unit_unsupported'

        return None

    def _validate_bbox(
        self,
        bbox: Sequence[float],
    ) -> Optional[str]:
        if len(bbox) != 4:
            return 'bbox_wrong_length'
        try:
            x1, y1, x2, y2 = (float(value) for value in bbox)
        except (TypeError, ValueError):
            return 'bbox_not_numeric'
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
            return 'bbox_non_finite'
        if x1 < 0 or y1 < 0 or x2 <= x1 or y2 <= y1:
            return 'bbox_invalid'
        if (
            self._config.image_width_px is not None
            and x2 > self._config.image_width_px
        ):
            return 'bbox_outside_width'
        if (
            self._config.image_height_px is not None
            and y2 > self._config.image_height_px
        ):
            return 'bbox_outside_height'
        return None

    def _convert_detection(
        self,
        item: LegacyRawDetection,
    ) -> CanonicalDetection:
        scale = 0.01 if item.range_unit == 'cm' else 1.0
        range_m = (
            None
            if item.range_value is None
            else float(item.range_value) * scale
        )
        range_stddev_m = (
            None
            if item.range_stddev_value is None
            else float(item.range_stddev_value) * scale
        )
        return CanonicalDetection(
            sign_class=self._config.label_to_sign_class[item.label],
            confidence=float(item.confidence),
            bbox_xyxy_px=tuple(float(value) for value in item.bbox_xyxy_px),
            range_m=range_m,
            range_convention=item.range_convention,
            range_stddev_m=range_stddev_m,
            track_id=item.track_id,
        )


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float




@dataclass(frozen=True)
class PoseLookupResult:
    ok: bool
    reason: str
    sample: Optional[PoseSample] = None


class PoseAtTimePort(Protocol):
    def pose_at(self, timestamp_s: float) -> PoseLookupResult:
        ...


@dataclass(frozen=True)
class SiteCandidate:
    """Possible physical sign face from static field geometry.

    There is deliberately no expected/actual sign class in static geometry.
    """

    site_id: str
    face_id: str
    controlled_approach_id: str
    bearing_rad: float
    range_m: float
    face_visible: bool = True
    right_roadside: bool = True
    current_junction: bool = True
    bearing_consistent: bool = True
    association_score: float = 0.0

    @property
    def face_key(self) -> str:
        return f'{self.site_id}/{self.face_id}'


class FieldQueryPort(Protocol):
    def candidates(
        self,
        *,
        pose: PoseSample,
        detection: CanonicalDetection,
    ) -> tuple[SiteCandidate, ...]:
        ...


@dataclass(frozen=True)
class ObservationContext:
    run_id: str
    observation_id: str
    approach_id: str
    map_instance_id: str
    route_revision: str


@dataclass(frozen=True)
class SignFusionConfig:
    # Two distinct frames reject a one-frame hallucination without demanding
    # several seconds of visibility from a moving robot.
    required_positive_frames: int = 2
    positive_window_s: float = 6.0
    positive_min_span_s: float = 0.15
    max_range_error_m: float = 0.55
    range_sigma_multiplier: float = 3.0
    comparable_confidence_delta: float = 0.15
    # A sign can be observed while driving straight, but the field association
    # treats robot yaw as the road heading.  Do not accumulate map evidence
    # while that heading is changing through a turn.
    max_mapping_angular_speed_radps: float = 0.15
    positive_kind: Optional[EvidenceKind] = None
    confirmed_kind: Optional[EvidenceKind] = None
    conflict_kind: Optional[EvidenceKind] = None
    hold_kind: Optional[EvidenceKind] = None
    rejected_kind: Optional[EvidenceKind] = None


@dataclass(frozen=True)
class _Positive:
    sequence: int
    timestamp_s: float
    confidence: float


class SignFusion:
    """Capture-time fusion keyed by (physical face, canonical SignClass)."""

    def __init__(
        self,
        pose_port: PoseAtTimePort,
        field_query: FieldQueryPort,
        config: SignFusionConfig | None = None,
    ) -> None:
        self._pose_port = pose_port
        self._field_query = field_query
        self._config = config or SignFusionConfig()

        if self._config.required_positive_frames < 1:
            raise ValueError('required_positive_frames must be >= 1')
        if self._config.positive_window_s <= 0:
            raise ValueError('positive_window_s must be > 0')
        if not (
            0 <= self._config.positive_min_span_s
            <= self._config.positive_window_s
        ):
            raise ValueError('invalid positive_min_span_s')
        if (
            not math.isfinite(self._config.max_mapping_angular_speed_radps)
            or self._config.max_mapping_angular_speed_radps < 0.0
        ):
            raise ValueError('max_mapping_angular_speed_radps must be finite and >= 0')

        self._kinds = {
            # Frozen Agent-1 semantics:
            # - POSITIVE means fusion has met its confirmation requirements.
            # - INCONCLUSIVE is used while positive evidence is still
            #   accumulating or when association cannot be resolved.
            'unconfirmed': self._config.positive_kind
            or _resolve_kind('unconfirmed'),
            'confirmed': self._config.confirmed_kind
            or _resolve_kind('confirmed'),
            'conflict': self._config.conflict_kind
            or _resolve_kind('conflict'),
            'hold': self._config.hold_kind
            or _resolve_kind('hold'),
            'rejected': self._config.rejected_kind
            or _resolve_kind('rejected'),
        }

        self._active_run_id: Optional[str] = None
        self._positives: dict[
            tuple[str, str, SignClass],
            Deque[_Positive],
        ] = defaultdict(deque)
        self._confirmed: set[tuple[str, str, SignClass]] = set()
        self._event_counter = 0

    def begin_run(self, run_id: str) -> None:
        """Full new-run reset, including previously confirmed evidence."""
        if not run_id:
            raise ValueError('run_id must not be empty')
        self._active_run_id = run_id
        self._positives.clear()
        self._confirmed.clear()
        self._event_counter = 0

    def reset_run(self, run_id: str) -> None:
        self.begin_run(run_id)

    def reset_unconfirmed(self, reason: str) -> None:
        """Observation-session reset; confirmed evidence remains for this run."""
        del reason
        self._positives = defaultdict(
            deque,
            {
                key: values
                for key, values in self._positives.items()
                if key in self._confirmed
            },
        )

    def ingest(
        self,
        frame: CanonicalDetectionFrame,
        context: ObservationContext,
    ) -> tuple[EvidenceEvent, ...]:
        if self._active_run_id is None:
            self.begin_run(context.run_id)
        elif context.run_id != self._active_run_id:
            return self._hold_whole_frame(
                frame,
                context,
                'run_identity_mismatch',
            )

        pose_result = self._pose_port.pose_at(
            frame.capture_timestamp_s
        )
        if not pose_result.ok or pose_result.sample is None:
            return self._hold_whole_frame(
                frame,
                context,
                f'capture_pose:{pose_result.reason}',
            )

        pose = pose_result.sample
        if (
            abs(pose.angular_speed_radps)
            > self._config.max_mapping_angular_speed_radps
        ):
            # Evidence gathered before a turn must not be completed by frames
            # from a different road heading after the turn.
            self.reset_unconfirmed('turn_in_progress')
            return self._reject_whole_frame(
                frame,
                context,
                'turn_in_progress',
            )

        events: list[EvidenceEvent] = []
        for detection_index, detection in enumerate(frame.detections):
            candidate, reason = self._associate(
                pose,
                detection,
                context,
            )
            if candidate is None:
                events.append(
                    self._event(
                        semantic='rejected',
                        context=context,
                        frame=frame,
                        detection=detection,
                        detection_index=detection_index,
                        site_id='',
                        approach_id=context.approach_id,
                        reason=reason,
                    )
                )
                continue

            conflict = self._find_competing_class(
                candidate,
                detection,
                frame.capture_timestamp_s,
            )
            if conflict:
                events.append(
                    self._event(
                        semantic='conflict',
                        context=context,
                        frame=frame,
                        detection=detection,
                        detection_index=detection_index,
                        site_id=candidate.face_key,
                        approach_id=candidate.controlled_approach_id,
                        reason='competing_class_same_face',
                    )
                )
                continue

            key = (
                candidate.site_id,
                candidate.face_id,
                detection.sign_class,
            )
            queue = self._positives[key]
            self._prune(queue, frame.capture_timestamp_s)

            if all(item.sequence != frame.sequence for item in queue):
                queue.append(
                    _Positive(
                        sequence=frame.sequence,
                        timestamp_s=frame.capture_timestamp_s,
                        confidence=detection.confidence,
                    )
                )

            count = len(queue)
            span = (
                queue[-1].timestamp_s - queue[0].timestamp_s
                if count >= 2
                else 0.0
            )
            confirmed = (
                count >= self._config.required_positive_frames
                and span >= self._config.positive_min_span_s
            )
            semantic = 'confirmed' if confirmed else 'unconfirmed'
            if confirmed:
                self._confirmed.add(key)

            events.append(
                self._event(
                    semantic=semantic,
                    context=context,
                    frame=frame,
                    detection=detection,
                    detection_index=detection_index,
                    site_id=candidate.face_key,
                    approach_id=candidate.controlled_approach_id,
                    reason=(
                        'required_distinct_positive_frames_met'
                        if confirmed
                        else 'positive_unconfirmed'
                    ),
                )
            )

        return tuple(events)

    def positive_count(
        self,
        *,
        site_id: str,
        face_id: str,
        sign_class: SignClass,
    ) -> int:
        return len(self._positives[(site_id, face_id, sign_class)])

    def _associate(
        self,
        pose: PoseSample,
        detection: CanonicalDetection,
        context: ObservationContext,
    ) -> tuple[Optional[SiteCandidate], str]:
        candidates = self._field_query.candidates(
            pose=pose,
            detection=detection,
        )
        if not candidates:
            return None, 'no_candidate_face'

        usable = [item for item in candidates if item.face_visible]
        if not usable:
            return None, 'back_face_rejected'
        usable = [item for item in usable if item.right_roadside]
        if not usable:
            return None, 'wrong_roadside_rejected'
        usable = [item for item in usable if item.current_junction]
        if not usable:
            return None, 'next_junction_rejected'
        usable = [item for item in usable if item.bearing_consistent]
        if not usable:
            return None, 'bearing_inconsistent'
        if context.approach_id:
            usable = [
                item
                for item in usable
                if item.controlled_approach_id == context.approach_id
            ]
            if not usable:
                return None, 'wrong_approach_rejected'

        if detection.range_m is not None:
            if detection.range_convention is RangeConvention.RANGE_UNKNOWN:
                return None, 'range_convention_unknown'
            uncertainty = detection.range_stddev_m or 0.0
            tolerance = max(
                self._config.max_range_error_m,
                self._config.range_sigma_multiplier * uncertainty,
            )
            usable = [
                item
                for item in usable
                if abs(item.range_m - detection.range_m) <= tolerance
            ]
            if not usable:
                return None, 'range_inconsistent'

        if len(usable) != 1:
            return None, 'ambiguous_candidate_face'

        return usable[0], 'ok'

    def _find_competing_class(
        self,
        candidate: SiteCandidate,
        detection: CanonicalDetection,
        timestamp_s: float,
    ) -> bool:
        for key, queue in list(self._positives.items()):
            site_id, face_id, sign_class = key
            if site_id != candidate.site_id or face_id != candidate.face_id:
                continue
            if sign_class == detection.sign_class:
                continue

            self._prune(queue, timestamp_s)
            if not queue:
                continue

            strongest = max(item.confidence for item in queue)
            if (
                abs(strongest - detection.confidence)
                <= self._config.comparable_confidence_delta
            ):
                return True
        return False

    def _prune(
        self,
        queue: Deque[_Positive],
        timestamp_s: float,
    ) -> None:
        cutoff = timestamp_s - self._config.positive_window_s
        while queue and queue[0].timestamp_s < cutoff:
            queue.popleft()

    def _hold_whole_frame(
        self,
        frame: CanonicalDetectionFrame,
        context: ObservationContext,
        reason: str,
    ) -> tuple[EvidenceEvent, ...]:
        if not frame.detections:
            return ()
        return tuple(
            self._event(
                semantic='hold',
                context=context,
                frame=frame,
                detection=detection,
                detection_index=index,
                site_id='',
                approach_id=context.approach_id,
                reason=reason,
            )
            for index, detection in enumerate(frame.detections)
        )

    def _reject_whole_frame(
        self,
        frame: CanonicalDetectionFrame,
        context: ObservationContext,
        reason: str,
    ) -> tuple[EvidenceEvent, ...]:
        if not frame.detections:
            return ()
        return tuple(
            self._event(
                semantic='rejected',
                context=context,
                frame=frame,
                detection=detection,
                detection_index=index,
                site_id='',
                approach_id=context.approach_id,
                reason=reason,
            )
            for index, detection in enumerate(frame.detections)
        )

    def _event(
        self,
        *,
        semantic: str,
        context: ObservationContext,
        frame: CanonicalDetectionFrame,
        detection: CanonicalDetection,
        detection_index: int,
        site_id: str,
        approach_id: str,
        reason: str,
    ) -> EvidenceEvent:
        self._event_counter += 1
        event_id = (
            f'{context.run_id}:{context.observation_id}:'
            f'{frame.sequence}:{detection_index}:{self._event_counter}'
        )
        kwargs = {
            'run_id': context.run_id,
            'event_id': event_id,
            'timestamp_s': frame.capture_timestamp_s,
            'kind': self._kinds[semantic],
            'site_id': site_id,
            'approach_id': approach_id,
            'sign_class': detection.sign_class,
            'confidence': detection.confidence,
            'observation_id': context.observation_id,
            'reason': reason,
        }
        return _construct_evidence_event(kwargs)


def _construct_evidence_event(kwargs: dict) -> EvidenceEvent:
    """Instantiate the frozen dataclass and fail clearly on unknown required fields."""
    if not hasattr(EvidenceEvent, '__dataclass_fields__'):
        return EvidenceEvent(**kwargs)

    missing_required = []
    accepted = {}
    for item in fields(EvidenceEvent):
        if item.name in kwargs:
            accepted[item.name] = kwargs[item.name]
        elif (
            item.default is MISSING
            and item.default_factory is MISSING
        ):
            missing_required.append(item.name)

    if missing_required:
        raise RuntimeError(
            'Frozen EvidenceEvent gained unsupported required fields: '
            + ','.join(sorted(missing_required))
        )
    return EvidenceEvent(**accepted)


def _resolve_kind(semantic: str) -> EvidenceKind:
    aliases = {
        'unconfirmed': (
            'INCONCLUSIVE',
        ),
        'confirmed': (
            'POSITIVE',
        ),
        'conflict': (
            'CONFLICT',
        ),
        'hold': (
            'PRECAUTIONARY_HOLD',
            'INCONCLUSIVE',
        ),
        'rejected': (
            'INCONCLUSIVE',
            'PRECAUTIONARY_HOLD',
        ),
    }
    members = getattr(EvidenceKind, '__members__', {})
    for name in aliases[semantic]:
        if name in members:
            return members[name]

    semantic_upper = semantic.upper()
    for name, value in members.items():
        if semantic_upper in name.upper():
            return value

    raise RuntimeError(
        f'Cannot map semantic evidence kind {semantic!r} to frozen '
        f'EvidenceKind members {sorted(members)}. '
        'Pass an explicit kind in SignFusionConfig.'
    )
