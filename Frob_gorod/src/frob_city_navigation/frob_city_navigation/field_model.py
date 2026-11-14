"""Static, ROS-independent field model for Frob City WP04."""

from __future__ import annotations

from .geometry import (
    field_to_map_point,
    field_to_map_polygon,
    field_to_map_pose,
    map_to_field_point,
    map_to_field_polygon,
    map_to_field_pose,
)
from .models import (
    DirectedAnchor,
    FieldConfig,
    Gate,
    MotionPrimitive,
    NamedPolygon,
    ObservationRegion,
    Point2D,
    Polygon2D,
    Pose2D,
    SignFace,
    SignSite,
)


class FieldModel:
    """Validated static geometry indexed for deterministic pure-domain queries.

    Field YAML geometry is interpreted in the conceptual field frame.  The
    configured ``field_to_map`` rigid transform is the single conversion to the
    navigation ``map`` frame.  An uncalibrated transform may still be identity
    in software fixtures, but ``geometry_calibrated`` remains false and must not
    be presented as a physical validation result.
    """

    def __init__(self, config: FieldConfig):
        self.config = config
        self._anchors: dict[str, DirectedAnchor] = {
            a.anchor_id: DirectedAnchor(a.anchor_id, a.cell_id, a.pose)
            for a in config.anchors
        }
        self._gates: dict[str, Gate] = {
            g.gate_id: Gate(g.gate_id, g.pose) for g in config.gates
        }
        region_by_id = {p.polygon_id: p.polygon for p in config.drivable_regions}
        self._regions = region_by_id
        self._platforms = {p.polygon_id: p.polygon for p in config.platforms}
        self._observations: dict[str, ObservationRegion] = {
            o.observation_id: ObservationRegion(
                o.observation_id,
                o.polygon,
                o.anchor_id,
                o.nominal_heading_rad,
                o.heading_tolerance_rad,
            )
            for o in config.observation_regions
        }
        self._sign_sites: dict[str, SignSite] = {
            s.site_id: SignSite(
                s.site_id,
                s.position,
                tuple(SignFace(f.face_id, f.yaw_rad, f.controlled_approach_id) for f in s.faces),
            )
            for s in config.sign_sites
        }
        self._movements: dict[str, MotionPrimitive] = {}
        for edge in config.edges:
            regions = tuple(region_by_id[rid] for rid in edge.permitted_region_ids)
            gate = self._gates.get(edge.commitment_gate_id) if edge.commitment_gate_id else None
            self._movements[edge.edge_id] = MotionPrimitive(
                movement_id=edge.edge_id,
                from_anchor_id=edge.from_anchor_id,
                to_anchor_id=edge.to_anchor_id,
                movement=edge.movement,
                path=edge.path,
                permitted_regions=regions,
                approach_id=edge.approach_id,
                commitment_gate=gate,
            )

    @classmethod
    def from_config(cls, config: FieldConfig) -> "FieldModel":
        return cls(config)

    @property
    def geometry_calibrated(self) -> bool:
        return bool(self.config.geometry_calibrated and self.config.field_to_map.valid)

    @property
    def footprint(self) -> Polygon2D:
        return self.config.footprint

    @property
    def movement_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._movements))

    @property
    def anchor_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._anchors))

    @property
    def observation_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._observations))

    def movements_from(self, anchor_id: str) -> tuple[MotionPrimitive, ...]:
        return tuple(
            self._movements[mid]
            for mid in sorted(self._movements)
            if self._movements[mid].from_anchor_id == anchor_id
        )

    def observation_regions_at_anchor(self, anchor_id: str) -> tuple[ObservationRegion, ...]:
        return tuple(
            self._observations[oid]
            for oid in sorted(self._observations)
            if self._observations[oid].anchor_id == anchor_id
        )

    def drivable_regions(self) -> tuple[Polygon2D, ...]:
        return tuple(self._regions[k] for k in sorted(self._regions))

    @property
    def sign_site_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._sign_sites))

    def anchor(self, anchor_id: str) -> DirectedAnchor:
        try:
            return self._anchors[anchor_id]
        except KeyError as exc:
            raise KeyError(f"unknown anchor_id: {anchor_id}") from exc

    def movement_primitive(self, movement_id: str) -> MotionPrimitive:
        try:
            return self._movements[movement_id]
        except KeyError as exc:
            raise KeyError(f"unknown movement_id: {movement_id}") from exc

    def observation_region(self, observation_id: str) -> ObservationRegion:
        try:
            return self._observations[observation_id]
        except KeyError as exc:
            raise KeyError(f"unknown observation_id: {observation_id}") from exc

    def gate(self, gate_id: str) -> Gate:
        try:
            return self._gates[gate_id]
        except KeyError as exc:
            raise KeyError(f"unknown gate_id: {gate_id}") from exc

    def sign_site(self, site_id: str) -> SignSite:
        try:
            return self._sign_sites[site_id]
        except KeyError as exc:
            raise KeyError(f"unknown sign site: {site_id}") from exc

    def candidate_sign_sites(self, controlled_approach_id: str = "") -> tuple[SignSite, ...]:
        sites = []
        for site_id in sorted(self._sign_sites):
            site = self._sign_sites[site_id]
            if not controlled_approach_id or any(
                f.controlled_approach_id == controlled_approach_id for f in site.faces
            ):
                sites.append(site)
        return tuple(sites)

    def platform_polygons(self) -> tuple[Polygon2D, ...]:
        return tuple(self._platforms[k] for k in sorted(self._platforms))

    def field_to_map_pose(self, pose: Pose2D) -> Pose2D:
        return field_to_map_pose(pose, self.config.field_to_map)

    def map_to_field_pose(self, pose: Pose2D) -> Pose2D:
        return map_to_field_pose(pose, self.config.field_to_map)

    def field_to_map_point(self, point: Point2D) -> Point2D:
        return field_to_map_point(point, self.config.field_to_map)

    def map_to_field_point(self, point: Point2D) -> Point2D:
        return map_to_field_point(point, self.config.field_to_map)

    def field_to_map_polygon(self, polygon: Polygon2D) -> Polygon2D:
        return field_to_map_polygon(polygon, self.config.field_to_map)

    def map_to_field_polygon(self, polygon: Polygon2D) -> Polygon2D:
        return map_to_field_polygon(polygon, self.config.field_to_map)

    def is_inside_field_bounds(self, point: Point2D, tolerance_m: float = 0.0) -> bool:
        return (
            -tolerance_m <= point.x_m <= self.config.width_m + tolerance_m
            and -tolerance_m <= point.y_m <= self.config.height_m + tolerance_m
        )
