from pathlib import Path

import yaml

from frob_city_navigation.competition_profile import build_competition_field, build_competition_nav2
from frob_city_navigation.config_loader import load_project_config
from frob_city_navigation.models import ConfigPaths, EvidenceEvent, EvidenceKind, SignClass
from frob_city_navigation.models import PoseSample, SegmentBuildStatus
from frob_city_navigation.field_model import FieldModel
from frob_city_navigation.graph import build_transitions
from frob_city_navigation.path_builder import PathBuilder
from frob_city_navigation.semantic_map import SemanticMap


CONFIG = Path(__file__).parents[1] / 'config'


def test_competition_profile_builds_atomic_5x5_topology():
    manual = yaml.safe_load((CONFIG / 'andrey_manual_field.yaml').read_text())
    field = build_competition_field(manual)

    assert len(field['edges']) == 68
    assert len(field['observation_regions']) == 24
    assert len(field['gates']) == 24
    movements = {edge['movement'] for edge in field['edges']}
    assert movements == {'LEFT', 'STRAIGHT', 'RIGHT'}
    controlled = [
        face['controlled_approach_id']
        for site in field['sign_sites'] for face in site['faces']
        if face['controlled_approach_id']
    ]
    assert len(controlled) == 32
    assert set(controlled) <= {
        edge['approach_id'] for edge in field['edges'] if edge['approach_id']
    }


def test_no_generated_maneuver_returns_to_its_source_side():
    manual = yaml.safe_load((CONFIG / 'andrey_manual_field.yaml').read_text())
    field = build_competition_field(manual)
    for edge in field['edges']:
        if not edge['id'].startswith('MANEUVER_'):
            continue
        source, rest = edge['id'].removeprefix('MANEUVER_').split('_VIA_', 1)
        destination = rest.split('_TO_', 1)[1]
        assert destination != source


def test_competition_nav2_is_forward_only_and_uses_guard_chain():
    source = yaml.safe_load((CONFIG / 'nav2_city_params.generated.yaml').read_text())
    params = build_competition_nav2(source, {'x_m': 1.0, 'y_m': 2.0, 'yaw_rad': 0.3})

    amcl = params['amcl']['ros__parameters']
    assert amcl['set_initial_pose'] is True
    assert amcl['initial_pose'] == {'x': 1.0, 'y': 2.0, 'z': 0.0, 'yaw': 0.3}
    follow = params['controller_server']['ros__parameters']['FollowPath']
    assert follow['allow_reversing'] is False
    assert follow['desired_linear_vel'] == 0.12
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['min_velocity'][0] == 0.0
    assert params['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] == '/city/cmd_vel_raw'
    assert params['behavior_server']['ros__parameters']['behavior_plugins'] == ['spin', 'wait']


def test_every_legal_target_face_gets_valid_forward_service_geometry():
    config = load_project_config(ConfigPaths(
        field=CONFIG / 'andrey_competition_field.yaml',
        event=CONFIG / 'andrey_competition_event.yaml',
        perception=CONFIG / 'andrey_competition_perception.yaml',
        navigation=CONFIG / 'andrey_competition_navigation.yaml',
    ))
    semantic = SemanticMap(config.field)
    semantic.begin_run('RUN', 'FIELD')
    sequence = 0
    for site in config.field.sign_sites:
        for face in site.faces:
            if not face.controlled_approach_id:
                continue
            sequence += 1
            semantic.apply(EvidenceEvent(
                run_id='RUN', event_id=f'e{sequence}', timestamp_s=float(sequence),
                kind=EvidenceKind.POSITIVE,
                site_id=f'{site.site_id}/{face.face_id}',
                approach_id=face.controlled_approach_id,
                sign_class=SignClass.BUS_STOP, confidence=0.9,
            ))

    targets = semantic.snapshot().targets
    assert len(targets) == 32
    assert all(target.geometry_valid for target in targets)
    assert all(target.directed_road_id for target in targets)


def test_every_generated_maneuver_has_valid_swept_footprint():
    config = load_project_config(ConfigPaths(
        field=CONFIG / 'andrey_competition_field.yaml',
        event=CONFIG / 'andrey_competition_event.yaml',
        perception=CONFIG / 'andrey_competition_perception.yaml',
        navigation=CONFIG / 'andrey_competition_navigation.yaml',
    ))
    field = FieldModel(config.field)
    semantic = SemanticMap(config.field)
    snapshot = semantic.begin_run('RUN', 'FIELD')
    for index, approach in enumerate(snapshot.approaches):
        semantic.apply(EvidenceEvent(
            run_id='RUN', event_id=f'clear-{index}', timestamp_s=float(index + 1),
            kind=EvidenceKind.QUALIFIED_CLEAR,
            approach_id=approach.approach_id,
            observation_id=f'qualified-{index}',
        ))
    snapshot = semantic.snapshot()
    builder = PathBuilder(field, config.navigation)
    for transition in build_transitions(field, snapshot, config.navigation):
        start_map = field.field_to_map_pose(transition.path.poses[0])
        built = builder.build_segment(
            transition, snapshot, PoseSample(1.0, start_map)
        )
        assert built.status is SegmentBuildStatus.BUILT, (
            transition.transition_id, built.reason
        )
