from __future__ import annotations
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'test' / 'fixtures' / 'wp14' / 'wp14_scenarios.json'
# When run directly from the work-package bundle, fixtures live in config/.
if not FIXTURE.exists():
    FIXTURE = ROOT / 'config' / 'wp14_scenarios.json'


def test_wp14_manifest_is_hardware_disconnected_and_domains_are_unique():
    data=json.loads(FIXTURE.read_text())
    assert data['schema_version']==1
    assert data['hardware_access'] is False
    domains=list(data['domains'].values())
    assert len(domains)==len(set(domains))
    assert all(isinstance(v,int) and 100 <= v <= 232 for v in domains)


def test_wp14_full_mission_contract_is_two_unique_stops_then_parking():
    data=json.loads(FIXTURE.read_text())['full_mission']
    assert data['required_unique_bus_stops']==2
    kinds=[x['kind'] for x in data['targets']]
    assert kinds==['BUS_STOP','BUS_STOP','PARKING']
    ids=[x['id'] for x in data['targets']]
    assert len(ids)==len(set(ids))==3
    assert data['expected_nav2_goals']==3
    assert data['expected_execution_reports']==3
    assert data['expected_final_phase']=='DONE'


def test_wp14_failure_matrix_contains_required_fail_closed_cases():
    cases=set(json.loads(FIXTURE.read_text())['failure_cases'])
    assert {'nav2_reject','nav2_abort','stale_odom','map_identity_change','system_readiness_loss'} <= cases
