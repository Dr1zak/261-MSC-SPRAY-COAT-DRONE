import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from construction_site import OBSTACLES, WAYPOINTS, validate_flight_path


def test_construction_flight_matches_site_collision_map():
    scenario = yaml.safe_load((ROOT / 'config/scenarios/construction_site.yaml').read_text())
    assert tuple(tuple(rect) for rect in scenario['obstacles']) == OBSTACLES
    assert tuple(scenario['home']) == WAYPOINTS[0][:2] == WAYPOINTS[-1][:2]
    validate_flight_path(tuple(scenario['bounds']))
