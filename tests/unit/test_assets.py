import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_step_mapping_matches_source_and_urdf_mesh_groups():
    config = yaml.safe_load((ROOT / 'config/asset_mapping.yaml').read_text())
    source = json.loads((ROOT / 'x500_mount_v6/assembly_data.json').read_text())
    mapping = {part['name']: part['link'] for part in source['mass_items']}
    assert len(mapping) == 90
    assert config['step_part_to_urdf_link'] == mapping
    assert sum(len(groups) for groups in source['mesh_groups'].values()) == 17
    for link, groups in source['mesh_groups'].items():
        assert all(mapping[name] == link for group in groups for name in group['parts'])
