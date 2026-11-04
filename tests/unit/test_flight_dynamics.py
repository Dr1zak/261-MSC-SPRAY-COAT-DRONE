import csv
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from flight_dynamics import FlightController, ThrustCurve, added_mass_properties, urdf_mass_kg


def test_imported_mass_and_hover_force_allocation():
    config = yaml.safe_load((ROOT / 'config/flight_controller.yaml').read_text())
    mass = urdf_mass_kg(ROOT / 'x500_mount_v6/urdf/x500_v2_sprayer.urdf')
    assert np.isclose(mass, 2.74439014475)
    components = yaml.safe_load((ROOT / 'config/airframe_estimate.yaml').read_text())['components']
    added, added_com, added_inertia = added_mass_properties(components)
    assert np.isclose(added, 1.0251)
    assert np.all(np.diag(added_inertia) > 0)
    curve = ThrustCurve(ROOT / config['thrust_curve_csv'], config['max_continuous_throttle'])
    assert np.isclose(curve.force(.70), .814 * 9.81)
    assert np.isclose(curve.throttle_for_force(curve.force(.77)), .77)
    # The COM measured in the Isaac import shifts load toward the positive-X motors.
    total = mass + added
    com = (mass * np.array((0.01548, 0.00279, -0.10927)) + added * added_com) / total
    controller = FlightController(config, total, com, curve)
    forces = np.linalg.solve(controller.allocation, [total * 9.81, 0, 0, 0])
    assert np.all(forces > 0)
    assert np.all(forces < curve.max_thrust)
    assert forces[2] > forces[0] and forces[3] > forces[1]
    assert np.allclose(controller.allocation @ forces, [total * 9.81, 0, 0, 0])


def test_x650_study_mass_curve_and_propeller_spacing():
    config = yaml.safe_load((ROOT / 'config/x650_study.yaml').read_text())
    curve = ThrustCurve(ROOT / config['thrust_curve_csv'], config['max_continuous_throttle'])
    base = urdf_mass_kg(ROOT / 'x500_mount_v6/urdf/x500_v2_sprayer.urdf')
    added, _, _ = added_mass_properties(config['mass_components'])
    risers = sum(float(row['mass_kg']) for row in csv.DictReader(
        (ROOT / 'x500_mount_v6/mass_breakdown.csv').open())
        if row['link'] == 'landing_risers')
    study_mass = base + added - risers
    assert np.isclose(study_mass, 4.632162, atol=1e-5)
    assert np.isclose(curve.force(.65), 1.150 * 9.81)
    assert np.isclose(curve.current_a(.75), 7.8)
    controller = FlightController(config, study_mass, (0, 0, -.03), curve)
    assert controller.motors[0] < .70
    rotor = np.asarray(config['rotor_positions_m'])
    adjacent = np.linalg.norm(rotor[0] - rotor[1])
    assert adjacent - config['propeller_diameter_m'] > config['rotor_disk_clearance_margin_m']


def test_x650_native_urdf_uses_cad_frame_and_its_own_mass_budget():
    config = yaml.safe_load((ROOT / 'config/x650_flight.yaml').read_text())
    urdf = ROOT / config['model_urdf']
    tree = ET.parse(urdf)
    links = {link.get('name') for link in tree.findall('link')}
    assert tree.getroot().get('name') == 'x650_compact_sprayer'
    assert {'base_link','rail_mount','sprayer_body','battery_6s','spray_tip'} <= links
    assert 'landing_risers' not in links
    frame_mesh = tree.find("./link[@name='base_link']/visual/geometry/mesh")
    assert frame_mesh.get('filename') == '../meshes/x650_frame.stl'
    assert (urdf.parent / frame_mesh.get('filename')).is_file()
    assert np.isclose(urdf_mass_kg(urdf), 4.63216208085)
    assert not config.get('mass_components')  # included in the imported URDF
