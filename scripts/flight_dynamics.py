"""Quadrotor force allocation and feedback control for the imported X500."""
from __future__ import annotations

import math
import csv
import xml.etree.ElementTree as ET
import numpy as np


def urdf_mass_kg(path):
    tree = ET.parse(path)
    return sum(float(elem.attrib['value']) for elem in tree.iter('mass'))


def added_mass_properties(components):
    """Point-mass approximation for omitted equipment in base_link axes."""
    masses = np.array([float(item['mass_kg']) for item in components])
    positions = np.array([item['position_m'] for item in components], dtype=float)
    if np.any(masses <= 0) or positions.shape != (len(masses), 3):
        raise ValueError('Invalid airframe mass budget')
    mass = float(masses.sum())
    com = np.average(positions, axis=0, weights=masses)
    inertia = sum(m * (np.dot(p-com, p-com) * np.eye(3) - np.outer(p-com, p-com))
                  for m, p in zip(masses, positions))
    return mass, com, inertia


class ThrustCurve:
    def __init__(self, path, max_throttle=0.90):
        with open(path, newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.throttle = np.array([0.0] + [float(r['throttle']) for r in rows])
        self.thrust = np.array([0.0] + [float(r['thrust_g']) * 9.81 / 1000 for r in rows])
        self.torque = np.array([0.0] + [float(r['torque_nm']) for r in rows])
        self.current = np.array([0.0] + [float(r['current_a']) for r in rows])
        if np.any(np.diff(self.throttle) <= 0) or np.any(np.diff(self.thrust) <= 0):
            raise ValueError('Thrust calibration must be strictly increasing')
        self.max_throttle = float(max_throttle)
        if not self.throttle[0] < self.max_throttle <= self.throttle[-1]:
            raise ValueError('Continuous throttle cap is outside the measured curve')
        self.max_thrust = self.force(self.max_throttle)

    def force(self, throttle):
        return np.interp(throttle, self.throttle, self.thrust)

    def reaction_torque(self, throttle):
        return np.interp(throttle, self.throttle, self.torque)

    def current_a(self, throttle):
        return np.interp(throttle, self.throttle, self.current)

    def throttle_for_force(self, force):
        return np.interp(force, self.thrust, self.throttle)


def rotation_from_wxyz(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def allocation_matrix(rotor_positions, spin_signs, yaw_reaction):
    pos = np.asarray(rotor_positions, dtype=float)
    spin = np.asarray(spin_signs, dtype=float)
    if pos.shape != (4, 3) or spin.shape != (4,):
        raise ValueError('Expected four rotor positions and spin signs')
    matrix = np.stack((np.ones(4), pos[:, 1], -pos[:, 0], yaw_reaction * spin))
    if abs(np.linalg.det(matrix)) < 1e-8:
        raise ValueError('Rotor allocation matrix is singular')
    return matrix


class FlightController:
    def __init__(self, config, mass_kg, center_of_mass_m=(0, 0, 0), curve=None):
        self.mass = float(mass_kg)
        self.g = float(config['gravity_m_s2'])
        self.rotor_positions = np.asarray(config['rotor_positions_m'], dtype=float)
        self.center_of_mass = np.asarray(center_of_mass_m, dtype=float)
        if curve is None:
            raise ValueError('Measured bench thrust curve required')
        self.curve = curve
        self.max_thrust = curve.max_thrust
        hover_throttle = curve.throttle_for_force(self.mass * self.g / 4)
        self.yaw_reaction = curve.reaction_torque(hover_throttle) / (self.mass * self.g / 4)
        self.spin = np.asarray(config['rotor_spin_signs'], dtype=float)
        self.allocation = allocation_matrix(self.rotor_positions - self.center_of_mass,
                                            self.spin, self.yaw_reaction)
        self.kp = np.asarray(config['position_kp_s2'], dtype=float)
        self.kd = np.asarray(config['velocity_kd_s'], dtype=float)
        self.kr = np.asarray(config['attitude_kp_nm'], dtype=float)
        self.kw = np.asarray(config['angular_velocity_kd_nms'], dtype=float)
        self.max_hacc = float(config['max_horizontal_acceleration_m_s2'])
        self.max_vacc = float(config['max_vertical_acceleration_m_s2'])
        self.max_tilt = math.radians(float(config['max_tilt_deg']))
        self.motor_tau = float(config['motor_time_constant_s'])
        if self.mass <= 0 or self.mass * self.g >= 4 * self.max_thrust:
            raise ValueError('Modeled drone cannot hover within configured thrust limit')
        self.motors = np.full(4, hover_throttle)

    def command(self, position, quaternion_wxyz, velocity, angular_velocity_world, target, dt,
                max_horizontal_speed=None):
        pos = np.asarray(position, dtype=float)
        vel = np.asarray(velocity, dtype=float)
        target = np.asarray(target, dtype=float)
        rotation = rotation_from_wxyz(quaternion_wxyz)
        desired_acc = self.kp * (target - pos) - self.kd * vel
        if max_horizontal_speed is not None:
            desired_velocity = self.kp[:2] * (target[:2] - pos[:2]) / self.kd[:2]
            speed = np.linalg.norm(desired_velocity)
            if speed > max_horizontal_speed:
                desired_velocity *= max_horizontal_speed / speed
            desired_acc[:2] = self.kd[:2] * (desired_velocity - vel[:2])
        horizontal = np.linalg.norm(desired_acc[:2])
        if horizontal > self.max_hacc:
            desired_acc[:2] *= self.max_hacc / horizontal
        desired_acc[2] = np.clip(desired_acc[2], -self.max_vacc, self.max_vacc)
        desired_acc[2] += self.g
        max_horizontal = desired_acc[2] * math.tan(self.max_tilt)
        horizontal = np.linalg.norm(desired_acc[:2])
        if horizontal > max_horizontal:
            desired_acc[:2] *= max_horizontal / horizontal
        # Preserve 8% collective thrust for attitude corrections near the cap.
        max_collective_acc = .92 * 4 * self.max_thrust / self.mass
        magnitude = np.linalg.norm(desired_acc)
        if magnitude > max_collective_acc:
            desired_acc *= max_collective_acc / magnitude
        desired_z = desired_acc / np.linalg.norm(desired_acc)
        desired_y = np.cross(desired_z, (1.0, 0.0, 0.0))
        desired_y /= np.linalg.norm(desired_y)
        desired_x = np.cross(desired_y, desired_z)
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))
        attitude_error_matrix = desired_rotation.T @ rotation - rotation.T @ desired_rotation
        attitude_error = .5 * np.array((attitude_error_matrix[2, 1],
                                        attitude_error_matrix[0, 2],
                                        attitude_error_matrix[1, 0]))
        body_rates = rotation.T @ np.asarray(angular_velocity_world, dtype=float)
        torque = -self.kr * attitude_error - self.kw * body_rates
        thrust = self.mass * max(0.0, np.dot(desired_acc, rotation[:, 2]))
        requested = np.linalg.solve(self.allocation, np.r_[thrust, torque])
        requested = np.clip(requested, 0, self.max_thrust)
        requested_throttle = self.curve.throttle_for_force(requested)
        alpha = 1 - math.exp(-dt / self.motor_tau)
        self.motors += alpha * (requested_throttle - self.motors)
        return self.curve.force(self.motors)
