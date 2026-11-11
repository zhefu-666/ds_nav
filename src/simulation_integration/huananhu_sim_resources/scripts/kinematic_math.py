"""Ideal motion mapping. All hardware-unknown choices live in config/model.json."""
import math
import numpy as np


def rotation(axis, angle):
    n = np.asarray(axis, dtype=float)
    n = n / np.linalg.norm(n)
    x, y, z = n
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def smooth_step(current, target, max_rate, dt):
    return current + max(-max_rate * dt, min(max_rate * dt, target - current))


def joint_positions(config, fold, yaw1, yaw2, aim, wheels=None):
    """Aim is relative to captured head attitude; compensate the folding rotation."""
    q = dict(wheels or {})
    q.update(yaw1_joint=yaw1, yaw2_joint=yaw2,
             pitch1_drive_joint=fold, pitch1_rocker_joint=fold,
             pitch1_coupler_joint=-fold, uwb_level_joint=-fold,
             pitch2_joint=aim - fold,
             yaw1_motor_joint=yaw1, yaw2_motor_joint=yaw2)
    axis = np.asarray(config['mechanism']['axis'])
    pivot = np.asarray(config['mechanism']['A'])
    fold_rotation = rotation(axis, fold)
    for spring in config['gas_springs']:
        fixed = np.asarray(spring['fixed'])
        initial_moving = np.asarray(spring['moving'])
        moving = pivot + fold_rotation @ (initial_moving - pivot)
        delta = moving - fixed
        length = np.linalg.norm(delta)
        direction = delta / length
        initial = np.asarray(spring['direction'])
        angle = math.atan2(float(axis @ np.cross(initial, direction)),
                           float(initial @ direction))
        q[spring['hinge_joint']] = angle
        q[spring['slide_joint']] = float(length - spring['initial_length'])
    for wheel in config['wheels']:
        q.setdefault(wheel['joint'], 0.0)
    return q


def wheel_velocities(config, vx, vy, wz):
    up = np.array([0.0, 0.0, 1.0])
    result = {}
    for wheel in config['wheels']:
        axis = np.asarray(wheel['axis_ros'])
        center = np.asarray(wheel['center_ros'])
        rolling = np.cross(axis, up)
        velocity = np.array([vx, vy, 0.0]) + np.cross(up * wz, center)
        result[wheel['joint']] = float(rolling @ velocity / wheel['radius'])
    return result


def forward_kinematics(config, positions):
    """Link poses in the model frame. Tree order is generated parent-first."""
    world = {'base_link': np.eye(4)}
    for joint in config['joints']:
        pose = np.array(joint['origin_matrix'], dtype=float)
        motion = np.eye(4)
        if joint['type'] == 'prismatic':
            motion[:3, 3] = np.asarray(joint['axis']) * positions.get(joint['name'], 0.0)
        elif joint['type'] != 'fixed':
            motion[:3, :3] = rotation(joint['axis'], positions.get(joint['name'], 0.0))
        world[joint['child']] = world[joint['parent']] @ pose @ motion
    return world


def demo_targets(elapsed):
    """36 second repeat: rise, scan, forward, strafe, turn, fold, rest."""
    t = elapsed % 36.0
    stand = math.radians(81)
    fold = stand if 3 <= t < 27 else 0.0
    yaw1 = 0.35 * math.sin((t - 6) * 0.8) if 6 <= t < 12 else 0.0
    yaw2 = 0.6 * math.sin((t - 6) * 1.0) if 6 <= t < 12 else 0.0
    aim = 0.12 * math.sin((t - 6) * 0.9) if 6 <= t < 12 else 0.0
    twist = (0.15, 0.0, 0.0) if 12 <= t < 16 else \
            (0.0, 0.15, 0.0) if 16 <= t < 20 else \
            (-0.15, -0.15, 0.0) if 20 <= t < 24 else \
            (0.0, 0.0, 0.25) if 24 <= t < 27 else (0.0, 0.0, 0.0)
    return fold, yaw1, yaw2, aim, twist
