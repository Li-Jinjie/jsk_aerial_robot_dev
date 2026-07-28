import numpy as np
import transformations as tf


def skew(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def quaternion_wxyz_to_matrix(quaternion):
    qw, qx, qy, qz = quaternion
    return tf.quaternion_matrix([qw, qx, qy, qz])[:3, :3]


class DragonNMPCReferenceGenerator:
    """Allocate a body wrench to four three-dimensional module forces."""

    def __init__(self, nmpc, positions, link_quaternions, mass, gravity):
        self.nmpc = nmpc
        self.mass = mass
        self.gravity = gravity
        self.previous_angles = np.zeros(8)
        self.update_geometry(positions, link_quaternions)

    def update_geometry(self, positions, link_quaternions):
        self.positions = np.asarray(positions, dtype=float).reshape(4, 3)
        self.link_rotations = [
            quaternion_wxyz_to_matrix(quaternion) for quaternion in np.asarray(link_quaternions).reshape(4, 4)
        ]
        blocks = []
        for position, rotation in zip(self.positions, self.link_rotations):
            blocks.append(np.vstack((np.eye(3), skew(position))) @ rotation)
        self.alloc_mat = np.concatenate(blocks, axis=1)
        self.alloc_mat_pinv = np.linalg.pinv(self.alloc_mat)

    def get_alloc_mat(self):
        return self.alloc_mat

    def get_alloc_mat_pinv(self):
        return self.alloc_mat_pinv

    @staticmethod
    def force_to_gimbal(force_link, previous_angles):
        thrust = np.linalg.norm(force_link)
        if thrust < 1.0e-9:
            return 0.0, np.asarray(previous_angles, dtype=float)

        fx, fy, fz = force_link
        transverse_norm = np.hypot(fy, fz)
        pitch = np.arctan2(fx, transverse_norm)
        if transverse_norm < 1.0e-8:
            roll = previous_angles[0]
        else:
            roll = np.arctan2(-fy, fz)

        primary = np.array([roll, pitch])
        alternate = np.array([roll + np.pi, np.pi - pitch])
        candidates = []
        for candidate in (primary, alternate):
            wrapped = (candidate + np.pi) % (2.0 * np.pi) - np.pi
            candidates.append(wrapped)
        angles = min(candidates, key=lambda value: np.linalg.norm(value - previous_angles))
        return thrust, angles

    def allocate_wrench(self, target_wrench):
        link_forces = (self.alloc_mat_pinv @ np.asarray(target_wrench, dtype=float).reshape(6)).reshape(4, 3)
        thrust = np.zeros(4)
        angles = np.zeros(8)
        for index, force_link in enumerate(link_forces):
            thrust[index], angles[2 * index : 2 * index + 2] = self.force_to_gimbal(
                force_link, self.previous_angles[2 * index : 2 * index + 2]
            )
        self.previous_angles = angles.copy()
        return thrust, angles

    def compute_trajectory(self, target_xyz, target_rpy, estimated_wrench=None):
        target_xyz = np.asarray(target_xyz, dtype=float).reshape(3)
        target_rpy = np.asarray(target_rpy, dtype=float).reshape(3)
        quaternion = tf.quaternion_from_euler(*target_rpy, axes="sxyz")
        target_qwxyz = np.asarray(quaternion, dtype=float)

        if estimated_wrench is None:
            estimated_wrench = np.zeros(6)
        estimated_wrench = np.asarray(estimated_wrench, dtype=float).reshape(6)

        rotation_wb = tf.quaternion_matrix(target_qwxyz)[:3, :3]
        actuator_force_w = np.array([0.0, 0.0, self.mass * self.gravity]) - estimated_wrench[:3]
        target_wrench = np.concatenate((rotation_wb.T @ actuator_force_w, -estimated_wrench[3:]))
        thrust, angles = self.allocate_wrench(target_wrench)
        return self.nmpc.get_reference(target_xyz, target_qwxyz, thrust, angles)
