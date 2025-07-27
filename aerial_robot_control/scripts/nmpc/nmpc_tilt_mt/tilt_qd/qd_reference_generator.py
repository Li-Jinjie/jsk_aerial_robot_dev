import numpy as np
import transformations as tf


class QDNMPCReferenceGenerator:
    """
    Class to generate reference trajectories.

    Parameters are physical properties set in robot's .yaml config as well as OCP dimensions.
    :param nmpc: NMPC controller object
    """

    def __init__(self, nmpc, p1_b, p2_b, p3_b, p4_b, dr1, dr2, dr3, dr4, kq_d_kt, mass, gravity):
        self.nmpc = nmpc

        self.p1_b = p1_b
        self.p2_b = p2_b
        self.p3_b = p3_b
        self.p4_b = p4_b
        self.dr1 = dr1
        self.dr2 = dr2
        self.dr3 = dr3
        self.dr4 = dr4
        self.kq_d_kt = kq_d_kt

        self.mass = mass
        self.gravity = gravity

        # Compute Allocation Matrix
        self._compute_alloc_mat()
        self._compute_alloc_mat_pinv()

        self.a_ref_prev = [0.0, 0.0, 0.0, 0.0]

    def get_alloc_mat(self):
        return self.alloc_mat

    def get_alloc_mat_pinv(self):
        return self.alloc_mat_pinv

    def _compute_alloc_mat(self):
        p1_b, p2_b, p3_b, p4_b = self.p1_b, self.p2_b, self.p3_b, self.p4_b
        dr1, dr2, dr3, dr4 = self.dr1, self.dr2, self.dr3, self.dr4
        kq_d_kt = self.kq_d_kt

        # Abbreviations
        sqrt_p1b_xy = np.sqrt(self.p1_b[0] ** 2 + self.p1_b[1] ** 2)
        sqrt_p2b_xy = np.sqrt(self.p2_b[0] ** 2 + self.p2_b[1] ** 2)
        sqrt_p3b_xy = np.sqrt(self.p3_b[0] ** 2 + self.p3_b[1] ** 2)
        sqrt_p4b_xy = np.sqrt(self.p4_b[0] ** 2 + self.p4_b[1] ** 2)

        # Define Allocation Matrix
        self.alloc_mat = np.zeros((6, 8))

        # - Force
        self.alloc_mat[0, 0] = p1_b[1] / sqrt_p1b_xy
        self.alloc_mat[1, 0] = -p1_b[0] / sqrt_p1b_xy
        self.alloc_mat[2, 1] = 1

        self.alloc_mat[0, 2] = p2_b[1] / sqrt_p2b_xy
        self.alloc_mat[1, 2] = -p2_b[0] / sqrt_p2b_xy
        self.alloc_mat[2, 3] = 1

        self.alloc_mat[0, 4] = p3_b[1] / sqrt_p3b_xy
        self.alloc_mat[1, 4] = -p3_b[0] / sqrt_p3b_xy
        self.alloc_mat[2, 5] = 1

        self.alloc_mat[0, 6] = p4_b[1] / sqrt_p4b_xy
        self.alloc_mat[1, 6] = -p4_b[0] / sqrt_p4b_xy
        self.alloc_mat[2, 7] = 1

        # - Torque
        self.alloc_mat[3, 0] = -dr1 * kq_d_kt * p1_b[1] / sqrt_p1b_xy + p1_b[0] * p1_b[2] / sqrt_p1b_xy
        self.alloc_mat[4, 0] = dr1 * kq_d_kt * p1_b[0] / sqrt_p1b_xy + p1_b[1] * p1_b[2] / sqrt_p1b_xy
        self.alloc_mat[5, 0] = -p1_b[0] ** 2 / sqrt_p1b_xy - p1_b[1] ** 2 / sqrt_p1b_xy

        self.alloc_mat[3, 1] = p1_b[1]
        self.alloc_mat[4, 1] = -p1_b[0]
        self.alloc_mat[5, 1] = -dr1 * kq_d_kt

        self.alloc_mat[3, 2] = -dr2 * kq_d_kt * p2_b[1] / sqrt_p2b_xy + p2_b[0] * p2_b[2] / sqrt_p2b_xy
        self.alloc_mat[4, 2] = dr2 * kq_d_kt * p2_b[0] / sqrt_p2b_xy + p2_b[1] * p2_b[2] / sqrt_p2b_xy
        self.alloc_mat[5, 2] = -p2_b[0] ** 2 / sqrt_p2b_xy - p2_b[1] ** 2 / sqrt_p2b_xy

        self.alloc_mat[3, 3] = p2_b[1]
        self.alloc_mat[4, 3] = -p2_b[0]
        self.alloc_mat[5, 3] = -dr2 * kq_d_kt

        self.alloc_mat[3, 4] = -dr3 * kq_d_kt * p3_b[1] / sqrt_p3b_xy + p3_b[0] * p3_b[2] / sqrt_p3b_xy
        self.alloc_mat[4, 4] = dr3 * kq_d_kt * p3_b[0] / sqrt_p3b_xy + p3_b[1] * p3_b[2] / sqrt_p3b_xy
        self.alloc_mat[5, 4] = -p3_b[0] ** 2 / sqrt_p3b_xy - p3_b[1] ** 2 / sqrt_p3b_xy

        self.alloc_mat[3, 5] = p3_b[1]
        self.alloc_mat[4, 5] = -p3_b[0]
        self.alloc_mat[5, 5] = -dr3 * kq_d_kt

        self.alloc_mat[3, 6] = -dr4 * kq_d_kt * p4_b[1] / sqrt_p4b_xy + p4_b[0] * p4_b[2] / sqrt_p4b_xy
        self.alloc_mat[4, 6] = dr4 * kq_d_kt * p4_b[0] / sqrt_p4b_xy + p4_b[1] * p4_b[2] / sqrt_p4b_xy
        self.alloc_mat[5, 6] = -p4_b[0] ** 2 / sqrt_p4b_xy - p4_b[1] ** 2 / sqrt_p4b_xy

        self.alloc_mat[3, 7] = p4_b[1]
        self.alloc_mat[4, 7] = -p4_b[0]
        self.alloc_mat[5, 7] = -dr4 * kq_d_kt

    def _compute_alloc_mat_pinv(self):
        self.alloc_mat_pinv = np.linalg.pinv(self.alloc_mat)

    @staticmethod
    def _ensure_servo_angles_continuity(a_ref: np.array, a_ref_prev: np.array) -> np.array:
        """
        Ensure that the servo angles are continuous by checking the difference
        between the current and previous reference angles.
        If the difference exceeds pi, adjust the angle to maintain continuity.
        """
        for i in range(len(a_ref)):
            if a_ref[i] - a_ref_prev[i] > np.pi:
                a_ref[i] -= 2 * np.pi
            elif a_ref[i] - a_ref_prev[i] < -np.pi:
                a_ref[i] += 2 * np.pi
        return a_ref

    def compute_trajectory(self, target_xyz, target_rpy):
        """
        Convert current target pose to a reference trajectory over the entire horizon.
        Compute target quaternions and control reference from a target rotation and then
        get assembled reference trajectories from controller file.

        :param target_xyz: Target position
        :param target_rpy: Target orientation (roll, pitch, yaw)
        :return xr: Reference for the state x
        :return ur: Reference for the input u
        """
        if len(target_xyz) != 3 or len(target_rpy) != 3:
            raise ValueError("Target state should be given in xyz and rpy.")
        roll = target_rpy[0]
        pitch = target_rpy[1]
        yaw = target_rpy[2]

        qwxyz = tf.quaternion_from_euler(roll, pitch, yaw, axes="sxyz")
        target_qwxyz = np.array([qwxyz]).T

        # Convert [0,0,gravity] to Body frame
        q_inv = tf.quaternion_conjugate(qwxyz)
        rot_inv = tf.quaternion_matrix(q_inv)
        fg_w = np.array([0, 0, self.mass * self.gravity, 0])  # World frame
        fg_b = rot_inv @ fg_w  # Body frame
        target_wrench = np.array([[fg_b.item(0), fg_b.item(1), fg_b.item(2), 0, 0, 0]]).T

        # A faster method if alloc_mat is dynamic:  x, _, _, _ = np.linalg.lstsq(alloc_mat, target_wrench, rcond=None)
        target_force = self.alloc_mat_pinv @ target_wrench

        # Compute reference values for thrust
        # Set either state or control input based on model properties, i.e., based on include flags
        ft1_ref = np.sqrt(target_force[0, 0] ** 2 + target_force[1, 0] ** 2)
        ft2_ref = np.sqrt(target_force[2, 0] ** 2 + target_force[3, 0] ** 2)
        ft3_ref = np.sqrt(target_force[4, 0] ** 2 + target_force[5, 0] ** 2)
        ft4_ref = np.sqrt(target_force[6, 0] ** 2 + target_force[7, 0] ** 2)
        ft_ref = [ft1_ref, ft2_ref, ft3_ref, ft4_ref]

        # Compute reference values for servo angles
        # Set either state or control input based on model properties, i.e., based on include flags
        a1_ref = np.arctan2(target_force[0, 0], target_force[1, 0])
        a2_ref = np.arctan2(target_force[2, 0], target_force[3, 0])
        a3_ref = np.arctan2(target_force[4, 0], target_force[5, 0])
        a4_ref = np.arctan2(target_force[6, 0], target_force[7, 0])
        a_ref = [a1_ref, a2_ref, a3_ref, a4_ref]

        # Ensure servo angles are continuous
        a_ref = self._ensure_servo_angles_continuity(a_ref, self.a_ref_prev)
        self.a_ref_prev = a_ref

        # Assemble reference trajectories in controller file since their definition is
        # closely related to the cost function
        xr, ur = self.nmpc.get_reference(target_xyz, target_qwxyz, ft_ref, a_ref)

        return xr, ur
