"""Nominal rigid-square DRAGON parameters used for code generation and tests.

The online C++ controller replaces mass, inertia, rotor positions, and link
orientations with values from FullVectoringRobotModel every control cycle.
"""

import numpy as np

mass = 8.016
gravity = 9.798

Ixx = 0.30140782
Iyy = 0.30009625
Izz = 0.58298614
Ixy = -0.01471238
Ixz = -0.00125958
Iyz = -0.00142017

p_b = [
    [-0.24989105, 0.04392697, 0.03497693],
    [-0.03789105, -0.21957303, 0.03497693],
    [0.22560895, -0.00757303, 0.03497693],
    [0.01360895, 0.25592697, 0.03497693],
]

# Quaternion of each link frame in the CoG/body frame, in wxyz order.
sqrt_half = np.sqrt(0.5)
q_bl = [
    [sqrt_half, 0.0, 0.0, -sqrt_half],
    [1.0, 0.0, 0.0, 0.0],
    [sqrt_half, 0.0, 0.0, sqrt_half],
    [0.0, 0.0, 0.0, 1.0],
]

t_rotor = 0.01
t_servo = 0.08

physical_param_list = [
    mass,
    gravity,
    Ixx,
    Iyy,
    Izz,
    Ixy,
    Ixz,
    Iyz,
]
for rotor_position, link_quaternion in zip(p_b, q_bl):
    physical_param_list.extend(rotor_position)
    physical_param_list.extend(link_quaternion)
physical_param_list.extend([t_rotor, t_servo])
