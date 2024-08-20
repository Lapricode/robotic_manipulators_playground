import numpy as np


def dh_transformation_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(q, dh_params):
    num_joints = len(q)
    T = T_base
    for i in range(num_joints):
        a, alpha, d, theta = dh_params[i][0], dh_params[i][1], dh_params[i][2], q[i] + dh_params[i][3]
        T_i = dh_transformation_matrix(a, alpha, d, theta)
        T = T @ T_i
    return T

def forward_kinematics2(q):
    q1, q2, q3, q4, q5, q6 = q[0], q[1], q[2], q[3], q[4], q[5]
    T03 = np.array([
        [np.cos(q1)*np.cos(q2+q3), -np.sin(q1), np.cos(q1)*np.sin(q2+q3), 0.160*np.cos(q1)*np.sin(q2)],
        [np.sin(q1)*np.cos(q2+q3), np.cos(q1), np.sin(q1)*np.sin(q2+q3), 0.160*np.sin(q1)*np.sin(q2)],
        [-np.sin(q2+q3), 0, np.cos(q2+q3), 0.160*np.cos(q2)],
        [0, 0, 0, 1]
    ])
    T36 = np.array([
        [np.cos(q4)*np.cos(q5)*np.cos(q6)-np.sin(q4)*np.sin(q6), \
        -np.cos(q4)*np.cos(q5)*np.sin(q6)-np.sin(q4)*np.cos(q6), np.cos(q4)*np.sin(q5), 0.067*np.cos(q4)*np.sin(q5)],
        [np.sin(q4)*np.cos(q5)*np.cos(q6)+np.cos(q4)*np.sin(q6), \
        -np.sin(q4)*np.cos(q5)*np.sin(q6)+np.cos(q4)*np.cos(q6), np.sin(q4)*np.sin(q5), 0.067*np.sin(q4)*np.sin(q5)],
        [-np.sin(q5)*np.cos(q6), np.sin(q5)*np.sin(q6), np.cos(q5), 0.194+0.067*np.cos(q5)],
        [0, 0, 0, 1]
    ])
    return T_base @ T03 @ T36 @ T_end_effector

def forward_kinematics3(q):
    q1, q2, q3, q4, q5, q6 = q[0], q[1], q[2], q[3], q[4], q[5]
    A01 = np.array([[np.cos(q1), 0, -np.sin(q1), 0], [np.sin(q1), 0, np.cos(q1), 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    A12 = np.array([[np.sin(q2), np.cos(q2), 0, 0.160*np.sin(q2)], [-np.cos(q2), np.sin(q2), 0, -0.160*np.cos(q2)], [0, 0, 1, 0], [0, 0, 0, 1]])
    A23 = np.array([[-np.sin(q3), 0, np.cos(q3), 0], [np.cos(q3), 0, np.sin(q3), 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    A34 = np.array([[np.cos(q4), 0, -np.sin(q4), 0], [np.sin(q4), 0, np.cos(q4), 0], [0, -1, 0, 0.194], [0, 0, 0, 1]])
    A45 = np.array([[np.cos(q5), 0, np.sin(q5), 0], [np.sin(q5), 0, -np.cos(q5), 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    A56 = np.array([[np.cos(q6), -np.sin(q6), 0, 0], [np.sin(q6), np.cos(q6), 0, 0], [0, 0, 1, 0.067], [0, 0, 0, 1]])
    return T_base @ A01 @ A12 @ A23 @ A34 @ A45 @ A56 @ T_end_effector

def differential_kinematics(q):
    q1, q2, q3, q4, q5, q6 = q[0], q[1], q[2], q[3], q[4], q[5]
    p0 = np.array([0, 0, 0])
    p1 = p0
    p2 = np.array([0.160*np.cos(q1)*np.sin(q2), 0.160*np.sin(q1)*np.sin(q2), 0.160*np.cos(q2)])
    p3 = p2
    p4 = np.array([0.160*np.cos(q1)*np.sin(q2) + 0.194*np.cos(q1)*np.sin(q2+q3), 0.160*np.sin(q1)*np.sin(q2) + 0.194*np.sin(q1)*np.sin(q2+q3), 0.160*np.cos(q2) + 0.194*np.cos(q2+q3)])
    p5 = p4
    p6 = np.array([0.067*np.cos(q1)*np.cos(q2+q3)*np.cos(q4)*np.sin(q5) - 0.067*np.sin(q1)*np.sin(q4)*np.sin(q5) + 0.160*np.cos(q1)*np.sin(q2) + (0.194+0.067*np.cos(q5))*np.cos(q1)*np.sin(q2+q3), \
        0.067*np.sin(q1)*np.cos(q2+q3)*np.cos(q4)*np.sin(q5) + 0.067*np.cos(q1)*np.sin(q4)*np.sin(q5) + 0.160*np.sin(q1)*np.sin(q2) + (0.194+0.067*np.cos(q5))*np.sin(q1)*np.sin(q2+q3), \
        -0.067*np.sin(q2+q3)*np.cos(q4)*np.sin(q5) + 0.067*np.cos(q2+q3)*np.cos(q5) + 0.160*np.cos(q2) + 0.194*np.cos(q2+q3)])
    z0 = np.array([0, 0, 1])
    z1 = np.array([-np.sin(q1), np.cos(q1), 0])
    z2 = z1
    z3 = np.array([np.cos(q1)*np.sin(q2+q3), np.sin(q1)*np.sin(q2+q3), np.cos(q2+q3)])
    z4 = np.array([-np.cos(q1)*np.cos(q2+q3)*np.sin(q4) - np.sin(q1)*np.cos(q4), -np.sin(q1)*np.cos(q2+q3)*np.sin(q4) + np.cos(q1)*np.cos(q4), np.sin(q2+q3)*np.sin(q4)])
    z5 = np.array([np.cos(q1)*np.cos(q2+q3)*np.cos(q4)*np.sin(q5) - np.sin(q1)*np.sin(q4)*np.sin(q5) + np.cos(q1)*np.sin(q2+q3)*np.cos(q5), \
        np.sin(q1)*np.cos(q2+q3)*np.cos(q4)*np.sin(q5) + np.cos(q1)*np.sin(q4)*np.sin(q5) + np.sin(q1)*np.sin(q2+q3)*np.cos(q5), \
        -np.sin(q2+q3)*np.cos(q4)*np.sin(q5) + np.cos(q2+q3)*np.cos(q5)])
    J = np.array([np.concatenate((np.cross(z0, p6-p0), z0)), np.concatenate((np.cross(z1, p6-p1), z1)), np.concatenate((np.cross(z2, p6-p2), z2)), np.concatenate((np.cross(z3, p6-p3), z3)), \
        np.concatenate((np.cross(z4, p6-p4), z4)), np.concatenate((np.cross(z5, p6-p5), z5))]).T
    return J

dh_parameters = np.array([
    [0, -np.pi/2, 0, 0],
    [0.160, 0, 0, -np.pi/2],
    [0, np.pi/2, 0, np.pi/2],
    [0, -np.pi/2, 0.194, 0],
    [0, np.pi/2, 0, 0],
    [0, 0, 0.067, 0]
])

T_base = np.eye(4)
T_base[2][-1] = 0.203
T_end_effector = np.eye(4)
T_end_effector[2][-1] = 0.1

fk_initial = [0, 0, 0, 0, 0, 0]
fk_vars = [45, 90, 30, 60, -30, 0]
fk_vars2 = [10, 20, 30, 40, 50, 60]
q = np.deg2rad(fk_vars)

print("Forward Kinematics:")
T_end_effector_first = forward_kinematics(q, dh_parameters)
print(np.round(T_end_effector_first, 3))
T_end_effector_second = forward_kinematics2(q)
print(np.round(T_end_effector_second, 3))
T_end_effector_third = forward_kinematics3(q)
print(np.round(T_end_effector_third, 3))

print("\nEnd - Effector Position:")
q1, q2, q3, q4, q5, q6 = q[0], q[1], q[2], q[3], q[4], q[5]
pe = np.array([0.067*np.cos(q1)*np.cos(q2+q3)*np.cos(q4)*np.sin(q5) - 0.067*np.sin(q1)*np.sin(q4)*np.sin(q5) + 0.160*np.cos(q1)*np.sin(q2) + (0.194+0.067*np.cos(q5))*np.cos(q1)*np.sin(q2+q3), \
    0.067*np.sin(q1)*np.cos(q2+q3)*np.cos(q4)*np.sin(q5) + 0.067*np.cos(q1)*np.sin(q4)*np.sin(q5) + 0.160*np.sin(q1)*np.sin(q2) + (0.194+0.067*np.cos(q5))*np.sin(q1)*np.sin(q2+q3), \
    -0.067*np.sin(q2+q3)*np.cos(q4)*np.sin(q5) + 0.067*np.cos(q2+q3)*np.cos(q5) + 0.160*np.cos(q2) + 0.194*np.cos(q2+q3)])
pe = (T_base @ np.array([pe[0], pe[1], pe[2], 1]))[:3]
print(np.round(pe, 3))

print("\nDifferential Kinematics:")
J = differential_kinematics(q)
print(np.round(J, 3))