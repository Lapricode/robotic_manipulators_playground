import numpy as np


# definitions and implementations of some general purpose functions

def xyz_to_R(x, y, z):  # convert the xyz extrinsic Euler angles (measured in radians) to the corresponding rotation matrix R
    x = float(x); y = float(y); z = float(z)  # convert the Euler angles (measured in radians) to floats
    # Rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]], dtype = float)  # the rotation matrix for the rotation around the x-axis (the third rotation)
    # Rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]], dtype = float)  # the rotation matrix for the rotation around the z-axis (the first rotation)
    # Ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]], dtype = float)  # the rotation matrix for the rotation around the y-axis (the second rotation)
    # R = Rz @ Ry @ Rx  # calculate the total rotation matrix
    R = np.array([[np.cos(y)*np.cos(z), np.sin(x)*np.sin(y)*np.cos(z) - np.cos(x)*np.sin(z), np.cos(x)*np.sin(y)*np.cos(z) + np.sin(x)*np.sin(z)], \
                    [np.cos(y)*np.sin(z), np.sin(x)*np.sin(y)*np.sin(z) + np.cos(x)*np.cos(z), np.cos(x)*np.sin(y)*np.sin(z) - np.sin(x)*np.cos(z)], \
                    [-np.sin(y), np.sin(x)*np.cos(y), np.cos(x)*np.cos(y)]], dtype = float)  # calculate the rotation matrix
    return R  # return the 3x3 rotation matrix

def R_to_xyz(R, y_angle_case = None):  # convert the rotation matrix R to the corresponding xyz extrinsic Euler angles (measured in radians)
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]  # the elements of the first row of the rotation matrix
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]  # the elements of the second row of the rotation matrix
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]  # the elements of the third row of the rotation matrix
    if np.abs(r31 - 1.0) <= 1e-6:  # if the y extrinsic Euler angle is 90 degrees
        z = 0.0  # the z extrinsic Euler angle is set to be 0 degrees
        y = np.pi / 2  # the y extrinsic Euler angle is set to be 90 degrees
        x = np.arctan2(r12, r22)  # calculate the x extrinsic Euler angle (roll)
    elif np.abs(r31 + 1.0) <= 1e-6:  # if the y extrinsic Euler angle is -90 degrees
        z = 0.0  # the z extrinsic Euler angle is set to be 0 degrees
        y = -np.pi / 2  # the y extrinsic Euler angle is set to be -90 degrees
        x = np.arctan2(-r12, r22)  # calculate the x extrinsic Euler angle (roll)
    else:  # if the y extrinsic Euler angle is not 90 or -90 degrees
        if y_angle_case == 1 or y_angle_case == None:  # if the y extrinsic Euler angle is between -90 and 90 degrees or it is not defined
            x = np.arctan2(r32, r33)  # calculate the x extrinsic Euler angle (roll)
            y = np.arctan2(-r31, np.sqrt(r32**2 + r33**2))  # calculate the y extrinsic Euler angle (pitch)
            z = np.arctan2(r21, r11)  # calculate the z extrinsic Euler angle (yaw)
        elif y_angle_case == 2:  # if the y extrinsic Euler angle is between 90 and 270 degrees
            x = np.arctan2(-r32, -r33)  # calculate the x extrinsic Euler angle (roll)
            y = np.arctan2(-r31, -np.sqrt(r32**2 + r33**2))  # calculate the y extrinsic Euler angle (pitch)
            z = np.arctan2(-r21, -r11)  # calculate the z extrinsic Euler angle (yaw)
    return x, y, z  # return the xyz extrinsic Euler angles (roll, pitch, yaw)

def q_to_R(q):  # convert the quaternion q to the corresponding rotation matrix R
    q = np.array(q).reshape(-1,)  # reshape the quaternion q
    s = float(q[0]); v1 = float(q[1]); v2 = float(q[2]); v3 = float(q[3])  # extract the elements of the quaternion q
    r11 = 2. * (s**2 + v1**2) - 1.; r12 = 2. * (v1 * v2 - s * v3); r13 = 2. * (v1 * v3 + s * v2)  # the first row of the rotation matrix
    r21 = 2. * (v1 * v2 + s * v3); r22 = 2. * (s**2 + v2**2) - 1.; r23 = 2. * (v2 * v3 - s * v1)  # the second row of the rotation matrix
    r31 = 2. * (v1 * v3 - s * v2); r32 = 2. * (v2 * v3 + s * v1); r33 = 2. * (s**2 + v3**2) - 1.  # the third row of the rotation matrix
    return np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])  # return the 3x3 rotation matrix

def R_to_q(R):  # convert the rotation matrix R to the corresponding quaternion q
    pass

def xy_plane_transformation(normal_vector, phi_orient, origin_pos):  # calculate the transformation of an xy plane to the plane defined by the normal vector, the orientation angle, and the origin position
    nx, ny, nz = np.array(normal_vector).reshape(-1,) / np.linalg.norm(np.array(normal_vector))  # the components of the normal vector of the plane
    origin_pos = np.array(origin_pos).reshape(-1,)  # the components of the origin position of the plane
    phi_orient = np.deg2rad(float(phi_orient))  # the z orientation of the plane
    div_factor = np.sqrt(nx**2 + ny**2)  # the division factor of the normal vector of the plane
    R_orient = np.array([[np.cos(phi_orient), -np.sin(phi_orient), 0, 0], [np.sin(phi_orient), np.cos(phi_orient), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # the rotation matrix for the z orientation of the plane
    T_transl = np.array([[1, 0, 0, origin_pos[0]], [0, 1, 0, origin_pos[1]], [0, 0, 1, origin_pos[2]], [0, 0, 0, 1]])  # the translation matrix of the plane
    R_normallign = np.eye(4)  # initialize the alignment matrix of the plane
    if div_factor != 0.0:  # if the normal vector of the plane is not parallel to the z-axis
        R_normallign = np.array([[ny / div_factor, nx * nz / div_factor, nx, 0], [-nx / div_factor, ny * nz / div_factor, ny, 0], [0, -(nx**2 + ny**2) / div_factor, nz, 0], [0, 0, 0, 1]])  # the alignment matrix of the plane
    elif nz == 1.0:  # if the normal vector of the plane points to the z-axis
        R_normallign = np.eye(4)  # the alignment matrix of the plane
    elif nz == -1.0:  # if the normal vector of the plane points to the negative z-axis
        R_normallign = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])  # the alignment matrix of the plane
    T_total = T_transl @ R_normallign @ R_orient  # the total transformation matrix of the plane
    return T_total  # return the total transformation matrix of the plane

def get_components_from_xy_plane_transformation(T_xy):  # extract the normal vector, the orientation angle, and the origin position from the transformation matrix of an xy plane
    origin_pos = T_xy[:3, 3]  # get the origin position in the world frame in meters
    normal_vec = T_xy[:3, :3] @ np.array([0, 0, 1])  # get the normal vector in the world frame
    orientation_R = np.linalg.inv(xy_plane_transformation(normal_vec, 0, np.zeros(3)))[:3, :3] @ T_xy[:3, :3]  # get the orientation matrix (around the z axis) in the world frame
    orientation_angle = np.rad2deg(np.arctan2(orientation_R[1, 0], orientation_R[0, 0]))  # get the orientation angle in degrees
    return normal_vec, orientation_angle, origin_pos  # return the normal vector, the orientation angle, and the origin position of the plane

# def xy_plane_rot_transformation(normal_vector, phi_orient):  # calculate the transformation of the xy plane to the plane defined by the normal vector and the orientation angle
#     nx, ny, nz = np.array(normal_vector).reshape(-1,) / np.linalg.norm(np.array(normal_vector))  # the components of the normal vector of the plane
#     phi_orient = np.deg2rad(float(phi_orient))  # the z orientation of the plane
#     div_factor = np.sqrt(nx**2 + ny**2)  # the division factor of the normal vector of the plane
#     R_orient = np.array([[np.cos(phi_orient), -np.sin(phi_orient), 0, 0], [np.sin(phi_orient), np.cos(phi_orient), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # the rotation matrix for the z orientation of the plane
#     R_normallign = np.eye(4)  # initialize the alignment matrix of the plane
#     if div_factor != 0.0:  # if the normal vector of the plane is not parallel to the z-axis
#         R_normallign = np.array([[ny / div_factor, nx * nz / div_factor, nx, 0], [-nx / div_factor, ny * nz / div_factor, ny, 0], [0, -(nx**2 + ny**2) / div_factor, nz, 0], [0, 0, 0, 1]])  # the alignment matrix of the plane
#     elif nz == 1.0:  # if the normal vector of the plane points to the z-axis
#         R_normallign = np.eye(4)  # the alignment matrix of the plane
#     elif nz == -1.0:  # if the normal vector of the plane points to the negative z-axis
#         R_normallign = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])  # the alignment matrix of the plane
#     T_total = R_normallign @ R_orient  # the total transformation matrix of the plane
#     return T_total  # return the total transformation matrix of the plane