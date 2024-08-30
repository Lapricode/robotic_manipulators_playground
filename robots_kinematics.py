import numpy as np
import matplotlib.pyplot as plt
import itertools
import general_functions as gf

# for the kinematics of the robotic arm
# DH parameters are in a matrix form where each row represents a joint and the columns represent the parameters a, alpha, d, theta respectively
# the robot must be an rtb.DHRobot object
# world_base_T is the transformation matrix from the world to the base frame of coordinates
# base_0_T is the transformation matrix from the base to the 0 frame of coordinates
# n_end_effector_T is the transformation matrix from the n to the end-effector frame of coordinates

def dh_transformation_matrix(a, alpha, d, theta, joint_type, q):  # calculate the transformation matrix for the specified Denavit-Hartenberg parameters
    a = float(a); alpha = float(alpha); d = float(d); theta = float(theta)  # convert the DH parameters to floats
    if joint_type == "revolute":  # if the joint is revolute
        theta += q  # add the joint angle to the theta parameter
    elif joint_type == "prismatic":  # if the joint is prismatic
        d += q  # add the joint displacement to the d parameter
    return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                    [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                    [0, np.sin(alpha), np.cos(alpha), d],
                    [0.0, 0.0, 0.0, 1.0]], dtype = float)  # return the transformation matrix

def compute_forward_kinematics(robot, q_joints):  # calculate the forward kinematics of the robotic arm using roboticstoolbox
    return robot.fkine(np.array(q_joints).reshape(-1,))  # return the forward kinematics of the robotic arm model

def compute_forward_kinematics_2(world_base_T, base_0_T, n_end_effector_T, DH_parameters, joints_types, q_joints):  # calculate the forward kinematics of the robotic arm defined by the DH parameters
    DH_parameters = np.array(DH_parameters, dtype = float)  # convert the DH parameters to a numpy array
    a = DH_parameters[0, :].reshape((-1,)); alpha = DH_parameters[1, :].reshape((-1,)); d = DH_parameters[2, :].reshape((-1,)); theta = DH_parameters[3, :].reshape((-1,))  # reshape the DH parameters vectors
    T = np.array(world_base_T, dtype = float) @ np.array(base_0_T, dtype = float)  # initialize the total transformation matrix T
    for k in range(len(q_joints)):
        Ak = dh_transformation_matrix(a[k], alpha[k], d[k], theta[k], joints_types[k], q_joints[k])  # calculate the transformation matrix from the k to the k+1 frame of coordinates
        T = T @ Ak  # calculate the transformation matrix from the world to the k+1 frame of coordinates
    T = T @ np.array(n_end_effector_T, dtype = float)  # calculate the final transformation matrix from the world to the end-effector frame of coordinates
    return T  # return the total transformation matrix from the world to the end-effector frame of coordinates

def compute_forward_kinematics_until_frame(robot, world_base_T, base_0_T, n_end_effector_T, DH_parameters, joints_types, q_joints, until_frame = "end-effector"):  # calculate the partial forward kinematics of the robotic arm until the specified frame
    DH_parameters = np.array(DH_parameters, dtype = float)  # convert the DH parameters to a numpy array
    a = DH_parameters[0, :].reshape((-1,)); alpha = DH_parameters[1, :].reshape((-1,)); d = DH_parameters[2, :].reshape((-1,)); theta = DH_parameters[3, :].reshape((-1,))  # reshape the DH parameters vectors
    if until_frame == "base":  # if the transformation matrix is calculated until the base frame of coordinates
        return np.array(world_base_T, dtype = float)  # return the total transformation matrix from the world to the base frame of coordinates
    elif "frame" in until_frame:  # if the transformation matrix is calculated until a specific frame of coordinates
        T = np.array(world_base_T, dtype = float) @ np.array(base_0_T, dtype = float)  # initialize the total transformation matrix T
        until_frame = int(until_frame.split(" ")[1])  # extract the frame number that corresponds to the frame, it can be 1, 2, ..., n
        for k in range(until_frame):  # iterate through the frames until the specified frame
            Ak = dh_transformation_matrix(a[k], alpha[k], d[k], theta[k], joints_types[k], q_joints[k])  # calculate the transformation matrix from the k to the k+1 frame of coordinates
            T = T @ Ak  # calculate the transformation matrix from the world to the k+1 frame of coordinates
        return T  # return the total transformation matrix from the world to the specified frame of coordinates
    elif until_frame == "end-effector":  # if the transformation matrix is calculated until the end-effector frame of coordinates
        T = compute_forward_kinematics_2(world_base_T, base_0_T, n_end_effector_T, DH_parameters, joints_types, q_joints)  # calculate the forward kinematics of the robotic arm
        # T = compute_forward_kinematics(robot, q_joints)  # calculate the forward kinematics of the robotic arm using roboticstoolbox
        return T  # return the total transformation matrix from the world to the end-effector frame of coordinates

def compute_forward_kinematics_all_frames(world_base_T, base_0_T, n_end_effector_T, DH_parameters, joints_types, q_joints):  # calculate the partial forward kinematics of the robotic arm for all the important frames
    DH_parameters = np.array(DH_parameters, dtype = float)  # convert the DH parameters to a numpy array
    a = DH_parameters[0, :].reshape((-1,)); alpha = DH_parameters[1, :].reshape((-1,)); d = DH_parameters[2, :].reshape((-1,)); theta = DH_parameters[3, :].reshape((-1,))  # reshape the DH parameters vectors
    frames_list = [np.array(world_base_T, dtype = float), np.array(world_base_T, dtype = float) @ np.array(base_0_T, dtype = float)]  # a list to store the calculated frames
    T = frames_list[-1]  # initialize the total transformation matrix T
    for k in range(len(q_joints)):  # iterate through the frames until the specified frame
        Ak = dh_transformation_matrix(a[k], alpha[k], d[k], theta[k], joints_types[k], q_joints[k])  # calculate the transformation matrix from the k to the k+1 frame of coordinates
        T = T @ Ak  # calculate the transformation matrix from the world to the k+1 frame of coordinates
        frames_list.append(T)  # append the calculated frame to the list
    frames_list.append(T @ np.array(n_end_effector_T, dtype = float))  # calculate the final transformation matrix from the world to the end-effector frame of coordinates and append it to the list
    return frames_list  # return the list of the total transformation matrices from the world to all the important frames

def compute_inverse_kinematics(robot, end_effector_desired_pose_T, tol_error = 1e-8):  # calculate the inverse kinematics of the robotic arm using roboticstoolbox
    # This function uses ik_LM solver that is a Fast Levenberg-Marquadt Numerical Inverse Kinematics Solver, a fast solver implemented in C++
    # Other solvers are ik_NR that uses Newton-Raphson optimization method and ik_GN that uses Gauss-Newton optimization method
    invkine_solver = robot.ik_LM(Tep = np.array(end_effector_desired_pose_T, dtype = float), tol = tol_error)  # calculate the inverse kinematics using the Levenberg-Marquadt solver
    q_joints = invkine_solver[0]  # extract the joints configuration
    invkine_success = invkine_solver[1]  # extract the success flag, which is 1 if the solver converged to a solution and 0 otherwise
    return q_joints, invkine_success  # return the joints configuration and the success flag

def compute_differential_kinematics(robot, q_joints, q_dot_joints, wrt_frame = "world"):  # calculate the differential kinematics of the robotic arm using roboticstoolbox
    J0 = np.array(robot.jacob0(q_joints))  # calculate the geometric Jacobian matrix of the robotic arm wrt the world frame
    if wrt_frame == "world":  # if the Jacobian is calculated wrt the world frame
        return (J0 @ np.array(q_dot_joints).reshape(-1, 1)).reshape(-1,)  # return the end-effector velocities in the world frame
    elif wrt_frame == "end-effector":  # if the Jacobian is calculated wrt the end-effector frame
        Je = np.array(robot.jacobe(q_joints))  # calculate the geometric Jacobian matrix of the robotic arm wrt the end-effector frame
        # T_fkine = compute_forward_kinematics(robot, q_joints)  # calculate the forward kinematics of the robotic arm
        # Je = np.block([[T_fkine[:3, :3].T, np.zeros((3, 3))], [np.zeros((3, 3)), T_fkine[:3, :3].T]]) @ J0  # calculate the geometric Jacobian matrix of the robotic arm wrt the end-effector frame
        return (Je @ np.array(q_dot_joints).reshape(-1, 1)).reshape(-1,)  # return the end-effector velocities in the end-effector frame

def compute_inverse_differential_kinematics(robot, q_joints, end_effector_velocity, wrt_frame = "world"):  # calculate the inverse differential kinematics of the robotic arm
    J0 = np.array(robot.jacob0(q_joints))  # calculate the geometric Jacobian matrix of the robotic arm wrt the world frame
    J0_is_full_rank = (np.linalg.matrix_rank(J0) == min(J0.shape))  # check if the Jacobian matrix is full rank
    if J0_is_full_rank:  # if the Jacobian matrix is full rank
        if wrt_frame == "world":  # if the Jacobian is calculated wrt the world frame
            J0_pinv = np.linalg.pinv(J0)  # calculate the pseudo-inverse of the Jacobian matrix
            return (J0_pinv @ np.array(end_effector_velocity).reshape(-1, 1)).reshape(-1,), J0_is_full_rank  # return the joints velocities in the world frame
        elif wrt_frame == "end-effector":  # if the Jacobian is calculated wrt the end-effector frame
            Je = np.array(robot.jacobe(q_joints))  # calculate the geometric Jacobian matrix of the robotic arm wrt the end-effector frame
            Je_pinv = np.linalg.pinv(Je)  # calculate the pseudo-inverse of the Jacobian matrix
            return (Je_pinv @ np.array(end_effector_velocity).reshape(-1, 1)).reshape(-1,), J0_is_full_rank  # return the joints velocities in the end-effector frame
    else:  # if the Jacobian matrix is not full rank
        return np.zeros(len(q_joints)), J0_is_full_rank  # return zero joints velocities

def find_reachable_workspace(robot, divs = 5, show_plots = False):  # find the reachable workspace of the robotic arm, i.e. the positions that the end-effector can reach in the 3D space no matter the orientation of the end-effector frame
    joints_num = len(robot.q)  # get the number of joints of the robotic arm
    q_range = [(robot.qlim[1][k] - robot.qlim[0][k]) for k in range(joints_num)]  # calculate the range of the joints
    min_q = [robot.qlim[0][k] for k in range(joints_num)]  # calculate the minimum joints values
    reachable_pos = []  # initialize the list to store the reachable end-effector positions
    for indices in itertools.product(range(divs + 1), repeat = joints_num):  # iterate through the joints configurations
        q = np.array([min_q[k] + indices[k] * q_range[k] / divs for k in range(joints_num)])  # calculate the current joint configuration
        pos = np.array(robot.fkine(q), dtype = float)[:3, 3]  # calculate the current end-effector position
        if pos[2] >= -1e-3:  # if the end-effector position is above the xy-plane, meaning it is above the ground
            reachable_pos.append(pos)  # append the reachable end-effector position
    if show_plots:
        x_rp, y_rp, z_rp = [], [], []
        if len(reachable_pos) != 0:  x_rp, y_rp, z_rp = zip(*reachable_pos)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = "3d")
        ax.scatter(x_rp, y_rp, z_rp, c = "g", marker = ".", linewidths = 0.1, edgecolors = "k")
        ax.set_title(f"\nReachable workspace ({len(reachable_pos)} positions are shown)")
        ax.legend(["Reachable positions"])
        ax.grid(True)
        ax.set_xlim([min(x_rp) - 0.2 * abs(min(x_rp)), max(x_rp) + 0.2 * abs(max(x_rp))]); ax.set_ylim([min(y_rp) - 0.2 * abs(min(y_rp)), max(y_rp) + 0.2 * abs(max(y_rp))]); ax.set_zlim([min(z_rp) - 0.2 * abs(min(z_rp)), max(z_rp) + 0.2 * abs(max(z_rp))])
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)"); ax.set_aspect("equal"); ax.view_init(elev = 45, azim = 45)
        plt.show()
    else:
        pass
    return reachable_pos  # return the reachable end-effector positions

def find_kinematic_singularities_on_plane(robot, plane_x_range, plane_y_range, plane_T_rev, divs_x = 10, divs_y = 10, singul_bound = 1e-3, invkine_tol = 1e-3, show_plots = False):  # find the kinematic singularities of the robotic arm
    # plane_T_rev is the transformation matrix for the obstacles plane, but the normal vector is reversed (i.e. pointing away from the robot's end-effector)
    singular_q = []; non_singular_q = []  # initialize the lists to store the singular and non-singular joints configurations
    singular_pos = []; non_singular_pos = []  # initialize the lists to store the singular and non-singular end-effector positions
    reachable_pos = []; non_reachable_pos = []  # initialize the lists to store the reachable and non-reachable end-effector positions
    initial_plane = [[[x, y, 0, 1] for y in np.linspace(-plane_y_range/2, plane_y_range/2, divs_y)] for x in np.linspace(-plane_x_range/2, plane_x_range/2, divs_x)]  # create the initial plane points
    plane_T_rev_new = np.array(plane_T_rev, dtype = float)  # create a new transformation matrix for the plane
    for i in range(divs_x):  # iterate through the x-axis of the plane
        for j in range(divs_y):  # iterate through the y-axis of the plane
            pos_transformed = np.array(plane_T_rev_new @ np.array(initial_plane[i][j], dtype = float)).reshape(-1, 1)  # transform the plane points
            end_effector_desired_pose = np.hstack((plane_T_rev_new[:, :-1], pos_transformed))  # create the desired end-effector pose
            q, invkine_success = compute_inverse_kinematics(robot, end_effector_desired_pose, invkine_tol)  # calculate the inverse kinematics of the robotic arm
            # invkine_success = False  # initialize the flag for the success of the inverse kinematics solver
            # for _ in range(0, 360, 1):  # iterate through the various plane rotations
            #     q, invkine_success = compute_inverse_kinematics(robot, end_effector_desired_pose, invkine_tol)  # calculate the inverse kinematics of the robotic arm
            #     if invkine_success: break  # if the inverse kinematics solver converged to a solution, break the loop
            #     R_rot = np.array([[np.cos(np.deg2rad(1)), -np.sin(np.deg2rad(1)), 0, 0], [np.sin(np.deg2rad(1)), np.cos(np.deg2rad(1)), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype = float)  # create the rotation matrix for the plane
            #     plane_T_rev_new = plane_T_rev_new @ R_rot  # update the plane transformation matrix
            J = robot.jacob0(q)  # calculate the geometric Jacobian matrix of the robotic arm wrt the world frame
            if invkine_success:  # if the inverse kinematics solver converged to a solution
                if pos_transformed[2] >= -1e-3:  # if the end-effector position is above the xy-plane, meaning it is above the ground
                    reachable_pos.append(pos_transformed[:-1])  # append the reachable end-effector position
                    # if the Jacobian matrix is square and its determinant is less than the specified singularity bound, or the Jacobian matrix is non square and not full rank, a singular configuration has been detected
                    if (J.shape[0] == J.shape[1] and np.abs(np.linalg.det(J)) <= singul_bound) or (J.shape[0] != J.shape[1] and np.linalg.matrix_rank(J) != min(J.shape)):
                        singular_q.append(q)  # append the singular joint configuration
                        singular_pos.append(pos_transformed[:-1])  # append the singular end-effector position
                    else:
                        non_singular_q.append(q)  # append the non-singular joint configuration
                        non_singular_pos.append(pos_transformed[:-1])  # append the non-singular end-effector position
                else:  # if the end-effector position is below the xy-plane, meaning it is below the ground
                    non_reachable_pos.append(pos_transformed[:-1])  # append the non-reachable end-effector position
            else:  # if the inverse kinematics solver did not converge to a solution
                non_reachable_pos.append(pos_transformed[:-1])  # append the non-reachable end-effector position
    if show_plots:  # if the plots are requested to be shown
        x_ns, y_ns, z_ns = [], [], []; x_s, y_s, z_s = [], [], []; x_rp, y_rp, z_rp = [], [], []; x_nrp, y_nrp, z_nrp = [], [], []
        if len(non_singular_pos) != 0:  x_ns, y_ns, z_ns = zip(*non_singular_pos)
        if len(singular_pos) != 0:  x_s, y_s, z_s = zip(*singular_pos)
        if len(reachable_pos) != 0:  x_rp, y_rp, z_rp = zip(*reachable_pos)
        if len(non_reachable_pos) != 0:  x_nrp, y_nrp, z_nrp = zip(*non_reachable_pos)
        fig = plt.figure()
        ax1 = fig.add_subplot(111, projection = "3d")
        ax1.scatter(x_s, y_s, z_s, c = "r", marker = ".")
        ax1.scatter(x_ns, y_ns, z_ns, c = "g", marker = ".")
        ax1.scatter(x_nrp, y_nrp, z_nrp, c = "k", marker = ".")
        ax1.set_title(f"Reachable points = {len(reachable_pos)}: Non-singular points = {len(non_singular_pos)}, Singular points = {len(singular_pos)}\nNon reachable points = {len(non_reachable_pos)}\n(Jacobian determinant -if it exists- upper bound: {singul_bound})")
        ax1.legend(["Singular poses", "Non-singular poses", "Non-reachable poses"])
        ax1.grid(True)
        plane_pos = np.array(plane_T_rev @ np.array([0, 0, 0, 1], dtype = float)).reshape(-1, 1)[:-1]
        ax1.set_xlim([plane_pos[0] - 0.6 * plane_x_range, plane_pos[0] + 0.6 * plane_x_range]); ax1.set_ylim([plane_pos[1] - 0.6 * plane_y_range, plane_pos[1] + 0.6 * plane_y_range]); ax1.set_zlim([plane_pos[2] - 0.6 * min(plane_x_range, plane_y_range), plane_pos[2] + 0.6 * max(plane_x_range, plane_y_range)])
        ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_zlabel("z (m)"); ax1.set_aspect("equal"); ax1.view_init(elev = 0, azim = 0)
        plt.show()
        # non_singular_points = [non_singular_pos[k][:3, 3] for k in range(len(non_singular_pos))]
        # singular_points = [singular_pos[k][:3, 3] for k in range(len(singular_pos))]
        # x_ns, y_ns, z_ns = zip(*non_singular_points)
        # x_s, y_s, z_s = zip(*singular_points)
        # fig = plt.figure()
        # ax1 = fig.add_subplot(121, projection = "3d")
        # ax1.scatter(x_ns, y_ns, z_ns, c = "g", marker = ".")
        # ax1.set_title(f"Non-singular points ({len(non_singular_points)})")
        # ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_zlabel("z (m)"); ax1.set_aspect("equal"); ax1.view_init(elev = 0, azim = 0)
        # ax2 = fig.add_subplot(122, projection = "3d")
        # ax2.scatter(x_s, y_s, z_s, c = "r", marker = ".")
        # ax2.set_title(f"Singular points ({len(singular_points)})\nuppper bound: {singul_bound}")
        # ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)"); ax2.set_zlabel("z (m)"); ax2.set_aspect("equal"); ax2.view_init(elev = 0, azim = 0)
        # plt.show()
    else:
        pass
    return non_singular_q, singular_q  # return the non-singular and singular joints configurations

def find_kinematic_singularities_3d_space(robot, divs = 5, singul_bound = 1e-3, show_plots = False):  # find the kinematic singularities of the robotic arm
    joints_num = len(robot.q)  # get the number of joints of the robotic arm
    q_range = [(robot.qlim[1][k] - robot.qlim[0][k]) for k in range(joints_num)]  # calculate the range of the joints
    min_q = [robot.qlim[0][k] for k in range(joints_num)]  # calculate the minimum joints values
    singular_q = []; non_singular_q = []  # initialize the lists to store the singular and non-singular joints configurations
    singular_pos = []; non_singular_pos = []  # initialize the lists to store the singular and non-singular end-effector positions
    for indices in itertools.product(range(divs + 1), repeat = joints_num):  # iterate through the joints configurations
        q = np.array([min_q[k] + indices[k] * q_range[k] / divs for k in range(joints_num)])  # calculate the current joint configuration
        pos = np.array(robot.fkine(q), dtype = float)[:3, 3]  # calculate the current end-effector position
        J = robot.jacob0(q)  # calculate the geometric Jacobian matrix of the robotic arm wrt the world frame
        if pos[2] >= -1e-3:  # if the end-effector position is above the xy-plane, meaning it is above the ground
            # if the Jacobian matrix is square and its determinant is less than the specified singularity bound, or the Jacobian matrix is non square and not full rank, a singular configuration has been detected
            if (J.shape[0] == J.shape[1] and np.abs(np.linalg.det(J)) <= singul_bound) or (J.shape[0] != J.shape[1] and np.linalg.matrix_rank(J) != min(J.shape)):
                singular_q.append(q)  # append the singular joint configuration
                singular_pos.append(pos)  # append the singular end-effector position
            else:
                non_singular_q.append(q)  # append the non-singular joint configuration
                non_singular_pos.append(pos)  # append the non-singular end-effector position
    if show_plots:
        x_ns, y_ns, z_ns, x_s, y_s, z_s = [], [], [], [], [], []
        if len(non_singular_pos) != 0:  x_ns, y_ns, z_ns = zip(*non_singular_pos)
        if len(singular_pos) != 0:  x_s, y_s, z_s = zip(*singular_pos)
        fig = plt.figure()
        ax1 = fig.add_subplot(121, projection = "3d")
        ax1.scatter(x_ns, y_ns, z_ns, c = "g", marker = ".", linewidths = 0.1, edgecolors = "k")
        ax1.set_title(f"Non-singular points = {len(non_singular_pos)}")
        ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_zlabel("z (m)"); ax1.set_aspect("equal"); ax1.view_init(elev = 0, azim = 0)
        ax2 = fig.add_subplot(122, projection = "3d")
        ax2.scatter(x_s, y_s, z_s, c = "r", marker = ".", linewidths = 0.1, edgecolors = "k")
        ax2.set_title(f"Singular points = {len(singular_pos)}\n(Jacobian determinant -if it exists- upper bound: {singul_bound})")
        ax2.set_xlabel("x (m)"); ax2.set_ylabel("y (m)"); ax2.set_zlabel("z (m)"); ax2.set_aspect("equal"); ax2.view_init(elev = 0, azim = 0)
        plt.show()
    else:
        pass
    return non_singular_q, singular_q  # return the non-singular and singular joints configurations

def convert_xy_plane_to_end_effector_velocities(robot, xy_velocities, plane_T, wrt_frame = "world"):  # convert the xy world plane velocities to the given plane velocities and then to end-effector velocitiess
    # xy_velocities is a list of velocities in the xy world plane
    p_dot = np.array(xy_velocities).reshape(-1, len(xy_velocities))  # convert the xy world plane velocities to a numpy array
    pe_dot = np.array(plane_T[:3, :3].T @ p_dot).reshape(-1, len(xy_velocities))  # convert the xy plane velocities to the given plane velocities
    J0 = np.array(robot.jacob0(robot.q))  # calculate the geometric Jacobian matrix of the robotic arm wrt the world frame
    if wrt_frame == "world":  # if the Jacobian is calculated wrt the world frame
        return (J0 @ np.vstack((pe_dot, np.zeros((3, len(xy_velocities)))))).reshape(-1, len(xy_velocities))  # return the end-effector velocities in the world frame
    elif wrt_frame == "end-effector":  # if the Jacobian is calculated wrt the end-effector frame
        Je = np.array(robot.jacobe(robot.q))  # calculate the geometric Jacobian matrix of the robotic arm wrt the end-effector frame
        return (Je @ np.vstack((pe_dot, np.zeros((3, len(xy_velocities)))))).reshape(-1, len(xy_velocities))  # return the end-effector velocities in the end-effector frame


# import os
# import roboticstoolbox as rtb
# import spatialmath as sm

# thor_file = os.getcwd() + r"/robots_models_descriptions/thor_description/urdf/thor.urdf"
# robot = rtb.Robot.URDF(thor_file)
# robot.base = sm.SE3(0.0, 0.0, 0.0) #* sm.SE3.Rz(np.pi/2)

# find_reachable_workspace(robot, divs = 10, show_plots = True)
# # find_kinematic_singularities_3d_space(robot, divs = 7, singul_bound = 1e-3, show_plots = True)

# phi_orient = 0
# normal_vector = np.array([-1, 0, -1])
# theta = np.rad2deg(np.arctan2(normal_vector[2], normal_vector[0])) % 180
# normal_vector = -normal_vector / np.linalg.norm(normal_vector)
# h1 = 0.50; x1 = 0.30
# origin_pos = np.array([x1, 0.0, h1])
# h2 = 0.47; x2 = x1 + (h1 - h2) / np.tan(np.deg2rad(theta))
# origin_pos = np.array([x2, 0.0, h2])
# print(origin_pos)
# plane_T_rev = gf.xy_plane_transformation(normal_vector, phi_orient, origin_pos)
# # origin_pos = np.array([0.30, 0.0, 0.5])
# # origin_pos = np.array([0.34, 0.0, 0.46])

# singul_bound = 1e-3
# invkine_tol = 1e-7
# divs_x, divs_y = 50, 50
# plane_x_range, plane_y_range = 0.3, 0.3
# find_kinematic_singularities_on_plane(robot, plane_x_range, plane_y_range, plane_T_rev, divs_x, divs_y, singul_bound, invkine_tol, show_plots = True)