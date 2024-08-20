import numpy as np
import requests
import matplotlib.pyplot as plt


# define the class hntf2d_control_law for calculating the control law using the harmonic potential field navigation function in 2D transformed space
class hntf2d_control_law:
    def __init__(self, p_init, p_d, outer_boundary, inner_boundaries, k_d, k_i, K, w_phi, gamma, e_p, e_v):
        self.p_init = np.array(p_init)  # the start position on the real workspace
        self.p_d = np.array(p_d)  # the target position on the real workspace
        self.outer_boundary = np.array(outer_boundary)  # convert the outer boundary to a numpy array
        self.inner_boundaries = [np.array(inner_boundaries[i]) for i in range(len(inner_boundaries))]  # convert the inner boundaries to numpy arrays
        self.unit_circle = None  # the unit circle on the transformed workspace (unit disk)
        self.q_init = None  # the start position on the transformed workspace
        self.q_d = None  # the target position on the transformed workspace
        self.q_i = None  # the obstacles punctures positions on the transformed workspace (unit disk with punctures or R2 plane with punctures)
        self.q_init_disk = None  # store the start position from the unit disk
        self.q_d_disk = None  # store the target position from the unit disk
        self.q_i_disk = None  # store the obstacles punctures positions from the unit disk        
        self.k_d = k_d  # the target gain kd for the target position qd on the transformed workspace
        self.k_i = k_i  # the obstacles gains kis for the obstacles punctures positions qis on the transformed workspace
        self.K = K  # the scalar gain K for the control law
        self.w_phi = w_phi  # the scalar constant w_phi for the function psi
        self.gamma = gamma  # the scalar constant gamma for the function s
        self.e_p = e_p  # the scalar constant e_p for the function sigmap
        self.e_v = e_v  # the scalar constant e_v for the function sigmav
        self.wtd_transformation_is_built = False  # the flag to check if the world to disk transformation has been done
        self.dtp_transformation_is_built = False  # the flag to check if the disk to R2 transformation has been done
    def create_new_harmonic_map_object(self):  # create a new HarmonicMap2D object
        ssh_url = "http://127.0.0.1:5000/create_new_harmonic_map_object"  # the SSH URL to communicate with the virtual machine
        ssh_response = requests.post(ssh_url, json = {}).json()  # the SSH response from the virtual machine
        print(ssh_response["message"])  # print the message from the SSH response
    def calculate_realws_to_disk_transformation(self, plot_map = True, keep_plot_on_screen = False):  # calculate the harmonic transformation that maps the real workspace to the sphere world
        # the outer boundary goes to the unit circle, the inner boundaries go to discrete punctures
        if not self.wtd_transformation_is_built:  # check if the real workspace to disk transformation has not been done yet
            ssh_url = "http://127.0.0.1:5000/calculate_harmonic_transformation"  # the SSH URL to communicate with the virtual machine
            outer_boundary = []; inner_boundaries = []  # initialize the outer boundary and the inner boundaries
            outer_boundary = self.outer_boundary.tolist()  # make sure the outer boundary is a list
            for i in range(len(self.inner_boundaries)): inner_boundaries.append(self.inner_boundaries[i].tolist())  # make sure the inner boundaries are lists
            ssh_response = requests.post(ssh_url, json = {"outer_boundary": outer_boundary, "inner_boundaries": inner_boundaries}).json()  # the SSH response from the virtual machine
            self.unit_circle = np.array(ssh_response["unit_circle"])  # the unit_circle from the SSH response
            if np.linalg.norm(self.unit_circle[0] - self.unit_circle[-1]) != 0.0: self.unit_circle = np.vstack((self.unit_circle, self.unit_circle[0]))  # close the unit circle if it is not closed
            self.q_i = np.array(ssh_response["punctures"])  # the punctures from the SSH response
            self.q_init = self.realws_to_unit_disk_mapping(self.p_init)  # map the start position from the real workspace to the sphere world
            self.q_d = self.realws_to_unit_disk_mapping(self.p_d)  # map the target position from the real workspace to the sphere world
            self.q_init_disk = self.q_init.copy()  # store the start position from the unit disk
            self.q_d_disk = self.q_d.copy()  # store the target position from the unit disk
            self.q_i_disk = self.q_i.copy()  # store the obstacles punctures positions from the unit disk
            self.wtd_transformation_is_built = True  # set the flag to True to indicate that the real workspace to disk transformation has been done
        print("Harmonic transformation from the real workspace to unit disk computed successfully!")  # print a message to the console
        if plot_map:  # plot the transformation map
            text_margin = 0.00  # the margin for the text
            text_fontsize = 8  # the fontsize for the text
            legend_fontsize = 7  # the fontsize for the legend
            outer_boundaries_linewidth = 1  # the line width for the outer boundaries
            inner_boundaries_linewidth = 1  # the line width for the inner boundaries
            positions_fontsize = 5  # the fontsize for the positions
            punctures_fontsize = 4  # the fontsize for the punctures
            fig = plt.figure()
            ax1 = fig.add_subplot(121)
            outer_plot, = ax1.plot(self.outer_boundary[:, 0], self.outer_boundary[:, 1], "red", linewidth = outer_boundaries_linewidth)
            for i in range(len(self.inner_boundaries)):
                inner_plot, = ax1.plot(self.inner_boundaries[i][:, 0], self.inner_boundaries[i][:, 1], "blue", linewidth = inner_boundaries_linewidth)
                ax1.text(self.inner_boundaries[i][0, 0] + text_margin, self.inner_boundaries[i][0, 1] + text_margin, f"{i+1}", fontsize = text_fontsize)
            p_init_plot, = ax1.plot(self.p_init[0], self.p_init[1], "magenta", marker = "s", markersize = positions_fontsize)
            p_d_plot, = ax1.plot(self.p_d[0], self.p_d[1], "green", marker = "s", markersize = positions_fontsize)
            ax1.legend([outer_plot, inner_plot, p_init_plot, p_d_plot], ["Outer boundary", "Inner boundaries", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
            ax1.set_title(f"Original real workspace")
            ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_aspect("equal")
            ax1.grid(True)
            ax2 = fig.add_subplot(122)
            unit_disk_plot, = ax2.plot(self.unit_circle[:, 0], self.unit_circle[:, 1], "red", linewidth = outer_boundaries_linewidth)
            for i in range(len(self.q_i_disk)):
                punctures_plot, = ax2.plot(self.q_i_disk[i][0], self.q_i_disk[i][1], "blue", marker = "o", markersize = punctures_fontsize)
                ax2.text(self.q_i_disk[i][0] + text_margin, self.q_i_disk[i][1] + text_margin, f"{i+1}", fontsize = text_fontsize)
            q_init_plot, = ax2.plot(self.q_init_disk[0], self.q_init_disk[1], "magenta", marker = "s", markersize = positions_fontsize)
            q_d_plot, = ax2.plot(self.q_d_disk[0], self.q_d_disk[1], "green", marker = "s", markersize = positions_fontsize)
            ax2.legend([unit_disk_plot, punctures_plot, q_init_plot, q_d_plot], ["Unit disk", "Punctures", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
            ax2.set_title(f"Real workspace ->\n-> Unit disk transformation")
            ax2.set_xlabel("u"); ax2.set_ylabel("v"); ax2.set_aspect("equal")
            ax2.grid(True)
            plt.show(block = keep_plot_on_screen)
    def calculate_disk_to_R2_transformation(self, plot_map = True, keep_plot_on_screen = False):  # calculate the transformation that maps the sphere world to the R2 plane
        # the unit circle goes to infinity, the punctures positions have to be recalculated
        if self.wtd_transformation_is_built:  # check if the real workspace to disk transformation has been done
            if not self.dtp_transformation_is_built:  # check if the disk to R2 transformation has not been done yet
                for k in range(len(self.q_i)):  # calculate the punctures positions on the R2 plane
                    self.q_i[k] = self.unit_disk_to_R2_mapping(self.q_i[k])  # map the punctures from the unit disk to the R2 plane
                self.q_init = self.unit_disk_to_R2_mapping(self.q_init)  # map the start position from the sphere world to the R2 plane
                self.q_d = self.unit_disk_to_R2_mapping(self.q_d)  # map the target position from the sphere world to the R2 plane
                self.dtp_transformation_is_built = True  # set the flag to True to indicate that the unit disk to R2 transformation has been done
            print("Transformation from the unit disk to the R2 plane computed successfully!")  # print a message to the console
            if plot_map:  # plot the transformation map
                text_margin = 0.00  # the margin for the text
                text_fontsize = 8  # the fontsize for the text
                legend_fontsize = 7  # the fontsize for the legend
                unit_circle_linewidth = 1  # the line width for the unit circle
                positions_fontsize = 5  # the fontsize for the positions
                punctures_fontsize = 4  # the fontsize for the punctures
                points_max_norm = max([np.linalg.norm(self.q_i[k]) for k in range(len(self.q_i))] + [np.linalg.norm(self.q_init), np.linalg.norm(self.q_d)])  # calculate the maximum norm of the points
                fig = plt.figure()
                ax1 = fig.add_subplot(121)
                unit_disk_plot, = ax1.plot(self.unit_circle[:, 0], self.unit_circle[:, 1], "red", linewidth = unit_circle_linewidth)
                for i in range(len(self.q_i_disk)):
                    punctures_plot, = ax1.plot(self.q_i_disk[i][0], self.q_i_disk[i][1], "blue", marker = "o", markersize = punctures_fontsize)
                    ax1.text(self.q_i_disk[i][0] + text_margin, self.q_i_disk[i][1] + text_margin, f"{i+1}", fontsize = text_fontsize)
                q_init_disk_plot, = ax1.plot(self.q_init_disk[0], self.q_init_disk[1], "magenta", marker = "s", markersize = positions_fontsize)
                q_d_disk_plot, = ax1.plot(self.q_d_disk[0], self.q_d_disk[1], "green", marker = "s", markersize = positions_fontsize)
                ax1.legend([unit_disk_plot, punctures_plot, q_init_disk_plot, q_d_disk_plot], ["Unit disk", "Punctures", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                ax1.set_title(f"Real workspace ->\n-> Unit disk transformation")
                ax1.set_xlabel("u"); ax1.set_ylabel("v"); ax1.set_aspect("equal")
                ax1.grid(True)
                ax2 = fig.add_subplot(122)
                for i in range(len(self.q_i)):
                    displayed_punctures_plot, = ax2.plot(self.q_i[i][0], self.q_i[i][1], "blue", marker = "o", markersize = punctures_fontsize)
                    ax2.text(self.q_i[i][0] + text_margin, self.q_i[i][1] + text_margin, f"{i+1}", fontsize = text_fontsize)
                q_init_R2_plot, = ax2.plot(self.q_init[0], self.q_init[1], "magenta", marker = "s", markersize = positions_fontsize)
                q_d_R2_plot, = ax2.plot(self.q_d[0], self.q_d[1], "green", marker = "s", markersize = positions_fontsize)
                ax2.legend([displayed_punctures_plot, q_init_R2_plot, q_d_R2_plot], ["Punctures", "start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                ax2.set_title(f"Unit disk ->\n-> R2 plane transformation")
                ax2.set_xlabel("x"); ax2.set_ylabel("y"); ax2.set_aspect("equal")
                ax2.set_xlim(-1.5 * points_max_norm, 1.5 * points_max_norm); ax2.set_ylim(-1.5 * points_max_norm, 1.5 * points_max_norm)
                ax2.grid(True)
                plt.show(block = keep_plot_on_screen)
    def realws_to_unit_disk_mapping(self, p):  # map the point p from the real workspace to the sphere world (unit disk)
        # p = [px, py] is the point on the real workspace
        p = np.array(p).reshape(-1, 1)  # make sure p is a numpy array
        ssh_url = "http://127.0.0.1:5000/map_point_to_unit_disk"  # the SSH URL to communicate with the virtual machine
        ssh_response = requests.post(ssh_url, json = {"point": p.tolist()}).json()  # the SSH response from the virtual machine
        q = np.array(ssh_response["mapped_point"]).flatten()  # the mapped point q on the sphere world (unit disk)
        return q  # return the mapped point q
    def workspace_to_unit_disk_jacobian(self, p):  # calculate the jacobian of the transformation from the real workspace to the sphere world at the given point p
        # p = [px, py] is the point on the real workspace
        p = np.array(p).reshape(-1, 1)  # make sure p is a numpy array
        ssh_url = "http://127.0.0.1:5000/compute_jacobian_at_point"  # the SSH URL to communicate with the virtual machine
        ssh_response = requests.post(ssh_url, json = {"point": p.tolist()}).json()  # the SSH response from the virtual machine
        jacobian = np.array(ssh_response["jacobian_matrix"])  # the jacobian matrix of the transformation at the point p
        return jacobian  # return the jacobian matrix at the given point p
    def unit_disk_to_R2_mapping(self, p):  # map the point p from the sphere world to the R2 plane
        # p = [px, py] is the point on the sphere world (unit disk)
        p = np.array(p).reshape(-1, 1)  # make sure p is a numpy array
        q = (1.0 / (1.0 - np.linalg.norm(p)) * p).flatten()  # the mapped point q on the R2 plane
        return q  # return the mapped point q
    def unit_disk_to_R2_jacobian(self, p):  # calculate the jacobian of the transformation from the sphere world to the R2 plane at the given point p
        # p = [px, py] is the point on the sphere world (unit disk)
        p = np.array(p).reshape(-1, 1)  # make sure p is a numpy array
        p_norm = np.linalg.norm(p)  # the norm of the point p
        jacobian = ((1.0 - p_norm) * np.eye(2) + (p @ p.T) / p_norm) / (1.0 - p_norm)**2.0  # the jacobian matrix of the transformation at the point p
        return jacobian  # return the jacobian matrix at the given point p
    def harmonic_field_phi(self, q):  # calculate the harmonic potential field phi
        # kd * np.log(np.linalg.norm(q - qd) / max_dist) is the target harmonic potential field phi
        # sum(ki * np.log(np.linalg.norm(q - qi) / max_dist)) is the obstacles harmonic potential field phi
        return self.k_d * np.log(np.linalg.norm(q - self.q_d)) - sum([self.k_i[i] * np.log(np.linalg.norm(q - self.q_i[i])) for i in range(len(self.q_i))])  # return the total harmonic potential field phi
    def harmonic_field_phi_gradient(self, q):  # calculate the gradient of the harmonic potential field phi
        # kd * (q - qd) / np.linalg.norm(q - qd)**2 is the gradient of the target harmonic potential field phi
        # sum(ki * (q - qi) / np.linalg.norm(q - qi)**2) is the gradient of the obstacles harmonic potential field phi
        return self.k_d * (q - self.q_d) / (np.linalg.norm(q - self.q_d))**2.0 - sum([self.k_i[i] * (q - self.q_i[i]) / np.log(np.linalg.norm(q - self.q_i[i]))**2.0 for i in range(len(self.q_i))])  # return the gradient of the harmonic potential field phi
    def psi(self, q):  # calculate the function psi
        return (1.0 + np.tanh(self.harmonic_field_phi(q) / self.w_phi))  # return the function psi
    def psi_gradient(self, q):  # calculate the gradient of the function psi
        return (1.0 - np.tanh(self.harmonic_field_phi(q) / self.w_phi)**2.0) / (2.0 * self.w_phi) @ self.harmonic_field_phi_gradient(q)  # return the gradient of the function psi
    def sigmap_func(self, x):  # calculate the value of the function sigmap
        return x**2.0 * (3.0 - 2.0 * x)  # return the value of the function sigmap
    def sigmav_func(self, x):  # calculate the value of the function sigmav
        return 0.0 if x < 0.0 else x**2.0  # return the value of the function sigmav
    def scalar_gain_s(self, q):  # calculate the scalar gain s
        return (1 - self.gamma) * self.sigmav_func((self.psi_gradient(q) @ q) / self.e_v) + self.gamma * self.sigmap_func(0.0)  # return the scalar gain s
    def control_law(self, p):  # calculate the control law
        # p is the current position of the end-effector on the real workspace
        p = np.array(p)  # make sure q is a numpy array
        q = p
        # q = self.workspace_to_unit_disk_tranformation(p)  # transform the current position p to the sphere world
        s = self.scalar_gain_s(q)
        psi_grad = self.psi_gradient(q)
        obstacles_points = self.workspace_to_unit_disk_tranformation(self.outer_boundary, self.inner_boundaries)
        J_wtd = self.workspace_to_unit_disk_tranformation_jacobian(self.outer_boundary, self.inner_boundaries)
        J_dtp = self.unit_disk_to_R2_tranformation_jacobian(obstacles_points)
        return -self.K * s * np.linalg.inv(J_wtd) @ np.linalg.inv(J_dtp) @  psi_grad  # return the control law
    def plot_control_law(self):  # plot the control law
        # use black line for the control law drawing
        return


# # The outer boundary is oriented counter-clockwise.
# points_num = 100
# t = np.linspace(-1., +1., points_num+1)[:points_num]
# outer_boundary = np.zeros((4*points_num, 2))
# # Outer boundary: line segment ((-1, -1) -> (1, -1))
# outer_boundary[:points_num, 0] = t[:]
# outer_boundary[:points_num, 1] = -1
# # Outer boundary: line segment ((1, -1) -> (1, 1))
# outer_boundary[points_num:2*points_num, 0] = 1
# outer_boundary[points_num:2*points_num, 1] = t[:]
# # Outer boundary: line segment ((1, 1) -> (-1, 1))
# outer_boundary[2*points_num:3*points_num, 0] = -t[:]
# outer_boundary[2*points_num:3*points_num, 1] = 1
# # Outer boundary: line segment ((-1, 1) -> (-1, -1))
# outer_boundary[3*points_num:4*points_num, 0] = -1
# outer_boundary[3*points_num:4*points_num, 1] = -t[:]
# # Outer boundary: first and last vertices must be equal
# outer_boundary[-1, :] = outer_boundary[0, :]

# # Add a box-shaped inner boundary:
# # XXX: Inner boundaries must be clock-wise oriented.
# points_num = 10
# t = np.linspace(-0.1, +0.1, points_num+1)[:points_num]
# inner_boundary = np.zeros((4*points_num, 2))
# # Inner boundary: line segment ((-0.1, -0.1) -> (-0.1, 0.1))
# inner_boundary[:points_num, 0] = -0.1
# inner_boundary[:points_num, 1] = t[:]
# # Inner boundary: line segment ((-0.1, 0.1) -> (0.1, 0.1))
# inner_boundary[points_num:2*points_num, 0] = t[:]
# inner_boundary[points_num:2*points_num, 1] = 0.1
# # Inner boundary: line segment ((0.1, 0.1) -> (0.1, -0.1))
# inner_boundary[2*points_num:3*points_num, 0] = 0.1
# inner_boundary[2*points_num:3*points_num, 1] = -t[:]
# # Inner boundary: line segment ((0.1, -0.1) -> (-0.1, -0.1))
# inner_boundary[3*points_num:4*points_num, 0] = -t[:]
# inner_boundary[3*points_num:4*points_num, 1] = -0.1
# # Inner boundary: first and last vertices must be equal
# inner_boundary[-1, :] = inner_boundary[0, :]

# t = np.linspace(-0.1, +0.1, 41)[:40]
# inner_boundary2 = np.zeros((120, 2))
# # Inner boundary: line segment ((-0.1, -0.1) -> (-0.1, 0.1))
# inner_boundary2[:40, 0] = -0.1
# inner_boundary2[:40, 1] = t[:]
# # Inner boundary: line segment ((-0.1, 0.1) -> (0.1, 0.1))
# inner_boundary2[40:80, 0] = t[:]
# inner_boundary2[40:80, 1] = 0.1
# # Inner boundary: line segment ((0.1, 0.1) -> (0.1, -0.1))
# t = np.linspace(-0.1, +0.1, 21)[:20]
# inner_boundary2[80:100, 0] = 0.1
# inner_boundary2[80:100, 1] = -t[:]
# # Inner boundary: line segment ((0.1, -0.1) -> (-0.1, -0.1))
# inner_boundary2[100:120, 0] = -t[:]
# inner_boundary2[100:120, 1] = -0.1
# # Inner boundary: first and last vertices must be equal
# inner_boundary2[-1, :] = inner_boundary2[0, :]


# outer_boundary = outer_boundary
# inner_boundaries = [inner_boundary, (inner_boundary2+np.array([0.4, 0.2]))]
# obst_plane_x_length = 3
# obst_plane_y_length = 3

# hntf2d_control_object = hntf2d_control_law(p_init = [-0.8, 0.8], p_d = [0.9, -0.9], outer_boundary = outer_boundary, inner_boundaries = inner_boundaries, k_d = 1.0, k_i = [1.0, 1.0], w_phi = 1.0, gamma = 0.5, e_p = 0.1, e_v = 0.1, K = 1.0)
# hntf2d_control_object.create_new_harmonic_map_object()
# hntf2d_control_object.calculate_realws_to_disk_transformation(plot_map = True, keep_plot_on_screen = False)
# hntf2d_control_object.calculate_disk_to_R2_transformation(plot_map = True, keep_plot_on_screen = True)