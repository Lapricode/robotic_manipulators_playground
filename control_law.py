import numpy as np
import requests


# define the class hntf2d_control_law for calculating the control law using the harmonic potential field navigation function in 2D transformed space
class hntf2d_control_law:
    def __init__(self, p_init, p_d, outer_boundary, inner_boundaries, k_d, k_i, K, w_phi, gamma, e_p, e_v):
        self.p_init = np.array(p_init)  # the start position on the real workspace
        self.p_d = np.array(p_d)  # the target position on the real workspace
        self.outer_boundary = np.array(outer_boundary)  # convert the outer boundary to a numpy array
        self.inner_boundaries = [np.array(inner_boundaries[i]) for i in range(len(inner_boundaries))]  # convert the inner boundaries to numpy arrays
        self.unit_circle = np.array([(np.cos(theta), np.sin(theta)) for theta in np.linspace(0, 2 * np.pi, 100)])  # the unit circle on the transformed workspace (unit disk)
        self.q_init = self.p_init.copy()  # the start position on the transformed workspace
        self.q_d = self.p_d.copy()  # the target position on the transformed workspace
        self.q_i = [np.zeros(2) for i in range(len(self.inner_boundaries))]  # the obstacles punctures positions on the transformed workspace
        self.q_init_disk = self.p_init.copy()  # store the start position on the unit disk
        self.q_d_disk = self.p_d.copy()  # store the target position on the unit disk
        self.q_i_disk = self.q_i.copy()  # store the obstacles punctures positions on the unit disk
        self.k_d = k_d  # the target gain kd for the target position qd on the transformed workspace
        self.k_i = k_i  # the obstacles gains kis for the obstacles punctures positions qis on the transformed workspace
        self.K = K  # the scalar gain K for the control law
        self.w_phi = w_phi  # the scalar constant w_phi for the navigation function psi
        self.gamma = gamma  # the scalar constant gamma for the function s
        self.e_p = e_p  # the scalar constant e_p for the function sigmap
        self.e_v = e_v  # the scalar constant e_v for the function sigmav
        self.harmonic_map_object_is_built = False  # the flag to check if the HarmonicMap2D object has been created
        self.wtd_transformation_is_built = False  # the flag to check if the world to disk transformation has been done
        self.dtp_transformation_is_built = False  # the flag to check if the disk to R2 transformation has been done
    def create_new_harmonic_map_object(self):  # create a new HarmonicMap2D object
        try:  # try to create a new HarmonicMap2D object
            ssh_url = "http://127.0.0.1:5000/create_new_harmonic_map_object"  # the SSH URL to communicate with the virtual machine
            ssh_response = requests.post(ssh_url, json = {}).json()  # the SSH response from the virtual machine
            self.harmonic_map_object_is_built = True  # set the flag to True to indicate that the HarmonicMap2D object has been created
            print(ssh_response["message"])  # print the message from the SSH response
        except:  # if an error occurs
            print("Error: Could not create a new HarmonicMap2D object!")  # print an error message
    def realws_to_disk_transformation(self):  # calculate the harmonic transformation that maps the real workspace to the sphere world
        # the outer boundary goes to the unit circle, the inner boundaries go to discrete punctures
        try:  # try to calculate the transformation that maps the real workspace to the unit disk
            if self.harmonic_map_object_is_built and not self.wtd_transformation_is_built:  # check if the real workspace to disk transformation has not been done yet
                ssh_url = "http://127.0.0.1:5000/calculate_harmonic_transformation"  # the SSH URL to communicate with the virtual machine
                outer_boundary = self.outer_boundary.tolist()  # make sure the outer boundary is a list
                inner_boundaries = []  # initialize the inner boundaries
                for i in range(len(self.inner_boundaries)): inner_boundaries.append(self.inner_boundaries[i].tolist())  # make sure the inner boundaries are lists
                ssh_response = requests.post(ssh_url, json = {"outer_boundary": outer_boundary, "inner_boundaries": inner_boundaries}).json()  # the SSH response from the virtual machine
                self.unit_circle = np.array(ssh_response["unit_circle"])  # the unit_circle from the SSH response
                if np.linalg.norm(self.unit_circle[0] - self.unit_circle[-1]) != 0.0:  # check if the unit circle is not closed
                    self.unit_circle = np.vstack((self.unit_circle, self.unit_circle[0]))  # close the unit circle if it is not closed
                self.q_i_disk = np.array(ssh_response["punctures"])  # the obstacles punctures positions from the SSH response
                self.q_i = self.q_i_disk.copy()  # store the obstacles punctures positions on the transformed workspace
                self.wtd_transformation_is_built = True  # set the flag to True to indicate that the real workspace to disk transformation has been done
                print("Harmonic transformation from the real workspace to unit disk computed successfully!")  # print a message to the console
        except:  # if an error occurs
            print("Error: Could not compute the transformation from the real workspace to the unit disk!")  # print an error message
    def disk_to_R2_transformation(self):  # calculate the transformation that maps the sphere world to the R2 plane
        # the unit circle goes to infinity, the punctures positions have to be recalculated
        try:  # try to calculate the transformation that maps the unit disk to the R2 plane
            if self.harmonic_map_object_is_built and self.wtd_transformation_is_built and not self.dtp_transformation_is_built:  # check if the real workspace to disk transformation has been done and the disk to R2 transformation has not been done yet
                for k in range(len(self.q_i)):  # loop through all the punctures positions
                    self.q_i[k] = self.unit_disk_to_R2_mapping(self.q_i[k])  # map the punctures from the unit disk to the R2 plane
                self.dtp_transformation_is_built = True  # set the flag to True to indicate that the unit disk to R2 transformation has been done
                print("Transformation from the unit disk to the R2 plane computed successfully!")  # print a message to the console
        except:  # if an error occurs
            print("Error: Could not compute the transformation from the unit disk to the R2 plane!")  # print an error message
    def start_target_positions_transformations(self):  # calculate the mapping of the start and target positions from the real workspace to the R2 plane
        if self.wtd_transformation_is_built:  # check if the real workspace to disk transformation has been done
            self.q_init_disk = self.realws_to_unit_disk_mapping(self.p_init)  # map the start position from the real workspace to the sphere world
            self.q_d_disk = self.realws_to_unit_disk_mapping(self.p_d)  # map the target position from the real workspace to the sphere world
            self.q_init = self.q_init_disk.copy()  # store the start position on the transformed workspace
            self.q_d = self.q_d_disk.copy()  # store the target position on the transformed workspace
        if self.dtp_transformation_is_built:  # check if the disk to R2 transformation has been done
            self.q_init = self.unit_disk_to_R2_mapping(self.q_init_disk)  # map the start position from the sphere world to the R2 plane
            self.q_d = self.unit_disk_to_R2_mapping(self.q_d_disk)  # map the target position from the sphere world to the R2 plane
    def realws_to_unit_disk_mapping(self, p):  # map the point p from the real workspace to the sphere world (unit disk)
        # p = [px, py] is the point on the real workspace
        try:  # try to map the point p from the real workspace to the unit disk
            p = np.array(p).reshape(-1, 1)  # make sure p is a numpy array
            ssh_url = "http://127.0.0.1:5000/map_point_to_unit_disk"  # the SSH URL to communicate with the virtual machine
            ssh_response = requests.post(ssh_url, json = {"point": p.tolist()}).json()  # the SSH response from the virtual machine
            q = np.array(ssh_response["mapped_point"]).flatten()  # the mapped point q on the sphere world (unit disk)
            return q  # return the mapped point q
        except:  # if an error occurs
            print("Error: Could not map the point from the real workspace to the sphere world!")  # print an error message
            return None  # return None
    def realws_to_unit_disk_jacobian(self, p):  # calculate the jacobian of the transformation from the real workspace to the sphere world at the given point p
        # p = [px, py] is the point on the real workspace
        try:  # try to calculate the jacobian of the transformation at the point p
            p = np.array(p).reshape(-1, 1)  # make sure p is a numpy array
            ssh_url = "http://127.0.0.1:5000/compute_jacobian_at_point"  # the SSH URL to communicate with the virtual machine
            ssh_response = requests.post(ssh_url, json = {"point": p.tolist()}).json()  # the SSH response from the virtual machine
            jacobian = np.array(ssh_response["jacobian_matrix"])  # the jacobian matrix of the transformation at the point p
            return jacobian  # return the jacobian matrix at the given point p
        except:  # if an error occurs
            print("Error: Could not compute the jacobian of the transformation at the given point!")  # print an error message
            return None  # return None
    def unit_disk_to_R2_mapping(self, p):  # map the point p from the sphere world to the R2 plane
        # p = [px, py] is the point on the sphere world (unit disk)
        # 1e-7 is added to the denominator to avoid division by zero
        p = np.array(p).reshape(-1, 1)  # make sure p is a numpy array
        p_norm = np.linalg.norm(p)  # the norm of the point p
        q = (1.0 / (1.0 - p_norm + 1e-7) * p).flatten()  # the mapped point q on the R2 plane
        # q = (p_norm / (1.0 - p_norm + 1e-7) * p).flatten()  # the mapped point q on the R2 plane
        # q = (1.0 / (np.sqrt(1.0 - p_norm) + 1e-7) * p).flatten()  # the mapped point q on the R2 plane
        return q  # return the mapped point q
    def unit_disk_to_R2_jacobian(self, p):  # calculate the jacobian of the transformation from the sphere world to the R2 plane at the given point p
        # p = [px, py] is the point on the sphere world (unit disk)
        # 1e-7 is added to the denominator to avoid division by zero
        p = np.array(p).reshape(-1, 1)  # make sure p is a numpy array
        p_norm = np.linalg.norm(p)  # the norm of the point p
        jacobian = (p_norm * (1.0 - p_norm) * np.eye(2) + np.outer(p, p)) / (p_norm * (1.0 - p_norm)**2.0 + 1e-7)  # the jacobian matrix of the transformation at the point p
        # jacobian = (p_norm * (1.0 - p_norm) * np.eye(2) + np.outer(p, p)) / (p_norm * (1.0 - p_norm)**2.0 + 1e-7) - np.eye(2)  # the jacobian matrix of the transformation at the point p
        # jacobian = (p_norm * (1.0 - p_norm) * np.eye(2) + np.outer(p, p) / 2.0) / (p_norm * (1.0 - p_norm) * np.sqrt(1.0 - p_norm) + 1e-7)  # the jacobian matrix of the transformation at the point p
        return jacobian  # return the jacobian matrix at the given point p
    def harmonic_field_phi(self, q):  # calculate the harmonic potential field phi
        # kd * np.log(np.linalg.norm(q - qd)) is the target harmonic potential field phi
        # sum(ki * np.log(np.linalg.norm(q - qi))) is the obstacles harmonic potential field phi
        return self.k_d * np.log(np.linalg.norm(q - self.q_d)) - sum([self.k_i[i] * np.log(np.linalg.norm(q - self.q_i[i])) for i in range(len(self.q_i))])  # return the total harmonic potential field phi
    def harmonic_field_phi_gradient(self, q):  # calculate the gradient of the harmonic potential field phi
        # kd * (q - qd) / np.linalg.norm(q - qd)**2 is the gradient of the target harmonic potential field phi
        # sum(ki * (q - qi) / np.linalg.norm(q - qi)**2) is the gradient of the obstacles harmonic potential field phi
        # 1e-7 is added to the denominator to avoid division by zero
        return self.k_d * (q - self.q_d) / (np.linalg.norm(q - self.q_d)**2.0 + 1e-7) - sum([self.k_i[i] * (q - self.q_i[i]) / (np.linalg.norm(q - self.q_i[i])**2.0 + 1e-7) for i in range(len(self.q_i))])  # return the gradient of the harmonic potential field phi
    def navigation_psi(self, q):  # calculate the navigation function psi
        return (1.0 + np.tanh(self.harmonic_field_phi(q) / self.w_phi)) / 2.0  # return the navigation function psi
    def navigation_psi_gradient(self, q):  # calculate the gradient of the navigation function psi
        # return (1.0 / np.cosh(self.harmonic_field_phi(q) / self.w_phi)**2.0) / (2.0 * self.w_phi) * self.harmonic_field_phi_gradient(q)  # return the gradient of the navigation function psi
        return (1.0 - np.tanh(self.harmonic_field_phi(q) / self.w_phi)**2.0) / (2.0 * self.w_phi) * self.harmonic_field_phi_gradient(q)  # return the gradient of the navigation function psi
    def sigmap_func(self, x):  # calculate the value of the function sigmap
        # return 1 if x <= 1.0 else np.exp(1.0 - x)  # return the value of the function sigmap
        return 1 if x > 1.0 else x**2.0 * (3.0 - 2.0 * x)  # return the value of the function sigmap
    def sigmav_func(self, x):  # calculate the value of the function sigmav
        return 0.0 if x <= 0.0 else x**2.0  # return the value of the function sigmav
    def scalar_gain_s(self, q):  # calculate the scalar gain s
        psi_grad = self.navigation_psi_gradient(q)  # calculate the gradient of the navigation function psi at the point q
        return self.K * (self.gamma * self.sigmap_func((np.linalg.norm(q - self.q_d)) / self.e_p) + (1 - self.gamma) * self.sigmav_func(np.dot(psi_grad.T, q) / (self.e_v + np.linalg.norm(psi_grad) * np.linalg.norm(q))))  # return the scalar gain s
    def run_control_law(self, dt = 0.01, error_tolerance = 0.001, max_iterations = 1000):  # calculate the control law and the path on the real workspace
        # dt is the time step for the control law and error_tolerance is the error tolerance for the target position
        # the start position is p_init and the target position is p_d
        p = np.array(self.p_init)  # initialize the control law position on the real workspace
        p_dot = np.zeros(2)  # initialize the control law velocity on the real workspace
        q = np.array(self.q_init)  # initialize the control law position on the transformed workspace
        q_dot = np.zeros(2)  # initialize the control law velocity on the transformed workspace
        steps_counter = 0  # initialize the steps counter
        p_list = [p]  # initialize the list of points on the real workspace
        p_dot_list = []  # initialize the list of control laws on the real workspace
        # q_list = [q]  # initialize the list of points on the transformed workspace
        # q_dot_list = []  # initialize the list of control laws on the transformed workspace
        while steps_counter < max_iterations:  # loop until the maximum number of steps is reached, unless the target position is reached (within the error tolerance, then break the loop)
            steps_counter += 1
            s = self.scalar_gain_s(q)  # calculate the scalar gain s
            psi_grad = self.navigation_psi_gradient(q)  # calculate the gradient of the navigation function psi at the point q
            q_dot = -s * psi_grad  # calculate the control law on the transformed workspace
            # q_dot_list.append(q_dot)  # append the control law on the transformed workspace to the list
            p_dot_new = self.convert_q_dot_to_p_dot(p, q_dot)  # convert the control law from the transformed workspace to the real workspace
            if np.linalg.norm(p_dot_new) != 0.0: p_dot = p_dot_new  # update the control law on the real workspace if it is not zero
            # p_dot_list.append(p_dot)  # append the control law on the real workspace to the list
            p = self.euler(p, p_dot, dt)  # update the point p on the real workspace
            # p = self.runge_kutta_4th_order(p, q_dot, self.convert_q_dot_to_p_dot, dt)  # update the point p on the real workspace
            p_list.append(p)  # append the point p to the list
            q_disk = self.realws_to_unit_disk_mapping(p)  # calculate the mapping of the point p from the real workspace to the unit disk
            q = self.unit_disk_to_R2_mapping(q_disk)  # calculate the mapping of the point q from the unit disk to the R2 plane
            # q = self.euler(q, q_dot, dt)  # update the point q on the transformed workspace
            # q_list.append(q)  # append the point q to the list
            if np.linalg.norm(p - self.p_d) <= error_tolerance:  # check if the target position is reached within the error tolerance
                break  # break the loop
            # if np.linalg.norm(q - self.q_d) <= error_tolerance:  # check if the target position is reached within the error tolerance
            #     break  # break the loop
        # if np.linalg.norm(p_list[-1] - self.p_d) > error_tolerance:  # if the target position is not reached within the error tolerance after the maximum number of steps
        #     return p_list, p_dot_list, False  # return the failed control law on the real workspace and a flag indicating that the target position is not reached
        # return p_list, p_dot_list, True  # return the successfull control law on the real workspace and a flag indicating that the target position is reached
        if np.linalg.norm(p_list[-1] - self.p_d) > error_tolerance:  # if the target position is not reached within the error tolerance after the maximum number of steps
            return p_list, p_dot_list, False  # return the failed control law on the real workspace and a flag indicating that the target position is not reached
        return p_list, p_dot_list, True  # return the successfull control law on the real workspace and a flag indicating that the target position is reached
    def convert_q_dot_to_p_dot(self, p, q_dot):  # convert the control law from the transformed workspace to the real workspace
        q_disk = self.realws_to_unit_disk_mapping(p)  # calculate the mapping of the point p from the real workspace to the unit disk
        J_wtd = self.realws_to_unit_disk_jacobian(p)  # calculate the jacobian of the transformation from the real workspace to the unit disk at the point q
        J_dtp = self.unit_disk_to_R2_jacobian(q_disk)  # calculate the jacobian of the transformation from the unit disk to the R2 plane at the point q
        try:  # if the inverse of the jacobian matrices can be calculated
            p_dot = np.linalg.inv(J_wtd) @ np.linalg.inv(J_dtp) @ q_dot  # calculate the control law on the real workspace
            return p_dot  # return the control law on the real workspace
        except:  # if an error occurs, due to a singular Jacobian matrix for example
            return np.zeros(2)  # return a zero control law
    def euler(self, pk, uk, dt):  # implement the euler numerical integration
        p = pk + uk * dt  # calculate the next position p
        return p  # return the next position p
    def runge_kutta_4th_order(self, pk, uk, dynamics, dt):  # implement the runge-kutta 4th order numerical integration
        f1 = dynamics(pk, uk)
        f2 = dynamics(pk + f1 * dt / 2.0, uk)
        f3 = dynamics(pk + f2 * dt / 2.0, uk)
        f4 = dynamics(pk + f3 * dt, uk)
        p = pk + (f1 + 2*f2 + 2*f3 + f4) / 6.0 * dt  # calculate the next position p
        return p  # return the next position p