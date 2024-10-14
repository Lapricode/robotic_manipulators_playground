import tkinter as tk
import tkinter.ttk as ttk
import tkinter.simpledialog as sd
import tkinter.messagebox as ms
import tkinter.colorchooser as cc
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image, ImageColor
import string, copy
import os, shutil
import serial, time
import roboticstoolbox as rtb
import numpy as np
import scipy as sc
import spatialmath as sm
import spatialgeometry as sg
import swift
import cv2
import threading
import serial_port_finder as spf
import gui_buttons_labels as gbl
import robots_kinematics as kin
import camera_detection as cd
import polygons_boundaries_operations as pbo
import general_functions as gf
import control_law_ssh_connection as cl
# import control_law_hntf2d as cl


# the class that creates the GUI window of the application
class robotic_manipulators_playground_window():
    def __init__(self, root, instance):
        self.root = root  # the root window of the GUI
        self.root.title("Robotic manipulators playground, made by Printzios Lampros (https://github.com/Lapricode)")  # the title of the windows
        self.root.geometry("+0+0")  # the position of the window
        self.gui_instance = instance  # the instance of the class that contains the functions and the variables of the robotic manipulator playground
        # do some initial actions for saving and loading the proper folders and files in the system
        camera_intrinsic_parameters_file_name = "camera_intrinsic_parameters.txt"  # the file name to store the camera intrinsic parameters
        aruco_dictitionaries_file_name = "aruco_dictionaries.txt"  # the file name to store the aruco dictionaries
        robots_run_by_swift_file_name = "robots_run_by_swift.txt"  # the file name to store the robots that can be simulated by the Swift simulator
        robots_models_descriptions_folder_name = "robots_models_descriptions"  # the folder name to store the descriptions (meshes and urdfs/xacros) of the robots models
        saved_robotic_manipulators_folder_name = "saved_robotic_manipulators"  # the folder name to store the saved robotic manipulators
        saved_obstacles_transformations_folder_name = "saved_obstacles_transformations"  # the folder name to store the saved obstacles transformations
        saved_workspace_images_infos_folder_name = "saved_workspace_images_infos"  # the folder name to store the saved workspace images
        saved_obstacles_objects_infos_folder_name = "saved_workspace_obstacles_infos"  # the folder name to store the saved workspace obstacles
        saved_camera_calibration_images_folder_name = "camera_calibration_images"  # the folder name to store the saved camera calibration images
        saved_control_law_parameters_folder_name = "saved_control_law_parameters"  # the folder name to store the saved control law parameters
        menus_descriptions_folder_name = "menus_descriptions"  # the folder name to store the descriptions of the menus
        self.camera_intrinsic_parameters_file_path = str(os.getcwd() + fr"/{camera_intrinsic_parameters_file_name}")  # the path of the file to store the camera intrinsic parameters
        self.aruco_dictionaries_file_path = str(os.getcwd() + fr"/{aruco_dictitionaries_file_name}")  # the path of the file to store the aruco dictionaries
        self.robots_run_by_swift_file_path = str(os.getcwd() + fr"/{robots_run_by_swift_file_name}")  # the path of the file to store the robots that can be simulated by the Swift simulator
        self.saved_robots_descriptions_folder_path = str(os.getcwd() + fr"/{robots_models_descriptions_folder_name}")  # the path of the folder where the descriptions (meshes and urdfs/xacros) of the robots models are stored
        self.saved_robotic_manipulators_folder_path = str(os.getcwd() + fr"/{saved_robotic_manipulators_folder_name}")  # the path of the folder where the saved robotic manipulators are stored
        self.rtbdata_robots_meshes_path = str(rtb.rtb_path_to_datafile("xacro"))  # the rtbdata path to store the meshes and urdfs/xacros of the robotic manipulators models in the library
        self.saved_obstacles_transformations_folder_path = str(os.getcwd() + fr"/{saved_obstacles_transformations_folder_name}")  # the path of the folder where the saved obstacles transformations are stored
        self.saved_workspace_images_infos_folder_path = str(os.getcwd() + fr"/{saved_workspace_images_infos_folder_name}")  # the path of the folder where the saved workspace images are stored
        self.saved_obstacles_objects_infos_folder_path = str(os.getcwd() + fr"/{saved_obstacles_objects_infos_folder_name}")  # the path of the folder where the saved workspace obstacles are stored
        self.saved_camera_calibration_images_folder_path = str(os.getcwd() + fr"/{saved_camera_calibration_images_folder_name}")  # the path of the folder where the saved camera calibration images are stored
        self.saved_control_law_parameters_folder_path = str(os.getcwd() + fr"/{saved_control_law_parameters_folder_name}")  # the path of the folder where the saved control law parameters are stored
        self.menus_descriptions_folder_path = str(os.getcwd() + fr"/{menus_descriptions_folder_name}")  # the path of the folder where the descriptions of the menus are stored
        # if robots_models_descriptions_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the descriptions (meshes and urdfs/xacros) of the robots models does not exist
        #     os.mkdir(self.saved_robots_descriptions_folder_path)  # create the folder to store the descriptions (meshes and urdfs/xacros) of the robots models
        #     for robot_name in self.swift_simulated_robots_list:  # for each robotic manipulator that can be simulated by the Swift simulator
        #         os.mkdir(self.saved_robots_descriptions_folder_path + fr"/{robot_name}_description")  # create a folder for the robotic manipulator model
        #         os.mkdir(self.saved_robots_descriptions_folder_path + fr"/{robot_name}_description/meshes")  # create a folder to store the meshes of the robotic manipulator model
        #         os.mkdir(self.saved_robots_descriptions_folder_path + fr"/{robot_name}_description/urdf")  # create a folder to store the urdf of the robotic manipulator model
        # if saved_robotic_manipulators_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the saved robotic manipulator models does not exist
        #     os.mkdir(self.saved_robotic_manipulators_folder_path)  # create the folder to store the saved robotic manipulator models
        # if saved_obstacles_transformations_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the saved obstacles transformations does not exist
        #     os.mkdir(self.saved_obstacles_transformations_folder_path)  # create the folder to store the saved obstacles transformations
        # if saved_workspace_images_infos_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the saved workspace images does not exist
        #     os.mkdir(self.saved_workspace_images_infos_folder_path)  # create the folder to store the saved workspace images
        # if saved_obstacles_objects_infos_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the saved workspace obstacles does not exist
        #     os.mkdir(self.saved_obstacles_objects_infos_folder_path)  # create the folder to store the saved workspace obstacles
        # if saved_camera_calibration_images_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the saved camera calibration images does not exist
        #     os.mkdir(self.saved_camera_calibration_images_folder_path)  # create the folder to store the saved camera calibration images
        # if saved_control_law_parameters_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the saved control law parameters does not exist
        #     os.mkdir(self.saved_control_law_parameters_folder_path)  # create the folder to store the saved control law parameters
        # if menus_descriptions_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the descriptions of the menus does not exist
        #     os.mkdir(self.menus_descriptions_folder_path)  # create the folder to store the descriptions of the menus
        robots_models_descriptions_folders_list = [folder for folder in os.listdir(self.saved_robots_descriptions_folder_path) if os.path.isdir(self.saved_robots_descriptions_folder_path + fr"/{folder}")]  # the folders of the robots models descriptions
        for robot_model in robots_models_descriptions_folders_list:  # for each robotic manipulator model
            if robot_model not in os.listdir(self.rtbdata_robots_meshes_path):  # if the robotic manipulator model is not in the rtbdata folder
                shutil.copytree(self.saved_robots_descriptions_folder_path + fr"/{robot_model}", self.rtbdata_robots_meshes_path + fr"/{robot_model}")  # copy the robot model to the rtbdata folder
            else:  # if the robotic manipulator model is in the rtbdata folder
                shutil.rmtree(self.rtbdata_robots_meshes_path + fr"/{robot_model}")  # remove the robotic manipulator model from the rtbdata folder
                shutil.copytree(self.saved_robots_descriptions_folder_path + fr"/{robot_model}", self.rtbdata_robots_meshes_path + fr"/{robot_model}")  # copy the robot model to the rtbdata folder
        # define the menus of the GUI
        self.main_menu_choice = 0  # the number choice of the main menu
        self.submenus_titles = [["Define robot's parameters", "Adjust the visualization"], ["Forward kinematics", "Inverse kinematics"], ["Differential kinematics", "Inverse differential kinematics"], \
                            ["Establish communication with arduino microcontroller", "Serial monitor / Console", "Control joints and end-effector motors"], ["Camera control"], ["Workspace obstacles"], ["Obstacles avoidance solver"]]  # the titles of the submenus
        self.submenus_descriptions = []  # the descriptions of the submenus
        for menu_num in range(len(self.submenus_titles)):  # for each menu
            self.submenus_descriptions.append([])  # append an empty list to store the descriptions of the submenus for the current menu
            for submenu_num in range(len(self.submenus_titles[menu_num])):  # for each submenu
                try: self.submenus_descriptions[-1].append("".join(open(self.menus_descriptions_folder_path + fr"/menu_{menu_num + 1}/submenu_{submenu_num + 1}.txt", "r", encoding = "utf-8").readlines()))  # read the description of the current submenu
                except: pass
        construct_robotic_manipulator_menu_build_details = dict(title = "Construct the robotic manipulator", submenus_titles = self.submenus_titles[0], submenus_descriptions = self.submenus_descriptions[0], \
                                                                build_function = self.build_construct_robotic_manipulator_menus)  # a dictionary to store the details of the construct robotic manipulator menu
        forward_kinematics_menu_build_details = dict(title = "Forward kinematics analysis", submenus_titles = self.submenus_titles[1], submenus_descriptions = self.submenus_descriptions[1], \
                                                        build_function = self.build_robotic_manipulator_forward_kinematics_menus)  # a dictionary to store the details of the forward kinematics menu
        differential_kinematics_menu_build_details = dict(title = "Differential kinematics analysis", submenus_titles = self.submenus_titles[2], submenus_descriptions = self.submenus_descriptions[2], \
                                                            build_function = self.build_robotic_manipulator_differential_kinematics_menus)  # a dictionary to store the details of the differential kinematics menu
        control_robotic_manipulator_menu_build_details = dict(title = "Control the robotic manipulator", submenus_titles = self.submenus_titles[3], submenus_descriptions = self.submenus_descriptions[3], \
                                                                build_function = self.build_control_robotic_manipulator_menus)  # a dictionary to store the details of the control robotic manipulator menu
        camera_control_menu_build_details = dict(title = "Control the camera", submenus_titles = self.submenus_titles[4], submenus_descriptions = self.submenus_descriptions[4], \
                                                    build_function = self.build_camera_control_menus)  # a dictionary to store the details of the camera control menu
        workspace_obstacles_menu_build_details = dict(title = "Create workspace obstacles", submenus_titles = self.submenus_titles[5], submenus_descriptions = self.submenus_descriptions[5], \
                                                        build_function = self.build_workspace_obstacles_menus)  # a dictionary to store the details of the workspace obstacles menu
        solve_obstacles_avoidance_menu_build_details = dict(title = "Solve obstacles avoidance", submenus_titles = self.submenus_titles[6], submenus_descriptions = self.submenus_descriptions[6], \
                                                            build_function = self.build_solve_obstacles_avoidance_menus)  # a dictionary to store the details of the solve obstacles avoidance menu
        self.main_menus_build_details = [construct_robotic_manipulator_menu_build_details, forward_kinematics_menu_build_details, differential_kinematics_menu_build_details, control_robotic_manipulator_menu_build_details, \
                                        camera_control_menu_build_details, workspace_obstacles_menu_build_details, solve_obstacles_avoidance_menu_build_details]  # a list to store the basic details of all the main menus in order to build them
        # define the workspace parameters
        self.workspace_canvas_points = []  # the list to store the points of the workspace
        self.pointing_to_point = "(0.000, 0.000, 0.000)"  # the point to which the user's cursor is pointing to in the workspace
        self.x_axis_range = 0.5  # the shown length of the x axis of the workspace
        self.y_axis_range = 0.5  # the shown length of the y axis of the workspace
        self.z_axis_range = 0.5  # the shown length of the z axis of the workspace
        self.axis_range_values = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.5, 2.0]  # the possible values of the axis ranges of the workspace
        self.magnify_workspace_constant = 400  # the constant used to magnify the workspace
        self.workspace_width_ratio = 0.5  # the ratio of the width of the workspace area to the width of the root window
        self.workspace_width_ratios_values = [0.3, 0.5, 0.7, 1.0]  # the possible values of the workspace width ratio
        self.workspace_sensitivity_values = [0.1, 0.5, 1.0]; self.workspace_sensitivity_degrees = ["high", "normal", "low"]  # the possible values/degrees of the sensitivity of the workspace control
        self.workspace_control_sensitivity = self.workspace_sensitivity_values[self.workspace_sensitivity_degrees.index("normal")]  # control the workspace motion sensitivity
        self.axis_enable_visualization = "shown"  # control the axis visualization
        self.terrain_enable_visualization = "shown"  # control the terrain visualization
        self.robotic_manipulator_enable_visualization = "shown"  # control the robotic manipulator visulization
        self.robot_joints_enable_visualization = "shown"  # control the robotic manipulator joints visualization
        self.robot_links_enable_visualization = "shown"  # control the robotic manipulator links visualization
        self.end_effector_frame_enable_visualization = "shown"  # control the end-effector frame visualization
        self.obstacles_plane_enable_visualization = "shown"  # control the obstacles plane visualization
        self.plane_frame_enable_visualization = "frame"  # control the obstacles plane frame visualization
        self.obstacles_objects_enable_visualization = "hidden"  # control the obstacles objects visualization
        self.camera_enable_visualization = "shown"  # control the camera visualization
        self.camera_frame_enable_visualization = "frame"  # control the camera frame visualization
        self.drawing_order_list = ["robot", "obstacles"]  # the possible values of the drawing order
        self.drawing_order = self.drawing_order_list[0]  # control the drawing order of the robotic manipulator and the obstacles
        self.x_axis_view = "+"  # the view of the x axis of the workspace, it can be "+" (front) or "-" (back) or "0" (none)
        self.y_axis_view = "+"  # the view of the y axis of the workspace, it can be "+" (right) or "-" (left) or "0" (none)
        self.z_axis_view = "+"  # the view of the z axis of the workspace, it can be "+" (up) or "-" (down) or "0" (none)
        self.control_or_kinematics_variables_visualization_list = ["control", "kinematics"]  # the possible values of the control or fkine variables visualization
        self.control_or_kinematics_variables_visualization = self.control_or_kinematics_variables_visualization_list[0]  # determine which variables (control or forward kinematics) to visualize
        # define the robotic manipulator model technical parameters
        self.robotic_manipulator_model_name = ""  # the name of the current robotic manipulator model
        self.joints_number = 6  # set the total number of joints of the robotic manipulator
        self.frames_number, self.links_number = self.set_frames_links_number(joints_number = self.joints_number)  # set the total number of frames and links of the robotic manipulator
        self.max_joints_number = 12  # the maximum possible number of joints of the robotic manipulator
        self.chosen_joint_number_model = 1  # the number of the chosen joint of the robotic manipulator for model changes
        self.joints_types_list = ["revolute", "prismatic"]  # the possible types of the robotic manipulator joints
        self.joints_types = [self.joints_types_list[0] for _ in range(self.joints_number)]  # the types of the robotic manipulator joints
        self.den_har_parameters_extreme_limits = [[-360.0, 360.0], [-5.0, 5.0]]  # the extreme limits of the parameters in degrees or meters, depending on the joint's type
        self.a_den_har_parameters = [0.0 for _ in range(self.joints_number)]  # the a parameters of the Denavit - Hartnemberg convention in meters
        self.a_parameters_limits = copy.deepcopy(self.den_har_parameters_extreme_limits[1])  # the limits of the a parameters in meters
        self.d_den_har_parameters = [0.0 for _ in range(self.joints_number)]  # the d parameters of the Denavit - Hartnemberg convention in meters
        self.d_parameters_limits = copy.deepcopy(self.den_har_parameters_extreme_limits[1])  # the limits of the d parameters in meters
        self.alpha_den_har_parameters = [0.0 for _ in range(self.joints_number)]  # the alpha parameters of the Denavit - Hartnemberg convention in radians
        self.alpha_parameters_limits = copy.deepcopy(np.deg2rad(self.den_har_parameters_extreme_limits[0]))  # the limits of the alpha parameters in radians
        self.theta_den_har_parameters = [0.0 for _ in range(self.joints_number)]  # the theta parameters of the Denavit - Hartnemberg convention in radians
        self.theta_parameters_limits = copy.deepcopy(np.deg2rad(self.den_har_parameters_extreme_limits[0]))  # the limits of the theta parameters in radians
        self.base_position_wrt_world = np.array([0.0, 0.0, 0.0])  # the position of the base system (in meters) of the robotic manipulator in the workspace, wrt the world frame
        self.base_position_limits = [[-10.0, 10.0], [-10.0, 10.0], [-10.0, 10.0]]  # the limits of the base position in meters
        self.zero_frame_position_wrt_base = np.array([0.0, 0.0, 0.0])  # the position of the zero frame (in meters) of the robotic manipulator in the workspace, wrt the base system
        self.zero_frame_position_limits = [[-10.0, 10.0], [-10.0, 10.0], [-10.0, 10.0]]  # the limits of the zero frame position in meters
        self.base_orientation_wrt_world = np.array([0.0, 0.0, 0.0])  # the orientation of the base system (xyz extrinsic Euler angles in radians) of the robotic manipulator in the workspace, wrt the world frame
        self.base_orientation_limits = [[-180.0, 180.0], [-180.0, 180.0], [-180.0, 180.0]]  # the limits of the base orientation angles in degrees
        self.zero_frame_orientation_wrt_base = np.array([0.0, 0.0, 0.0])  # the orientation of the zero frame system (xyz extrinsic Euler angles in radians) of the robotic manipulator in the workspace, wrt the base system
        self.zero_frame_orientation_limits = [[-180.0, 180.0], [-180.0, 180.0], [-180.0, 180.0]]  # the limits of the zero frame orientation angles in degrees
        self.end_effector_position_wrt_last_frame = np.array([0.0, 0.0, 0.0])  # the position of the end-effector system (in meters) of the robotic manipulator in the workspace, wrt the last joint frame
        self.end_effector_position_limits = [[-10.0, 10.0], [-10.0, 10.0], [-10.0, 10.0]]  # the limits of the end-effector position in meters
        self.end_effector_orientation_wrt_last_frame = np.array([0.0, 0.0, 0.0])  # the orientation of the end-effector system (xyz extrinsic Euler angles in radians) of the robotic manipulator in the workspace, wrt the last joint frame
        self.end_effector_orientation_limits = [[-180.0, 180.0], [-180.0, 180.0], [-180.0, 180.0]]  # the limits of the end-effector orientation angles in degrees
        self.robotic_manipulator_is_built = False  # the flag to check if the robotic manipulator is built
        self.built_robotic_manipulator = None  # the current built robotic manipulator model
        self.built_robotic_manipulator_info = {}  # the information of the built robotic manipulator model
        self.saved_robots_models_files_list = [file[:-4] for file in os.listdir(self.saved_robotic_manipulators_folder_path)]  # the list of the saved robotic manipulator models
        # define the robotic manipulator visualization parameters
        self.min_visualization_size = 1  # the minimum size of the visualization of the workspace
        self.max_visualization_size = 10  # the maximum size of the visualization of the workspace
        self.workspace_canvas_color = "#73938b"  # the color of the workspace canvas
        self.up_side_terrain_color = "#6e6e6e"  # the color of the up side of the workspace canvas
        self.down_side_terrain_color = "#333333"  # the color of the down side of the workspace canvas
        self.workspace_terrain_color = self.up_side_terrain_color  # the color of the terrain of the workspace
        self.workspace_axis_colors = ["red", "green", "blue"]  # the colors of the x, y, z axis of the workspace
        self.workspace_axis_sizes = 5  # the sizes of the x, y, z, axis of the workspace
        self.front_side_2d_plane_color = "white"  # the color of the front side 2D plane of the workspace, where the workspace obstacles are located
        self.back_side_2d_plane_color = "#333333"  # the color of the back side 2D plane of the workspace, where the workspace obstacles are located
        self.workspace_2d_plane_color = self.front_side_2d_plane_color  # the color of the 2D plane of the workspace, where the workspace obstacles are located
        self.workspace_2d_plane_size = 5  # the size of the 2D plane of the workspace, where the workspace obstacles are located
        self.workspace_obstacles_color = "#111111"  # the color of the workspace obstacles
        self.workspace_obstacles_size = 2  # the size of the workspace obstacles
        self.chosen_frame_visualization = "frame 0"  # the chosen frame of the robotic manipulator for visualization
        self.chosen_link_visualization = "link 1"  # the chosen link of the robotic manipulator for visualization
        self.joints_z_axis_positions = [0.0 for _ in range(self.joints_number)]  # the z axis (of the joint's frame) positions of the robotic manipulator joints
        self.frames_origins_colors = ["black" for _ in range(self.frames_number)]  # the colors of the robotic manipulator frames origins
        self.frames_origins_sizes = [7 for _ in range(self.frames_number)]  # the widths of the robotic manipulator frames origins
        self.initial_links_lengths = [0.0 for _ in range(self.links_number)]  # the initial lengths of the robotic manipulator links
        self.links_colors = ["red" for _ in range(self.links_number)]  # the colors of the robotic manipulator links
        self.links_sizes = [5 for _ in range(self.links_number)]  # the widths of the robotic manipulator links
        # define the parameters for the robotic manipulator control
        self.angles_precision = 1  # the decimal precision of the revolute variables
        self.distances_precision = 3  # the decimal precision of the prismatic variables
        self.chosen_joint_number_control = 1  # the number of the chosen joint of the robotic manipulator for the control
        self.control_joints_variables = [0.0 for _ in range(self.joints_number)]  # the variables for the robotic manipulator control in radians or meters, depending on the joint's type
        self.control_joints_variables_extreme_limits = copy.deepcopy(self.den_har_parameters_extreme_limits)  # the extreme limits of the control joints variables in degrees or meters, depending on the joint's type
        self.control_joints_variables_limits = [copy.deepcopy(self.control_joints_variables_extreme_limits) for _ in range(self.joints_number)]  # the limits of the control joints variables in degrees or meters, depending on the joint's type
        self.chosen_joint_motor_number = 0  # the order number of the chosen control joint motor of the robotic manipulator
        self.joints_motors_list = list([string.ascii_uppercase[joint]] for joint in range(self.joints_number))  # the list of the motors of the robotic manipulator
        self.joints_motors_mult_factors = [[1.0] for _ in range(self.joints_number)]  # a list to store the multiplication factors for the joints variables to be sent as commands to the joints motors
        self.control_end_effector_variable = 0.0  # the control variable for the control end-effector
        self.control_end_effector_limits = [0, 100]  # the control limits of the control end-effector
        self.end_effector_motor = "S"  # the motor of the end-effector
        self.end_effector_motor_mult_factor = 1.0  # the multiplication factor for the end-effector motor
        self.robotic_manipulator_control_mode = "manual"  # the mode of the robotic manipulator control, it can be "manual" or "automatic"
        self.robotic_manipulator_control_modes_list = ["manual", "automatic"]  # the possible modes of the robotic manipulator control
        # define the serial communication parameters
        self.serial_connection = serial.Serial()  # define the serial communication object for the serial connection between the arduino microcontroller and the computer
        self.serial_connection_thread_flag = False  # the flag to run/stop the serial communication thread
        self.serial_connection_elapsed_time = 0.0  # the elapsed time of the serial communication
        self.closed_serial_connection_message = "The serial connection is closed!"  # the message to be emitted when the serial connection is closed
        self.serial_connect_disconnect_command = "Connect"  # the command to connect or disconnect the serial communication between the arduino microcontroller and the computer
        self.available_serial_ports = []  # a list to store the available serial ports of the computer
        self.serial_port = ""  # the serial port used for the serial communication with the arduino microcontroller
        self.baudrates_list = ["115200", "57600", "38400", "28800", "19200", "14400", "9600", "4800", "2400", "1200", "600", "300"]  # the possible baudrates of the serial communication
        self.baudrate = self.baudrates_list[0]  # the baudrate used for the serial communication with the arduino microcontroller
        self.serial_connection_state = "Disconnected"  # the state of the serial connection with the arduino microcontroller
        self.serial_connection_states_list = ["Disconnected", "Connected", "Connected - Idle", "Connected - Run", "Connected - Home", "Connected - Almanipulator", "Connected - Hold", "Connected to Port"]  # the possible states of the serial connection with the arduino microcontroller
        self.serial_connection_indicator_colors = ["#ff0000", "#00ff00", "#00ff4b", "#00ff96", "#00ffff", "#ffff00", "#ff00ff", "#ffffff"]  # the colors of the serial connection indicator
        self.serial_monitor_text = ""  # the string variable to store the text of the serial monitor
        self.command_starting_text = ""  # the starting text of the commands sent to the arduino microcontroller
        self.command_ending_text = ""  # the ending text of the commands sent to the arduino microcontroller
        self.show_status_responses = False  # the flag to show the status responses of the robotic manipulator
        self.show_ok_responses = False  # the flag to show the ok responses for the commands sent to the robotic manipulator
        self.show_responses_indicators = ["✖", "✔"]  # the possible values of the show responses indicators
        self.expanded_serial_monitor = False  # choose if you want to expand the serial monitor menu
        self.allow_sending_all_ports = False  # choose if you want to allow sending commands to all serial ports
        # define the parameters for the kinematics (forward kinematics, inverse kinematics, differential kinematics, inverse differential kinematics) of the robotic manipulator
        self.chosen_joint_number_fkine = 1  # the number of the chosen joint of the robotic manipulator for the forward kinematics
        self.forward_kinematics_variables = [0.0 for _ in range(self.joints_number)]  # the variables of the forward kinematics (if the joint is revolute, the variable is the theta angle parameter in radians, else if the joint is prismatic, the variable is the d distance parameter in meters)
        self.chosen_frame_fkine = "end-effector"  # the chosen robotic manipulator frame for the forward kinematics
        self.fkine_frame_position = np.array([0.0, 0.0, 0.0])  # the position of the chosen robotic manipulator frame for the forward kinematics in meters
        self.fkine_frame_orientation = np.array([0.0, 0.0, 0.0])  # the orientation of the chosen robotic manipulator frame for the forward kinematics in radians (initialized in Euler angles form in radians)
        self.orientation_representation_fkine = "Euler xyz (°)"  # the representation of the orientation of the chosen robotic manipulator frame for the forward kinematics
        self.orientation_representations_fkine_list = ["Euler xyz (°)", "Quaternion", "Rot. matrix"]  # the possible representations of the orientation of the chosen robotic manipulator frame for the forward kinematics
        self.joints_range_divisions_list = [2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20]  # the possible values of the joints range divisions for the calculation of the robot's reachable workspace
        self.joints_range_divisions = self.joints_range_divisions_list[1]  # the number of joints range divisions for the calculation of the robot's reachable workspace
        self.chosen_invkine_3d_position = np.array([0.0, 0.0, 0.0])  # the 3D position of the end-effector for the inverse kinematics in meters
        self.chosen_invkine_orientation = np.array([0.0, 0.0, 0.0])  # the orientation of the end-effector for the inverse kinematics in radians (expressed in xyz extrinsic Euler angles in radians)
        self.invkine_joints_configuration = [0.0 for _ in range(self.joints_number)]  # the joints configuration for the inverse kinematics in radians or meters, depending on the joint's type
        self.invkine_tolerance = 1e-10  # the tolerance for the inverse kinematics
        self.chosen_joint_number_diffkine = 1  # the number of the chosen joint of the robotic manipulator for the differential kinematics
        self.differential_kinematics_velocities = [0.0 for _ in range(self.joints_number)]  # the velocities of the joints for the differential kinematics
        self.diffkine_velocities_limits = [[[-360, 360], [-1, 1]] for _ in range(self.joints_number)]  # the limits of the velocities of the joints for the differential kinematics
        self.diffkine_linear_vel = np.array([0.0, 0.0, 0.0])  # the velocity of the end-effector in the workspace for the differential kinematics in meters/sec
        self.diffkine_angular_vel = np.array([0.0, 0.0, 0.0])  # the angular velocity of the end-effector for the differential kinematics in radians/sec
        self.diffkine_wrt_frame_list = ["world", "end-effector"]  # the possible values of the differential kinematics wrt frame
        self.diffkine_wrt_frame = self.diffkine_wrt_frame_list[0]  # the frame wrt which the differential kinematics is calculated
        self.chosen_invdiffkine_linear_vel = np.array([0.0, 0.0, 0.0])  # the velocity of the end-effector for the inverse differential kinematics in meters/sec
        self.chosen_invdiffkine_angular_vel = np.array([0.0, 0.0, 0.0])  # the angular velocity of the end-effector for the inverse differential kinematics in radians/sec
        self.invdiffkine_joints_velocities = [0.0 for _ in range(self.joints_number)]  # the joints velocities for the inverse differential kinematics in radians/sec or meters/sec, depending on the joint's type
        self.invdiffkine_wrt_frame_list = ["world", "end-effector"]  # the possible values of the inverse differential kinematics wrt frame
        self.invdiffkine_wrt_frame = self.invdiffkine_wrt_frame_list[0]  # the frame wrt which the inverse differential kinematics is calculated
        # define the camera parameters
        self.camera_capture = None  # the camera capture object
        self.camera_choices_list = ["external", "internal"]  # the possible choices for the camera
        self.camera_choice = self.camera_choices_list[0]  # the choice of the camera, "internal" for the default PC's camera, "external" for the external camera
        self.camera_thread_flag = False  # the flag to run/stop the camera thread
        self.camera_frames_are_shown = False  # the flag to check if the camera frame is shown
        self.original_image_is_shown = False  # the flag to check if the original image is shown
        self.grayscale_image_is_shown = False  # the flag to check if the grayscale image is shown
        self.workspace_image_is_shown = False  # the flag to check if the workspace image is shown
        self.camera_pose_estimation_happening = False  # the flag to check if the camera pose estimation is happening
        self.obst_plane_pose_estimation_happening = False  # the flag to check if the obstacles plane pose estimation is happening
        self.draw_2d_plane_on_image_happening = False  # the flag to check if the drawing of the 2D plane on the image is happening
        self.obstacles_boundaries_are_shown = False  # the flag to check if the obstacles boundaries are shown
        self.camera_original_window_name = "Camera's original frames (press \"q\" to close the camera)"  # the name of the camera window
        self.grayscale_image_window_name = "Converted frames to black and white (press \"a\" to decrease the luminance threshold or \"w\" to increase it)"  # the name of the grayscale image window
        self.estimate_ArUco_pose_window_name = "Image with the detected ArUco markers and their poses"  # the name of the window to estimate the ArUco markers poses
        self.draw_shape_image_window_name = "Image with the drawn shape on the image plane (press \"s\" to save the grayscale image)"  # the name of the window with the drawn shape on the image plane
        self.workspace_image_window_name = "Workspace image shown with the drawn plane"  # the name of shown the workspace image window
        self.obstacles_boundaries_window_name = "Obstacles boundaries detected on the workspace image (\"e\"/\"r\"change boundaries precision, \"t\"/\"y\": change vertices lower limit, left/right click: remove/restore boundaries, \"d\": save boundaries)"  # the name of the detected obstacles boundaries window
        self.camera_aspect_ratios_list = ["16/9", "4/3"]  # the possible aspect ratios of the camera
        self.camera_aspect_ratios_values = [16/9, 4/3]  # the values of the aspect ratios of the camera
        self.camera_aspect_ratio = self.camera_aspect_ratios_values[0]  # the aspect ratio of the camera
        self.camera_resolutions_list = [[720, 360], [480, 240]]  # the possible resolutions of the camera for each aspect ratio
        self.camera_resolution = self.camera_resolutions_list[self.camera_aspect_ratios_values.index(self.camera_aspect_ratio)][0]  # the resolution of the camera
        self.camera_dFoV = 55  # the diagonal field of view of the camera in degrees
        self.images_size_factor = 1  # the factor to resize the images appearing in the windows on the screen
        self.luminance_threshold = 127  # the threshold of the luminance for the camera frames
        self.luminance_threshold_limits = [0, 255]  # the limits of the luminance threshold for the camera frames
        self.camera_optical_axis = np.array([0.0, 0.0, 1.0])  # the optical axis of the camera
        self.camera_optical_axis_prev = np.copy(self.camera_optical_axis)  # the previous optical axis of the camera
        self.camera_translation = np.array([0.0, 0.0, 0.0])  # the translation vector of the camera
        self.camera_translation_prev = np.copy(self.camera_translation)  # the previous translation vector of the camera
        self.camera_orientation = 0.0  # the orientation of the camera in degrees, in the interval [-180, 180)
        self.camera_z_rotation = 0.0  # the angle of the rotation around z-axis of the camera in degrees
        self.camera_translation_displacement = 0.0  # the displacement of the translation along the optical axis of the camera in meters
        self.camera_wrt_world_transformation_matrix = np.eye(4)  # the transformation matrix of the camera
        self.camera_intrinsic_matrix, self.camera_dist_coeffs, self.reprojection_error = cd.load_intrinsic_parameters(self.camera_intrinsic_parameters_file_path)  # the intrinsic parameters of the camera
        self.apply_moving_average_poses_estimation = True  # the flag to apply the moving average filter for the poses estimation
        self.saved_ArUco_markers_list = [line.strip() for line in open(self.aruco_dictionaries_file_path)]  # the list of the ArUco markers
        self.chosen_ArUco_marker = self.saved_ArUco_markers_list[1]  # the chosen ArUco markers
        self.ArUco_marker_positions_list = ["center", "top-left corner", "up-middle edge", "top-right corner", "right-middle edge", "bottom-right corner", "down-middle edge", "bottom-left corner", "left-middle edge"]  # the possible positions of the ArUco markers on the obstacles 2D plane
        self.ArUco_marker_position = self.ArUco_marker_positions_list[0]  # the position of the ArUco marker on the obstacles 2D plane
        self.ArUco_marker_orientations_list = [0, 45, 90, 135, 180, 225, 270, 315]  # the possible orientations of the ArUco markers on the obstacles 2D plane
        self.ArUco_marker_orientation = self.ArUco_marker_orientations_list[0]  # the orientation of the ArUco marker on the obstacles 2D plane
        self.ArUco_wrt_plane_transformation_matrix = np.eye(4)  # the transformation matrix from the obstacles 2D plane to the ArUco marker
        self.saved_workspace_images_list = [file[:-4] for file in os.listdir(self.saved_workspace_images_infos_folder_path) if file.endswith(".jpg")]  # the list of the saved workspace images
        if len(self.saved_workspace_images_list) != 0:  # if there are saved workspace images
            self.shown_workspace_image_name = self.saved_workspace_images_list[0]  # the shown workspace image file
        else:  # if there are no saved workspace images
            self.shown_workspace_image_name = ""  # the shown workspace image file
        self.shown_workspace_image_plane_corners = []  # the corners of the shown workspace image plane (in pixels)
        # define the parameters for the workspace obstacles
        self.obstacles_2d_plane_x_length = 0.5  # the length along the x axis of the 2D plane where the workspace obstacles are located
        self.obstacles_2d_plane_y_length = 0.5  # the length along the y axis of the 2D plane
        self.obstacles_2d_plane_normal_vector = np.array([0.0, 0.0, 1.0])  # the normal vector of the 2D plane
        self.obstacles_2d_plane_normal_vector_prev = np.copy(self.obstacles_2d_plane_normal_vector)  # the previous normal vector of the 2D plane
        self.obstacles_2d_plane_translation = np.array([0.0, 0.0, 0.0])  # the translation vector of the 2D plane
        self.obstacles_2d_plane_translation_prev = np.copy(self.obstacles_2d_plane_translation)  # the previous translation vector of the 2D plane
        self.obstacles_2d_plane_orientation = 0.0  # the orientation of the 2D plane in degrees, in the interval [-180, 180)
        self.obstacles_2d_plane_z_rotation = 0.0  # the angle of the rotation around z-axis of the 2D plane in degrees
        self.obstacles_2d_plane_translation_displacement = 0.0  # the displacement of the translation along the normal vector of the 2D plane in meters
        self.obst_plane_wrt_world_transformation_matrix = np.eye(4)  # the transformation matrix of the obstacles
        self.plane_singularities_tolerance = 1e-3  # the tolerance for the singularities of the 2D plane
        self.plane_singularities_samples = 25  # the number of points samples for the singularities of the 2D plane
        self.saved_obstacles_transformations_list = [file[:-4] for file in os.listdir(self.saved_obstacles_transformations_folder_path) if file.endswith(".txt")]  # the list of the saved obstacles transformations
        if len(self.saved_obstacles_transformations_list) != 0:  # if there are saved obstacles transformations
            self.chosen_plane_obstacles_transformation_name = self.saved_obstacles_transformations_list[0]  # the chosen obstacles transformation name
        else:  # if there are no saved obstacles transformations
            self.chosen_plane_obstacles_transformation_name = ""  # the chosen obstacles transformation name
        if len(self.saved_workspace_images_list) != 0:  # if there are saved workspace images
            self.chosen_detection_workspace_image_name = self.saved_workspace_images_list[0]  # the chosen workspace image for the obstacles detection
        else:  # if there are no saved workspace images
            self.chosen_detection_workspace_image_name = ""  # the chosen workspace image for the obstacles detection
        self.chosen_workspace_saved_obstacles_objects_list = [file[:-4] for file in os.listdir(self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}") if file.endswith(".txt")]  # the list to store the saved obstacles objects for the chosen workspace image
        if len(self.chosen_workspace_saved_obstacles_objects_list) != 0:  # if there are saved obstacles objects for the chosen workspace image
            self.chosen_obstacle_object_name = self.chosen_workspace_saved_obstacles_objects_list[0]  # the chosen obstacle name from the saved obstacles objects
        else:  # if there are no saved obstacles objects for the chosen workspace image
            self.chosen_obstacle_object_name = ""  # the chosen obstacle name from the saved obstacles objects
        self.workspace_obstacles_height_detection = 0.020  # the height of the workspace obstacles for the obstacles detection (in meters)
        self.workspace_obstacles_height_saved = self.workspace_obstacles_height_detection  # the saved height of the workspace obstacles (in meters)
        self.workspace_obstacles_max_height = 0.100  # the maximum height of the obstacles (in meters)
        self.boundaries_precision_parameter = 10  # the precision parameter for the obstacles boundaries detection (it is used to create an error for the boundaries approximation)
        self.boundaries_precision_limits = [0, 10]  # the limits of the precision parameter for the obstacles boundaries detection
        self.boundaries_minimum_vertices = 3  # the minimum number of vertices for the obstacles boundaries detection
        self.boundaries_minimum_vertices_limits = [3, 100]  # the limits of the minimum number of vertices for the obstacles boundaries detection
        self.xy_res_obstacle_mesh_list = [50, 100, 150, 200, 250, 300, 400, 500, 750, 1000]  # the possible values of the x and y axis resolution for the obstacles's mesh
        self.xy_res_obstacle_mesh = 100  # the x and y axis resolution for the obstacle mesh
        self.z_res_obstacle_mesh_list = [10, 25, 50, 100]  # the possible values of the z axis resolution for the obstacle mesh
        self.z_res_obstacle_mesh = 10  # the z axis resolution for the obstacle mesh
        self.workspace_image_plane_frame_wrt_world = np.eye(4)  # the transformation matrix of the image plane frame wrt the world frame for the chosen workspace image
        self.workspace_image_plane_x_length = self.obstacles_2d_plane_x_length  # the length (in meters) along the x axis of the chosen workspace image plane
        self.workspace_image_plane_y_length = self.obstacles_2d_plane_y_length  # the length (in meters) along the y axis of the chosen workspace image plane
        self.workspace_image_plane_image_points = []  # the image corners points (in pixels) of the chosen workspace image plane
        self.workspace_image_camera_frame_wrt_world = np.eye(4)  # the transformation matrix of the camera frame wrt the world frame for the chosen workspace image
        self.removed_boundaries_list = []  # the list to store the removed boundaries of the obstacles
        # define the parameters for the obstacles avoidance solver
        self.obst_avoid_solver_menu_is_enabled = False  # the flag that shows if the obstacles avoidance solver menu is enabled
        self.chosen_solver_workspace_image_name = self.chosen_detection_workspace_image_name  # the chosen workspace image for the obstacles avoidance solver
        self.workspace_plane_creation_parameters_list = ["from\nimage", "manually\ncreated"]  # the possible values to define the workspace plane
        self.workspace_plane_creation_parameter = self.workspace_plane_creation_parameters_list[0]  # the value to define the workspace plane for the obstacles avoidance solver
        self.obstacles_boundaries_for_solver = []  # the real 3D space obstacles boundaries (in meters), transformed on the xy plane of the world frame, for the obstacles avoidance solver
        self.obstacles_height_for_solver = 0.0  # the height of the obstacles (in meters) for the obstacles avoidance solver
        self.start_pos_workspace_plane = np.array([0.0, 0.0])  # the start (x, y) position (wrt the obstacles plane) of the end-effector on the workspace obstacles plane (in meters)
        self.target_pos_workspace_plane = np.array([0.0, 0.0])  # the final (x, y) position (wrt the obstacles plane) of the end-effector on the workspace obstacles plane (in meters)
        self.obstacles_infos_text = "outer boundary and\ninner boundaries infos"  # the text to show the obstacles infos of the current workspace image
        self.boundaries_are_detected = False  # the flag to check if the boundaries of the obstacles are detected
        self.check_positions_on_plane_text = "check positions"  # the text to check the start and target positions of the end-effector on the workspace obstacles plane
        self.positions_on_plane_are_correct = False  # the flag to check if the start and target positions of the end-effector on the workspace obstacles plane are correctly set
        self.check_transformations_built_text = "check transformations"  # the text to check the transformations built for the obstacles avoidance solver
        self.transformations_are_built = False  # the flag to check if the transformations are built for the obstacles avoidance solver
        self.hntf2d_solver = None  # the solver object for the 2D obstacles avoidance
        self.k_d = 1.0  # the target gain kd for the target position qd on the transformed workspace
        self.k_d_limits = [1e-2, 1e3]  # the limits of the target gain kd for the target position qd on the transformed workspace
        self.k_i = [1.0]  # the obstacles gains kis for the obstacles positions qis on the transformed workspace
        self.k_i_limits = [1e-2, 1e3]  # the limits of the obstacles gains kis for the obstacles positions qis on the transformed workspace
        self.k_i_chosen_num = 1  # the currently chosen obstacle gain ki for the obstacle position qi on the transformed workspace
        self.w_phi = 10.0  # the scalar constant w_phi for the navigation function psi
        self.w_phi_limits = [1e-2, 1e3]  # the limits of the scalar constant w_phi for the navigation function psi
        self.vp_max = 0.05  # the maximum value of the velocity p_dot (in m/sec) of the end-effector on the real workspace
        self.vp_max_limits = [1e-3, 1.0]  # the limits of the maximum value of the velocity p_dot (in m/sec) of the end-effector on the real workspace
        self.dp_min = 0.01  # the distance dp_min (in m) of the end-effector from the target position on the real workspace, where the velocity p_dot starts to decrease
        self.dp_min_limits = [1e-3, 1.0]  # the limits of the distance dp_min (in m) of the end-effector from the target position on the real workspace, where the velocity p_dot starts to decrease
        self.saved_control_law_parameters_list = [file[:-4] for file in os.listdir(self.saved_control_law_parameters_folder_path)]  # the list of the saved control law parameters
        if len(self.saved_control_law_parameters_list) != 0:  # if there are saved control law parameters
            self.chosen_control_law_parameters_name = self.saved_control_law_parameters_list[0]  # the chosen control law parameters
        else:  # if there are no saved control law parameters
            self.chosen_control_law_parameters_name = ""  # the chosen control law parameters
        self.navigation_field_plot_points_divs_list = [25, 50, 100, 200, 300, 400, 500, 750, 1000]  # the possible divisions of the plot points for the navigation field
        self.navigation_field_plot_points_divs = 200  # the divisions of the plot points for the navigation field
        self.solver_time_step_dt = 0.010  # the time step for the obstacles avoidance solver (in seconds)
        self.solver_time_step_dt_limits = [1e-3, 1e-1]  # the limits of the time step for the obstacles avoidance solver (in seconds)
        self.solver_error_tolerance = 0.01  # the error tolerance of the control law for the obstacles avoidance solver (in meters)
        self.solver_error_tolerance_limits = [1e-3, 1.0]  # the limits of the error tolerance of the control law for the obstacles avoidance solver (in meters)
        self.solver_enable_error_correction = False  # the flag to enable the error correction of the control law for the obstacles avoidance solver
        self.solver_maximum_iterations = 500  # the maximum number of iterations of the control law for the obstacles avoidance solver
        self.solver_maximum_iterations_limits = [1.0, 1e4]  # the limits of the maximum number of iterations of the control law for the obstacles avoidance solver
        self.paths_parameters_list = []  # the list of the paths parameters for the obstacles avoidance solver
        self.realws_path_control_law_output = []  # the path on the real workspace found by the control law for the obstacles avoidance solver
        self.realws_paths_list = []  # the list of the total paths on the real workspace
        self.unit_disk_path_control_law_output = []  # the path on the unit disk found by the control law for the obstacles avoidance solver
        self.unit_disk_paths_list = []  # the list of the total paths on the unit disk
        self.R2_plane_path_control_law_output = []  # the path on the R2 plane found by the control law for the obstacles avoidance solver
        self.R2_plane_paths_list = []  # the list of the total paths on the R2 plane
        self.path_chosen = 0  # the number of the chosen path on the real workspace
        self.all_paths_are_shown = False  # the flag to check if all the paths on the real workspace are shown
        self.max_paths_stored = 9  # the maximum total number of stored paths allowed
        self.paths_colors = ["#ff0000", "#00ff00", "#0000ff", "#ffa500", "#800080", "#000099", "#a52a2a", "#ff00ff", "#32cd32"]  # the colors of the paths
        self.realws_velocities_control_law_output = []  # the velocities on the real workspace found by the control law for the obstacles avoidance solver
        self.realws_velocities_sequences_list = []  # the list of the total velocities sequences on the real workspace
        self.robot_joints_control_law_output = []  # the robot joints found by the control law for the obstacles avoidance solver
        self.move_robot_time_step_dt = self.solver_time_step_dt  # the time step for the robot control (in seconds)
        self.trajectory_relative_speeds_list = [0.1, 0.2, 0.5, 1.0]  # the possible relative speeds of the trajectory for the robot control
        self.trajectory_relative_speed = 1.0  # the relative speed of the trajectory for the robot control
        self.robot_control_thread_flag = False  # the flag to run/stop the robot control thread
        self.simulated_robot_is_moving = False  # the flag to check if the simulated robot is moving
        self.swift_robot_is_moving = False  # the flag to check if the Swift simulated robot is moving
        self.real_robot_is_moving = False  # the flag to check if the real robot is moving
        # define the variables for the online Swift simulator
        self.swift_simulated_robots_list = [line.strip() for line in open(self.robots_run_by_swift_file_path)]  # the list of the robotic manipulators that can be simulated by the Swift simulator
        self.swift_sim_env = None  # the variable for the Swift simulator environment
        self.swift_sim_thread_flag = False  # the flag to run/stop the Swift simulation thread
        self.swift_sim_dt = 0.01  # the time step of the Swift simulation in seconds
        self.swift_robotic_manipulator = None  # the robotic manipulator model built inside the Swift simulator
        self.swift_sim_2d_plane = None  # the 2D obstacles plane of the Swift simulation
        self.swift_sim_camera = None  # the camera of the Swift simulation
        self.swift_sim_obstacles_objects = []  # the obstacles objects located on the 2D plane for the Swift simulation
        # do some initial actions for the GUI window
        initial_root_width = 0.9 * self.root.winfo_screenwidth()  # the initial width of the root window
        self.workspace_area_width = self.workspace_width_ratio * initial_root_width  # the width of the workspace area
        self.workspace_area_height = 4/5 * self.root.winfo_screenheight()  # the height of the workspace area
        self.workspace_menus_bd_width = 5  # the border width of the workspace and menus areas
        self.menus_area_width = (1 - self.workspace_width_ratio) * initial_root_width  # the width of the menus area
        self.menus_area_height = self.workspace_area_height  # the height of the menus area
        self.root_was_not_in_focus = False  # boolean variable to check if the GUI window was previously minimized or generally not in focus
        self.canvas_fps = 100  # the frames per second of the workspace canvas
        self.canvas_destroy_counter = 0  # the counter to destroy the workspace canvas
        self.create_workspace_menus_areas()  # create the workspace area and the menus area
        self.change_main_menu(self.main_menu_choice)  # create the sub menus for the current main menu
        self.reset_workspace_canvas()  # reset the position of the axis origin to be on the center of the workspace
        self.draw_next_workspace_canvas_frame()  # begin the loop for controlling the motion of the workspace visualization
        # self.load_robotic_manipulator_model("Default")  # load the default robotic manipulator model
        # create the necessary bindings for the GUI window
        self.root.bind("<Control-r>", self.resize_root_window); self.root.bind("<Control-R>", self.resize_root_window)  # bind the resize window function to the Control + r and Control + R keys combinations
        self.root.bind("<Control-s>", self.save_robotic_manipulator_model_file); self.root.bind("<Control-S>", self.save_robotic_manipulator_model_file)  # bind the save robot model function to the Control + s and Control + S keys combinations
        self.root.bind("<Control-b>", self.build_robotic_manipulator_model); self.root.bind("<Control-B>", self.build_robotic_manipulator_model)  # bind the build robot model function to the Control + b and Control + B keys combinations
        self.root.bind("<Control-v>", self.visualize_control_or_kinematics_variables); self.root.bind("<Control-V>", self.visualize_control_or_kinematics_variables)  # bind the visualize control or kinematics variables function to the Control + v and Control + V keys combinations
        self.root.bind("<Control-a>", self.change_drawing_order); self.root.bind("<Control-A>", self.change_drawing_order)  # bind the change drawing order function to the Control + a and Control + A keys combinations
        self.root.bind("<Control-z>", self.visualize_robotic_manipulator); self.root.bind("<Control-Z>", self.visualize_robotic_manipulator)  # bind the visualize robotic manipulator function to the Control + z and Control + Z keys combinations
        self.root.bind("<Control-x>", self.visualize_obstacles); self.root.bind("<Control-X>", self.visualize_obstacles)  # bind the visualize obstacles function to the Control + x and Control + X keys combinations
        self.root.bind("<Control-c>", self.visualize_camera); self.root.bind("<Control-C>", self.visualize_camera)  # bind the visualize camera function to the Control + c and Control + C keys combinations
        self.root.bind("<Control-m>", self.move_simulated_robot_obstacles_avoidance); self.root.bind("<Control-M>", self.move_simulated_robot_obstacles_avoidance)  # bind the move simulated robot for obstacles avoidance function to the Control + m and Control + M keys combinations
        # self.root.bind("<FocusIn>", self.resize_root_window_2)  # bind the resize window function to the focus in event
        # self.root.bind("<Unmap>", self.root_not_in_focus)  # bind the resize window function to the focus out event
        for main_menu_num in range(len(self.main_menus_build_details)):
            self.root.bind(f"<Control-Key-{main_menu_num + 1}>", lambda event, num = main_menu_num: self.change_main_menu(num, event))
        self.root.bind("<Control-e>", lambda event: self.change_workspace_width_ratio(-1, event)); self.root.bind("<Control-E>", lambda event: self.change_workspace_width_ratio(-1, event))  # bind the change workspace width ratio function to the Control + e and Control + E keys combinations
        self.root.bind("<Control-d>", lambda event: self.change_workspace_width_ratio(+1, event)); self.root.bind("<Control-D>", lambda event: self.change_workspace_width_ratio(+1, event))  # bind the change workspace width ratio function to the Control + d and Control + D keys combinations
    def resize_root_window(self, event = None):  # resize the root window
        self.resize_root_window_2()  # resize the root window
    def resize_root_window_2(self, event = None):  # resize the root window
        if event == None or (self.root.state() == "normal" and self.root_was_not_in_focus):  # if the root window is in focus or the resize function is called by another function
            self.workspace_area.destroy()  # destroy the workspace area
            self.menus_area.destroy()  # destroy the menus area
            self.workspace_area_width = self.workspace_width_ratio * self.root.winfo_width() - 4. * self.workspace_menus_bd_width  # recalculate the width of the workspace area
            self.workspace_area_height = self.root.winfo_height() - 4. * self.workspace_menus_bd_width  # recalculate the height of the workspace area
            self.menus_area_width = (1 - self.workspace_width_ratio) * self.root.winfo_width() - 4. * self.workspace_menus_bd_width  # the width of the menus area
            self.menus_area_height = self.workspace_area_height  # the height of the menus area
            self.root_was_not_in_focus = False  # the GUI window was previously in focus
            self.create_workspace_menus_areas()  # recreate the workspace area and the menus area
            self.change_main_menu(self.main_menu_choice)  # recreate the sub menus for the current main menu
            self.reset_workspace_canvas_2()  # reset the position of the axis origin to be on the center of the workspace
            self.draw_next_workspace_canvas_frame()  # begin the loop for controlling the motion of the workspace visualization
    def root_not_in_focus(self, event):  # the function run when the root window is not in focus
        self.root_was_not_in_focus = True  # the GUI window was previously not in focus
    def alternate_matrix_elements(self, matrix, index_element):  # alternate the parametres that are inside the matrix based on the current index_element
        return (matrix[1:] + [matrix[0]])[matrix.index(index_element)]
    def create_workspace_menus_areas(self):  # create the workspace area and the menus area
        # create the workspace area
        self.workspace_area_borders_length = self.workspace_area_height / 12.0  # the length of the borders of the workspace area
        self.workspace_up_edge_height = self.workspace_area_borders_length  # the height of the up edge of the workspace area
        self.workspace_left_edge_width = 2.5 * self.workspace_area_borders_length  # the width of the left edge of the workspace area
        self.workspace_right_edge_width = 0.5 * self.workspace_area_borders_length  # the width of the right edge of the workspace area
        self.workspace_canvas_width = self.workspace_area_width - (self.workspace_left_edge_width + self.workspace_right_edge_width)  # the width of the canvas of the workspace area
        self.workspace_canvas_height = self.workspace_area_height - self.workspace_up_edge_height  # the height of the canvas of the workspace area
        self.switch_coor_system_matrix = np.array([[0, 1, 0, self.workspace_canvas_width / 2], [0, 0, -1, self.workspace_canvas_height / 2], [1, 0, 0, 0], [0, 0, 0, 1]])  # the transformation matrix needed because of the different workspace and canvas coordinates systems
        self.workspace_area_edges_color = "cyan"  # the color of the edges of the workspace area
        self.workspace_area_canvas_color = self.workspace_canvas_color  # the color of the canvas of the workspace area
        self.workspace_options_rows = 27  # the number of rows of the options located at the borders of the workspace area
        self.workspace_borders_font = 9  # the font of the options located at the borders of the workspace area
        self.workspace_title_font = 15  # the font of the title of the workspace area
        self.workspace_area = tk.Frame(self.root, width = self.workspace_area_width, height = self.workspace_area_height, bg = "black", bd = self.workspace_menus_bd_width, relief = "solid")
        self.workspace_area.grid(row = 0, column = 0, sticky = tk.NSEW)
        self.workspace_area_up_edge = tk.Frame(self.workspace_area, width = self.workspace_area_width, height = self.workspace_up_edge_height, bg = self.workspace_area_edges_color, bd = 5, relief = "ridge")
        self.workspace_area_up_edge.grid(row = 0, column = 0, columnspan = 3, sticky = tk.NSEW)
        self.workspace_area_left_edge = tk.Frame(self.workspace_area, width = self.workspace_left_edge_width, height = self.workspace_canvas_height, bg = self.workspace_area_edges_color, bd = 3, relief = "solid")
        self.workspace_area_left_edge.grid(row = 1, column = 0, sticky = tk.NSEW)
        self.workspace_area_right_edge = tk.Frame(self.workspace_area, width = self.workspace_right_edge_width, height = self.workspace_canvas_height, bg = self.workspace_area_edges_color, bd = 3, relief = "solid")
        self.workspace_area_right_edge.grid(row = 1, column = 2, sticky = tk.NSEW)
        workspace_area_title_ord = 1/2.5; workspace_area_title_x = 1/2; gbl.menu_label(self.workspace_area_up_edge, "World frame - Robotic manipulator and its Environment", f"Calibri {self.workspace_title_font} bold", "black", self.workspace_area_edges_color, workspace_area_title_x * self.workspace_area_width, workspace_area_title_ord * self.workspace_up_edge_height)
        self.create_workspace_canvas()  # create the workspace canvas
        # create the workspace options located at the borders of the workspace area
        workspace_width_ratio_label_ord = 1
        workspace_width_ratio_button_ord = workspace_width_ratio_label_ord+1.2
        workspace_sensitivity_label_ord = workspace_width_ratio_button_ord+1
        workspace_sensitivity_button_ord = workspace_sensitivity_label_ord+0.8
        x_axis_range_label_ord = workspace_sensitivity_button_ord+1.4
        x_axis_range_button_ord = x_axis_range_label_ord+1.2
        y_axis_range_label_ord = x_axis_range_label_ord
        y_axis_range_button_ord = y_axis_range_label_ord+1.2
        z_axis_range_label_ord = x_axis_range_label_ord
        z_axis_range_button_ord = z_axis_range_label_ord+1.2
        show_axis_label_ord = z_axis_range_button_ord+1
        show_axis_button_ord = show_axis_label_ord+0.8
        show_terrain_label_ord = show_axis_label_ord
        show_terrain_button_ord = show_terrain_label_ord+0.8
        show_robotic_manipulator_label_ord = show_terrain_button_ord+1
        show_robotic_manipulator_button_ord = show_robotic_manipulator_label_ord+0.8
        show_manipulator_points_label_ord = show_robotic_manipulator_button_ord+1
        show_manipulator_points_button_ord = show_manipulator_points_label_ord+0.8
        show_manipulator_links_label_ord = show_manipulator_points_button_ord+1
        show_manipulator_links_button_ord = show_manipulator_links_label_ord+0.8
        show_end_effector_frame_label_ord = show_manipulator_links_button_ord+1
        show_end_effector_frame_button_ord = show_end_effector_frame_label_ord+0.8
        show_obstacles_plane_label_ord = show_end_effector_frame_button_ord+1
        show_obstacles_plane_button_ord = show_obstacles_plane_label_ord+0.8
        show_obstacles_objects_label_ord = show_obstacles_plane_button_ord+1
        show_obstacles_objects_button_ord = show_obstacles_objects_label_ord+0.8
        show_camera_label_ord = show_obstacles_objects_button_ord+1
        show_camera_button_ord = show_camera_label_ord+0.8
        choose_drawing_order_label_ord = show_camera_button_ord+1
        choose_drawing_order_button_ord = choose_drawing_order_label_ord+0.8
        x_axis_view_label_ord = choose_drawing_order_button_ord+1.4
        x_axis_view_button_ord = x_axis_view_label_ord+1.2
        y_axis_view_label_ord = x_axis_view_label_ord
        y_axis_view_button_ord = y_axis_view_label_ord+1.2
        z_axis_view_label_ord = x_axis_view_label_ord
        z_axis_view_button_ord = z_axis_view_label_ord+1.2
        information_text_label_ord = self.workspace_options_rows
        workspace_width_ratio_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Workspace / window\nwidth ratio:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, workspace_width_ratio_label_x * self.workspace_left_edge_width, workspace_width_ratio_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        workspace_width_ratio_button_x = workspace_width_ratio_label_x; self.change_workspace_width_ratio_button = gbl.menu_button(self.workspace_area_left_edge, self.workspace_width_ratio, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, workspace_width_ratio_button_x * self.workspace_left_edge_width, workspace_width_ratio_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), lambda event: self.change_workspace_width_ratio(+1, event)).button
        workspace_sensitivity_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Control sensitivity:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, workspace_sensitivity_label_x * self.workspace_left_edge_width, workspace_sensitivity_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        workspace_sensitivity_button_x = workspace_sensitivity_label_x; self.change_workspace_sensitivity_button = gbl.menu_button(self.workspace_area_left_edge, self.workspace_sensitivity_degrees[self.workspace_sensitivity_values.index(self.workspace_control_sensitivity)], f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, workspace_sensitivity_button_x * self.workspace_left_edge_width, workspace_sensitivity_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_workspace_control_sensitivity).button
        x_axis_range_label_x = 1/4; gbl.menu_label(self.workspace_area_left_edge, "x axis\n(m):", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, x_axis_range_label_x * self.workspace_left_edge_width, x_axis_range_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        x_axis_range_button_x = x_axis_range_label_x; self.change_x_axis_range_button = gbl.menu_button(self.workspace_area_left_edge, self.x_axis_range, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, x_axis_range_button_x * self.workspace_left_edge_width, x_axis_range_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_x_axis_range).button
        y_axis_range_label_x = 2/4; gbl.menu_label(self.workspace_area_left_edge, "y axis\n(m):", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, y_axis_range_label_x * self.workspace_left_edge_width, y_axis_range_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        y_axis_range_button_x = y_axis_range_label_x; self.change_y_axis_range_button = gbl.menu_button(self.workspace_area_left_edge, self.y_axis_range, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, y_axis_range_button_x * self.workspace_left_edge_width, y_axis_range_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_y_axis_range).button
        z_axis_range_label_x = 3/4; gbl.menu_label(self.workspace_area_left_edge, "z axis\n(m):", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, z_axis_range_label_x * self.workspace_left_edge_width, z_axis_range_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        z_axis_range_button_x = z_axis_range_label_x; self.change_z_axis_range_button = gbl.menu_button(self.workspace_area_left_edge, self.z_axis_range, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, z_axis_range_button_x * self.workspace_left_edge_width, z_axis_range_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_z_axis_range).button
        show_axis_label_x = 1/3; gbl.menu_label(self.workspace_area_left_edge, "Axis:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_axis_label_x * self.workspace_left_edge_width, show_axis_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_axis_button_x = show_axis_label_x; self.show_axis_button = gbl.menu_button(self.workspace_area_left_edge, self.axis_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_axis_button_x * self.workspace_left_edge_width, show_axis_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_axis).button
        show_terrain_label_x = 2/3; gbl.menu_label(self.workspace_area_left_edge, "Terrain:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_terrain_label_x * self.workspace_left_edge_width, show_terrain_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_terrain_button_x = show_terrain_label_x; self.show_terrain_button = gbl.menu_button(self.workspace_area_left_edge, self.terrain_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_terrain_button_x * self.workspace_left_edge_width, show_terrain_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_terrain).button
        show_robotic_manipulator_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Robotic manipulator:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_robotic_manipulator_label_x * self.workspace_left_edge_width, show_robotic_manipulator_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_robotic_manipulator_button_x = show_robotic_manipulator_label_x; self.show_robotic_manipulator_button = gbl.menu_button(self.workspace_area_left_edge, self.robotic_manipulator_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_robotic_manipulator_button_x * self.workspace_left_edge_width, show_robotic_manipulator_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_robotic_manipulator).button
        show_manipulator_points_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Manipulator points:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_manipulator_points_label_x * self.workspace_left_edge_width, show_manipulator_points_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_manipulator_points_button_x = show_manipulator_points_label_x; self.show_manipulator_points_button = gbl.menu_button(self.workspace_area_left_edge, self.robot_joints_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_manipulator_points_button_x * self.workspace_left_edge_width, show_manipulator_points_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_manipulator_joints).button
        show_manipulator_links_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Manipulator links:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_manipulator_links_label_x * self.workspace_left_edge_width, show_manipulator_links_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_manipulator_links_button_x = show_manipulator_links_label_x; self.show_manipulator_links_button = gbl.menu_button(self.workspace_area_left_edge, self.robot_links_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_manipulator_links_button_x * self.workspace_left_edge_width, show_manipulator_links_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_manipulator_links).button
        show_end_effector_frame_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "End-effector frame:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_end_effector_frame_label_x * self.workspace_left_edge_width, show_end_effector_frame_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_end_effector_frame_button_x = show_end_effector_frame_label_x; self.show_end_effector_frame_button = gbl.menu_button(self.workspace_area_left_edge, self.end_effector_frame_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_end_effector_frame_button_x * self.workspace_left_edge_width, show_end_effector_frame_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_end_effector_frame).button
        show_obstacles_plane_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Obstacles plane:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_obstacles_plane_label_x * self.workspace_left_edge_width, show_obstacles_plane_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_obstacles_plane_button_x = 1/3; self.show_obstacles_plane_button = gbl.menu_button(self.workspace_area_left_edge, self.obstacles_plane_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_obstacles_plane_button_x * self.workspace_left_edge_width, show_obstacles_plane_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_obstacles_plane).button
        show_obstacles_plane_frame_button_x = 2/3; self.show_obstacles_plane_frame_button = gbl.menu_button(self.workspace_area_left_edge, self.plane_frame_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_obstacles_plane_frame_button_x * self.workspace_left_edge_width, show_obstacles_plane_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_obstacles_plane_frame).button
        show_obstacles_objects_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Obstacles objects:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_obstacles_objects_label_x * self.workspace_left_edge_width, show_obstacles_objects_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_obstacles_objects_button_x = show_obstacles_objects_label_x; self.show_obstacles_objects_button = gbl.menu_button(self.workspace_area_left_edge, self.obstacles_objects_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_obstacles_objects_button_x * self.workspace_left_edge_width, show_obstacles_objects_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_obstacles_objects).button
        show_camera_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Camera:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_camera_label_x * self.workspace_left_edge_width, show_camera_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_camera_button_x = 1/3; self.show_camera_button = gbl.menu_button(self.workspace_area_left_edge, self.camera_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_camera_button_x * self.workspace_left_edge_width, show_camera_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_camera).button
        show_camera_frame_button_x = 2/3; self.show_camera_frame_button = gbl.menu_button(self.workspace_area_left_edge, self.camera_frame_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_camera_frame_button_x * self.workspace_left_edge_width, show_camera_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_camera_frame).button
        choose_drawing_order_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Drawing order:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, choose_drawing_order_label_x * self.workspace_left_edge_width, choose_drawing_order_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        choose_drawing_order_button_x = choose_drawing_order_label_x; self.choose_drawing_order_button = gbl.menu_button(self.workspace_area_left_edge, self.drawing_order, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, choose_drawing_order_button_x * self.workspace_left_edge_width, choose_drawing_order_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_drawing_order).button
        x_axis_view_label_x = 1/4; gbl.menu_label(self.workspace_area_left_edge, "x axis\nview:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, x_axis_view_label_x * self.workspace_left_edge_width, x_axis_view_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        x_axis_view_button_x = x_axis_view_label_x; self.change_x_axis_view_button = gbl.menu_button(self.workspace_area_left_edge, self.x_axis_view, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, x_axis_view_button_x * self.workspace_left_edge_width, x_axis_view_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_x_axis_view).button
        y_axis_view_label_x = 2/4; gbl.menu_label(self.workspace_area_left_edge, "y axis\nview:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, y_axis_view_label_x * self.workspace_left_edge_width, y_axis_view_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        y_axis_view_button_x = y_axis_view_label_x; self.change_y_axis_view_button = gbl.menu_button(self.workspace_area_left_edge, self.y_axis_view, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, y_axis_view_button_x * self.workspace_left_edge_width, y_axis_view_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_y_axis_view).button
        z_axis_view_label_x = 3/4; gbl.menu_label(self.workspace_area_left_edge, "z axis\nview:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, z_axis_view_label_x * self.workspace_left_edge_width, z_axis_view_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        z_axis_view_button_x = z_axis_view_label_x; self.change_z_axis_view_button = gbl.menu_button(self.workspace_area_left_edge, self.z_axis_view, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, z_axis_view_button_x * self.workspace_left_edge_width, z_axis_view_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_z_axis_view).button
        information_text_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "If the window freezes\nyou can press \"Ctrl-R\"\n(used for resizing too).", f"Calibri {self.workspace_borders_font} bold", "brown", self.workspace_area_edges_color, information_text_label_x * self.workspace_left_edge_width, information_text_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        # create the menus area
        self.menus_area_borders_length = self.workspace_area_borders_length  # the length of the borders of the menus area
        self.menus_background_width = self.menus_area_width  # the width of the menus background
        self.menus_background_height = self.menus_area_height - self.menus_area_borders_length  # the height of the menus background
        self.menus_area_edges_color = "lime"
        self.menus_area_color = "orange"
        self.menus_title_font = 15  # the font of the title of the menus area
        self.menus_area = tk.Frame(self.root, width = self.menus_area_width, height = self.menus_area_height, bg = self.menus_area_color, bd = self.workspace_menus_bd_width, relief = "solid")
        self.menus_area.grid(row = 0, column = 1, sticky = tk.NSEW)
        self.menus_area_up_edge = tk.Frame(self.menus_area, width = self.menus_area_width, height = self.menus_area_borders_length, bg = self.menus_area_edges_color, bd = 5, relief = "ridge")
        self.menus_area_up_edge.grid(row = 0, column = 0, sticky = tk.NSEW)
        menus_area_title_ord = 1/2.5; menus_area_title_x = 1/2; self.main_menu_label = gbl.menu_label(self.menus_area_up_edge, self.main_menus_build_details[self.main_menu_choice]['title'], f"Calibri {self.menus_title_font} bold", "black", self.menus_area_edges_color, menus_area_title_x * self.menus_area_width, menus_area_title_ord * self.menus_area_borders_length).label
        previous_main_menu_button_ord = 1/2.5; previous_main_menu_button_x = 1/6; self.previous_main_menu_button = gbl.menu_button(self.menus_area_up_edge, "⇦", f"Calibri {self.menus_title_font} bold", "red", self.menus_area_edges_color, previous_main_menu_button_x * self.menus_area_width, previous_main_menu_button_ord * self.menus_area_borders_length, lambda event: self.change_main_menu(self.main_menu_choice-1, event))
        next_main_menu_button_ord = 1/2.5; next_main_menu_button_x = 5/6; self.next_main_menu_button = gbl.menu_button(self.menus_area_up_edge, "⇨", f"Calibri {self.menus_title_font} bold", "red", self.menus_area_edges_color, next_main_menu_button_x * self.menus_area_width, next_main_menu_button_ord * self.menus_area_borders_length, lambda event: self.change_main_menu(self.main_menu_choice+1, event))
        self.clear_menus_background()  # clear the menus background
    def create_workspace_canvas(self, event = None):  # create the workspace canvas where the robotic manipulator and its environment are visualized
        try: self.workspace_canvas.destroy()  # destroy the previous workspace canvas
        except: pass
        # create the workspace canvas
        self.workspace_canvas = tk.Canvas(self.workspace_area, width = self.workspace_canvas_width, height = self.workspace_canvas_height, bg = self.workspace_canvas_color, bd = 3, relief = "raised")
        self.workspace_canvas.grid(row = 1, column = 1, sticky = tk.NSEW)
        # create the buttons existing on the canvas
        try:
            self.choose_visualized_variables_button.destroy()
            self.visualized_variables_values_indicator.destroy()
        except: pass
        self.choose_visualized_variables_button = gbl.menu_button(self.workspace_canvas, self.control_or_kinematics_variables_visualization, f"Calibri {int(1.0*self.workspace_borders_font)} bold", "black", self.workspace_canvas_color, 50, 50, self.visualize_control_or_kinematics_variables).button
        self.visualized_variables_values_indicator = gbl.menu_label(self.workspace_canvas, "", f"Calibri {int(1.0*self.workspace_borders_font)} bold", "black", self.workspace_canvas_color, 50, self.workspace_canvas_height / 2).label
        # create the necessary bindings for the robotic manipulator workspace visualization
        self.workspace_canvas.bind("<Button-3>", lambda event: self.transfer_workspace_start(event))
        self.workspace_canvas.bind("<B3-Motion>", lambda event: self.transfer_workspace(event))
        self.workspace_canvas.bind("<Double-Button-3>", lambda event: self.reset_workspace_canvas_2(event))
        self.workspace_canvas.bind("<Button-1>", lambda event: self.rotate_workspace_start(event))
        self.workspace_canvas.bind("<B1-Motion>", lambda event: self.rotate_workspace(event))
        self.workspace_canvas.bind("<MouseWheel>", lambda event: self.scale_workspace(event))

    # functions for the workspace options located at the borders of the workspace area
    def change_workspace_width_ratio(self, change_type = +1, event = None):  # change the width ratio of the workspace area to the root window
        if change_type == +1:  # if the width ratio of the workspace area is increased
            self.workspace_width_ratio = self.alternate_matrix_elements(self.workspace_width_ratios_values, self.workspace_width_ratio)
        elif change_type == -1:  # if the width ratio of the workspace area is decreased
            self.workspace_width_ratio = self.alternate_matrix_elements(self.workspace_width_ratios_values[::-1], self.workspace_width_ratio)
        self.change_workspace_width_ratio_button.configure(text = self.workspace_width_ratio)
        self.resize_root_window()  # resize the root window
    def change_workspace_control_sensitivity(self, event = None):  # change the workspace mouse control sensitivity
        self.workspace_control_sensitivity = self.alternate_matrix_elements(self.workspace_sensitivity_values, self.workspace_control_sensitivity)
        self.change_workspace_sensitivity_button.configure(text = self.workspace_sensitivity_degrees[self.workspace_sensitivity_values.index(self.workspace_control_sensitivity)])
    def change_x_axis_range(self, event = None):  # change the x axis range
        self.x_axis_range = self.alternate_matrix_elements(self.axis_range_values, self.x_axis_range)
        self.change_x_axis_range_button.configure(text = self.x_axis_range)
    def change_y_axis_range(self, event = None):  # change the y axis range
        self.y_axis_range = self.alternate_matrix_elements(self.axis_range_values, self.y_axis_range)
        self.change_y_axis_range_button.configure(text = self.y_axis_range)
    def change_z_axis_range(self, event = None):  # change the z axis range
        self.z_axis_range = self.alternate_matrix_elements(self.axis_range_values, self.z_axis_range)
        self.change_z_axis_range_button.configure(text = self.z_axis_range)
    def show_axis(self, event = None):  # show or hide the axis
        self.axis_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.axis_enable_visualization)
        self.show_axis_button.configure(text = self.axis_enable_visualization)
    def show_terrain(self, event = None):  # show or hide the terrain
        self.terrain_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.terrain_enable_visualization)
        self.show_terrain_button.configure(text = self.terrain_enable_visualization)
    def show_robotic_manipulator(self, event = None):  # show or hide the robotic manipulator
        self.robotic_manipulator_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.robotic_manipulator_enable_visualization)
        self.show_robotic_manipulator_button.configure(text = self.robotic_manipulator_enable_visualization)
    def show_manipulator_joints(self, event = None):  # show or hide the joints of the robotic manipulator
        self.robot_joints_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.robot_joints_enable_visualization)
        self.show_manipulator_points_button.configure(text = self.robot_joints_enable_visualization)
    def show_manipulator_links(self, event = None):  # show or hide the links of the robotic manipulator
        self.robot_links_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.robot_links_enable_visualization)
        self.show_manipulator_links_button.configure(text = self.robot_links_enable_visualization)
    def show_end_effector_frame(self, event = None):  # show or hide the frame of the end-effector
        self.end_effector_frame_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.end_effector_frame_enable_visualization)
        self.show_end_effector_frame_button.configure(text = self.end_effector_frame_enable_visualization)
    def show_obstacles_plane(self, event = None):  # show or hide the obstacles of the workspace
        self.obstacles_plane_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.obstacles_plane_enable_visualization)
        self.show_obstacles_plane_button.configure(text = self.obstacles_plane_enable_visualization)
    def show_obstacles_plane_frame(self, event = None):  # show or hide the frame of the obstacles plane
        self.plane_frame_enable_visualization = self.alternate_matrix_elements(["frame", "plain"], self.plane_frame_enable_visualization)
        self.show_obstacles_plane_frame_button.configure(text = self.plane_frame_enable_visualization)
    def show_obstacles_objects(self, event = None):  # show or hide the obstacles objects
        self.obstacles_objects_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.obstacles_objects_enable_visualization)
        self.show_obstacles_objects_button.configure(text = self.obstacles_objects_enable_visualization)
    def show_camera(self, event = None):  # show or hide the camera object
        self.camera_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.camera_enable_visualization)
        self.show_camera_button.configure(text = self.camera_enable_visualization)
    def show_camera_frame(self, event = None):  # show or hide the frame of the camera object
        self.camera_frame_enable_visualization = self.alternate_matrix_elements(["frame", "plain"], self.camera_frame_enable_visualization)
        self.show_camera_frame_button.configure(text = self.camera_frame_enable_visualization)
    def change_drawing_order(self, event = None):  # change the drawing order of the robotic manipulator and the obstacles
        self.drawing_order = self.alternate_matrix_elements(self.drawing_order_list, self.drawing_order)
        self.choose_drawing_order_button.configure(text = self.drawing_order)
    def change_x_axis_view(self, event = None):  # change the x axis view
        self.x_axis_view = self.alternate_matrix_elements(["0", "-", "+"], self.x_axis_view)
        self.change_x_axis_view_button.configure(text = self.x_axis_view)
        self.set_canvas_view()
    def change_y_axis_view(self, event = None):  # change the y axis view
        self.y_axis_view = self.alternate_matrix_elements(["0", "-", "+"], self.y_axis_view)
        self.change_y_axis_view_button.configure(text = self.y_axis_view)
        self.set_canvas_view()
    def change_z_axis_view(self, event = None):  # change the z axis view
        self.z_axis_view = self.alternate_matrix_elements(["0", "-", "+"], self.z_axis_view)
        self.change_z_axis_view_button.configure(text = self.z_axis_view)
        self.set_canvas_view()
    def visualize_control_or_kinematics_variables(self, event = None):  # choose the control or forward kinematics variables
        self.control_or_kinematics_variables_visualization = self.alternate_matrix_elements(self.control_or_kinematics_variables_visualization_list, self.control_or_kinematics_variables_visualization)
        self.update_model_visualization_indicators()  # update the indicators of the model visualization
        self.update_differential_kinematics_indicators()  # update the indicators of the differential kinematics
        self.update_inverse_differential_kinematics_indicators()  # update the indicators of the inverse differential kinematics
    def visualize_robotic_manipulator(self, event = None):  # visualize or hide the robotic manipulator
        if self.robotic_manipulator_enable_visualization == "shown":  # if the robotic manipulator is shown
            self.robotic_manipulator_enable_visualization = "hidden"
            self.robot_joints_enable_visualization = "hidden"
            self.robot_links_enable_visualization = "hidden"
            self.end_effector_frame_enable_visualization = "hidden"
        else:  # if the robotic manipulator is hidden
            self.robotic_manipulator_enable_visualization = "shown"
            self.robot_joints_enable_visualization = "shown"
            self.robot_links_enable_visualization = "shown"
            self.end_effector_frame_enable_visualization = "shown"
        self.show_robotic_manipulator_button.configure(text = self.robotic_manipulator_enable_visualization)
        self.show_manipulator_points_button.configure(text = self.robot_joints_enable_visualization)
        self.show_manipulator_links_button.configure(text = self.robot_links_enable_visualization)
        self.show_end_effector_frame_button.configure(text = self.end_effector_frame_enable_visualization)
    def visualize_obstacles(self, event = None):  # visualize or hide the obstacles
        if self.obstacles_plane_enable_visualization == "shown" or self.plane_frame_enable_visualization == "frame" or self.obstacles_objects_enable_visualization == "shown":  # if the obstacles are shown in any form
            self.obstacles_plane_enable_visualization = "hidden"
            self.plane_frame_enable_visualization = "plain"
            self.obstacles_objects_enable_visualization = "hidden"
        else:  # if the obstacles are hidden
            self.obstacles_plane_enable_visualization = "shown"
            self.plane_frame_enable_visualization = "frame"
            self.obstacles_objects_enable_visualization = "shown"
        self.show_obstacles_plane_button.configure(text = self.obstacles_plane_enable_visualization)
        self.show_obstacles_plane_frame_button.configure(text = self.plane_frame_enable_visualization)
        self.show_obstacles_objects_button.configure(text = self.obstacles_objects_enable_visualization)
    def visualize_camera(self, event = None):  # visualize or hide the camera
        if self.camera_enable_visualization == "shown" or self.camera_frame_enable_visualization == "frame":  # if the camera is shown in any form
            self.camera_enable_visualization = "hidden"
            self.camera_frame_enable_visualization = "plain"
        else:  # if the camera is hidden
            self.camera_enable_visualization = "shown"
            self.camera_frame_enable_visualization = "frame"
        self.show_camera_button.configure(text = self.camera_enable_visualization)
        self.show_camera_frame_button.configure(text = self.camera_frame_enable_visualization)

    # functions for the control of the workspace
    def apply_workspace_transformation(self, event = None):  # apply the transformation defined by the proper transfer, rotation and scale variables to all the points of the workspace
        self.workspace_transfer_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, self.y_cor_workspace_origin], [0, 0, 1, self.z_cor_workspace_origin], [0, 0, 0, 1]])
        self.workspace_scale_matrix = np.array([[self.magnify_workspace_constant * self.scale_parameter, 0, 0, 0], [0, self.magnify_workspace_constant * self.scale_parameter, 0, 0], [0, 0, self.magnify_workspace_constant * self.scale_parameter, 0], [0, 0, 0, 1]])
        self.workspace_y_rot_matrix = np.array([[np.cos(np.deg2rad(self.rot_y_workspace)), 0, np.sin(np.deg2rad(self.rot_y_workspace)), 0], [0, 1, 0, 0], [-np.sin(np.deg2rad(self.rot_y_workspace)), 0, np.cos(np.deg2rad(self.rot_y_workspace)), 0], [0, 0, 0, 1]])
        self.workspace_z_rot_matrix = np.array([[np.cos(np.deg2rad(self.rot_z_workspace)), -np.sin(np.deg2rad(self.rot_z_workspace)), 0, 0], [np.sin(np.deg2rad(self.rot_z_workspace)), np.cos(np.deg2rad(self.rot_z_workspace)), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.workspace_rotation_matrix = self.workspace_y_rot_matrix @ self.workspace_z_rot_matrix
        self.workspace_transformation_matrix = self.workspace_transfer_matrix @ self.workspace_rotation_matrix @ self.workspace_scale_matrix
        transformed_axis_terrain_points = np.copy(self.axis_terrain_points)  # the transformed points of the axis and terrain
        transformed_robotic_manipulator_points = self.apply_robotic_manipulator_transformation()  # the transformed points of the robotic manipulator
        transformed_end_effector_points = self.apply_end_effector_transformation(self.end_effector_frame_points)  # the transformed points of the robotic manipulator end-effector
        transformed_obstacles_plane_points = self.apply_obstacles_transformation(self.obstacles_plane_points)  # the transformed points of the obstacles plane
        transformed_plane_frame_points = self.apply_obstacles_transformation(self.plane_frame_points)  # the transformed points of the obstacles frame
        transformed_obstacles_objects_points = self.apply_obstacles_transformation(self.obstacles_objects_points)  # the transformed points of the obstacles objects
        transformed_camera_device_points = self.apply_camera_transformation(self.camera_device_points)  # the transformed points of the camera device
        transformed_camera_frame_points = self.apply_camera_transformation(self.camera_frame_points)  # the transformed points of the camera frame        
        self.workspace_canvas_points = np.concatenate((transformed_axis_terrain_points, transformed_robotic_manipulator_points, transformed_end_effector_points, \
                                                        transformed_obstacles_plane_points, transformed_plane_frame_points, transformed_obstacles_objects_points, transformed_camera_device_points, transformed_camera_frame_points), axis = 0)
        self.canvas_moved_points = (self.switch_coor_system_matrix @ self.workspace_transformation_matrix @ self.workspace_canvas_points.T).T  # the moved points of the workspace, converted to canvas coordinates, after the workspace transformation is applied
        # for the visualization, based on the orientation of the terrain plane
        terrain_draw_side = np.dot(self.workspace_rotation_matrix[:3, :3] @ np.array([0, 0, 1]), np.array([1, 0, 0]))  # the draw side of the axis and terrain
        if terrain_draw_side >= 0:  # if the up side of the terrain is visible
            self.workspace_terrain_color = self.up_side_terrain_color  # the color of the up side of the terrain
        else:  # if the down side of the terrain is visible
            self.workspace_terrain_color = self.down_side_terrain_color  # the color of the down side of the terrain
        # for the visualization, based on the orientation of the workspace plane
        obstacles_2d_plane_draw_side = np.dot(self.workspace_rotation_matrix[:3, :3] @ self.obst_plane_wrt_world_transformation_matrix[:3, :3] @ np.array([0, 0, 1]), np.array([1, 0, 0]))  # the draw side of the 2D plane
        if obstacles_2d_plane_draw_side >= 0:  # if the front side of the 2D plane is visible
            self.workspace_2d_plane_color = self.front_side_2d_plane_color  # the color of the front side of the 2D plane
        else:  # if the back side of the 2D plane is visible
            self.workspace_2d_plane_color = self.back_side_2d_plane_color  # the color of the back side of the 2D plane
    def apply_robotic_manipulator_transformation(self, event = None):  # apply the transformation defined by the proper transfer and rotation matrices to the points of the robotic manipulator
        if self.robotic_manipulator_is_built:  # if a robotic manipulator is built
            # q_joints = self.get_robot_joints_variables(self.control_or_kinematics_variables_visualization)
            self.built_robotic_manipulator.q = self.get_robot_joints_variables(self.control_or_kinematics_variables_visualization)  # get the proper values for the joints variables (control or fkine)
            frames_origins, frames_orientations = self.get_all_frames_positions_orientations(self.built_robotic_manipulator.q)  # get the positions and the orientations of the frames of the robotic manipulator
            robotic_manipulator_points = [frames_origins[k].tolist() + [1.0] for k in range(len(frames_origins))]  # add the homogeneous coordinate to the origins of the frames of the robotic manipulator
            # add the points of the joints of the robotic manipulator
            for k in range(self.joints_number):  # for all the joints of the robotic manipulator
                robotic_manipulator_points.append((frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]).tolist() + [1.0])  # add the points of the joints of the robotic manipulator
            robotic_manipulator_points.append(frames_origins[0].tolist() + [1.0])  # add the origin of the base frame
            for k in range(self.joints_number):  # for all the joints of the robotic manipulator
                robotic_manipulator_points.append((frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]).tolist() + [1.0])  # add the points of the joints of the robotic manipulator
            robotic_manipulator_points.append(frames_origins[-1].tolist() + [1.0])  # add the origin of the end-effector frame
            return robotic_manipulator_points  # return the transformed points of the robotic manipulator
        else:  # if no robotic manipulator is built
            return [[0.0, 0.0, 0.0, 1.0] for k in range(self.robotic_manipulator_points_num)]  # return the default points of the robotic manipulator
    def apply_end_effector_transformation(self, end_effector_frame_points, event = None):  # apply the transformation defined by the proper transfer and rotation matrices to the points of the end-effector
        if self.robotic_manipulator_is_built:  # if a robotic manipulator is built
            q_joints = self.get_robot_joints_variables(self.control_or_kinematics_variables_visualization)  # get the proper values for the joints variables (control or fkine)
            end_effector_frame_origin, end_effector_frame_orientation = self.get_fkine_frame_position_orientation(q_joints, "end-effector")  # get the position and the orientation of the end-effector frame
            end_effector_T = np.eye(4)  # the transformation matrix of the end-effector frame
            end_effector_T[:3, :3] = end_effector_frame_orientation  # update the rotation matrix of the end-effector frame
            end_effector_T[:3, 3] = end_effector_frame_origin  # update the translation vector of the end-effector frame
            return (end_effector_T @ end_effector_frame_points.T).T  # return the transformed points of the end-effector
        else:  # if no robotic manipulator is built
            return (np.eye(4) @ end_effector_frame_points.T).T  # return the points of the end-effector
    def apply_obstacles_transformation(self, obstacles_points, event = None):  # apply the transformation defined by the proper transfer and rotation matrices to the points of the obstacles
        Tobs_total = self.obst_plane_wrt_world_transformation_matrix  # the total transformation matrix of the obstacles points
        return (Tobs_total @ obstacles_points.T).T  # return the transformed points of the obstacles that lie on the 2D plane
    def apply_camera_transformation(self, camera_points, event = None):  # apply the transformation defined by the proper transfer and rotation matrices to the points of the camera
        Tcam_total = self.camera_wrt_world_transformation_matrix  # the total transformation matrix of the camera points
        return (Tcam_total @ camera_points.T).T  # return the transformed points of the camera
    def reset_workspace_canvas(self, event = None):  # reset the workspace to its initial state
        self.scale_parameter = 1.0  # initialize the scale parameter of the workspace
        self.y_cor_workspace_origin = 0.0; self.z_cor_workspace_origin = 0.0  # initialize the coordinates of the origin of the workspace
        self.rot_y_workspace = 0.0; self.rot_z_workspace = 0.0  # initialize the rotation angles of the workspace
        self.last_transfer_y = 0.0; self.last_transfer_z = 0.0  # initialize the coordinates of the last mouse position when the user starts to transfer the workspace
        self.last_rotation_y = 0.0; self.last_rotation_z = 0.0  # initialize the coordinates of the last mouse position when the user starts to rotate the workspace
    def reset_workspace_canvas_2(self, event = None):  # reset the workspace to its initial state
        self.y_cor_workspace_origin = 0; self.z_cor_workspace_origin = 0  # initialize the coordinates of the origin of the workspace
    def transfer_workspace_start(self, event):  # initialize the coordinates of the last mouse position when the user starts to transfer the workspace
        self.last_transfer_y = event.x
        self.last_transfer_z = event.y
    def transfer_workspace(self, event):  # transfer the workspace according to the mouse movement
        self.y_cor_workspace_origin = self.y_cor_workspace_origin + 2 * self.workspace_control_sensitivity * (event.x - self.last_transfer_y)
        self.z_cor_workspace_origin = self.z_cor_workspace_origin - 2 * self.workspace_control_sensitivity * (event.y - self.last_transfer_z)
        self.last_transfer_y = event.x
        self.last_transfer_z = event.y
    def scale_workspace(self, event):  # scale the workspace according to the mouse wheel movement
        if event.delta == -120 and self.scale_parameter >= 0.2:
            self.scale_parameter -= self.workspace_control_sensitivity / 5
        elif event.delta == 120 and self.scale_parameter <= 15:
            self.scale_parameter += self.workspace_control_sensitivity / 5
    def rotate_workspace_start(self, event):  # initialize the coordinates of the last mouse position when the user starts to rotate the workspace
        self.last_rotation_y = event.y
        self.last_rotation_z = event.x
    def rotate_workspace(self, event):  # rotate the workspace according to the mouse movement
        self.rot_y_workspace = self.rot_y_workspace + self.workspace_control_sensitivity / 2 * (event.y - self.last_rotation_y)
        self.rot_z_workspace = self.rot_z_workspace + self.workspace_control_sensitivity / 2 * (event.x - self.last_rotation_z)
        self.last_rotation_y = event.y
        self.last_rotation_z = event.x
    def set_canvas_view(self, event = None):  # set the canvas view based on the x, y and z axis defined views
        x_axis_view = [0, -1, 1][["0", "-", "+"].index(self.x_axis_view)]
        y_axis_view = [0, -1, 1][["0", "-", "+"].index(self.y_axis_view)]
        z_axis_view = [0, -1, 1][["0", "-", "+"].index(self.z_axis_view)]
        self.rot_y_workspace = z_axis_view * [45, 90][[False, True].index(x_axis_view == 0 and y_axis_view == 0)]
        if y_axis_view == 0: self.rot_z_workspace = [4, 0, 0][[-1, 0, 1].index(x_axis_view)] * 45
        else: self.rot_z_workspace = (x_axis_view - 2) * (y_axis_view) * 45
    
    # functions for the visualization of the workspace
    def create_initial_workspace_canvas_points_connections(self, event = None):  # create the points and edges of the workspace (axis and the robotic manipulator)
        # define all the points of the workspace
        # create the axis and terrain points and edges
        self.axis_terrain_points = []  # initialize the points of the axis and terrain
        self.axis_terrain_points.append([0, 0, 0, 1])
        self.axis_terrain_points.append([self.x_axis_range, 0, 0, 1])
        self.axis_terrain_points.append([0, self.y_axis_range, 0, 1])
        self.axis_terrain_points.append([0, 0, self.z_axis_range, 1])
        self.axis_terrain_points.append([0, 0, 0, 1])
        self.axis_terrain_points.append([self.x_axis_range, self.y_axis_range, 0, 1])
        self.axis_terrain_points.append([-self.x_axis_range, self.y_axis_range, 0, 1])
        self.axis_terrain_points.append([-self.x_axis_range, -self.y_axis_range, 0, 1])
        self.axis_terrain_points.append([self.x_axis_range, -self.y_axis_range, 0, 1])
        self.axis_terrain_points = np.array(self.axis_terrain_points, dtype = float)  # convert the points of the axis and terrain to a numpy array
        self.axis_terrain_points_num = len(self.axis_terrain_points)  # update the number of the axis and terrain points
        self.axis_terrain_edges = self.axis_terrain_points_num * [None]  # initialize the edges of the axis and terrain
        self.axis_terrain_edges[0] = [[1, self.workspace_axis_colors[0], self.workspace_axis_sizes], [2, self.workspace_axis_colors[1], self.workspace_axis_sizes], [3, self.workspace_axis_colors[2], self.workspace_axis_sizes]]
        # create the robotic manipulator points and edges
        self.robotic_manipulator_points = []  # initialize the points of the robotic manipulator
        frames_origins, frames_orientations = self.get_all_frames_positions_orientations(np.zeros(self.joints_number,))  # get the positions and the orientations of the frames of the robotic manipulator
        self.robotic_manipulator_points = [frames_origins[k].tolist() + [1.0] for k in range(self.frames_number)]  # add the homogeneous coordinate to the points of the robotic manipulator
        for k in range(self.joints_number):  # for all the joints of the robotic manipulator
            self.robotic_manipulator_points.append((frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]).tolist() + [1.0])  # add the points (homogeneous) of the joints of the robotic manipulator
        points_num_before_links = len(self.robotic_manipulator_points)  # the number of the points of the robotic manipulator
        robotic_manipulator_links_points = [np.array(frames_origins[0])] + [np.array(frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]) for k in range(self.joints_number)] + [np.array(frames_origins[-1])]  # the points of the links of the robotic manipulator
        for k in range(self.joints_number + 2):  # for all the joints of the robotic manipulator (plus the base and the end-effector frames)
            self.robotic_manipulator_points.append(robotic_manipulator_links_points[k].tolist() + [1.0])  # add the points (homogeneous) of the links of the robotic manipulator
        self.robotic_manipulator_points = np.array(self.robotic_manipulator_points, dtype = float)  # convert the points of the robotic manipulator to a numpy array
        self.robotic_manipulator_points_num = len(self.robotic_manipulator_points)  # update the number of the robotic manipulator points
        self.robotic_manipulator_edges = self.robotic_manipulator_points_num * [None]  # initialize the edges of the robotic manipulator
        for k in range(self.links_number):
            self.robotic_manipulator_edges[points_num_before_links + k] = [[points_num_before_links + k + 1, self.links_colors[k], self.links_sizes[k]]]
        # create the robotic manipulator end-effector frame points and edges
        frame_axis_length = 0.05  # the length of the frame axis
        frame_axis_size = 2  # the size of the frame axis
        self.end_effector_frame_points = []  # initialize the points of the end-effector
        self.end_effector_frame_points.append([0.0, 0.0, 0.0, 1.0])
        self.end_effector_frame_points.append([frame_axis_length, 0.0, 0.0, 1.0])
        self.end_effector_frame_points.append([0.0, frame_axis_length, 0.0, 1.0])
        self.end_effector_frame_points.append([0.0, 0.0, frame_axis_length, 1.0])
        self.end_effector_frame_points = np.array(self.end_effector_frame_points, dtype = float)  # convert the points of the end-effector to a numpy array
        self.end_effector_frame_points_num = len(self.end_effector_frame_points)  # update the number of the end-effector points
        self.end_effector_frame_edges = self.end_effector_frame_points_num * [None]  # initialize the edges of the end-effector
        self.end_effector_frame_edges[0] = [[1, self.workspace_axis_colors[0], frame_axis_size], [2, self.workspace_axis_colors[1], frame_axis_size], [3, self.workspace_axis_colors[2], frame_axis_size]]
        # create the obstacles plane points and edges
        self.obstacles_plane_points = []  # initialize the points of the obstacles
        self.obstacles_plane_points.append([self.obstacles_2d_plane_x_length / 2.0, self.obstacles_2d_plane_y_length / 2.0, 0.0, 1.0])
        self.obstacles_plane_points.append([-self.obstacles_2d_plane_x_length / 2.0, self.obstacles_2d_plane_y_length / 2.0, 0.0, 1.0])
        self.obstacles_plane_points.append([-self.obstacles_2d_plane_x_length / 2.0, -self.obstacles_2d_plane_y_length / 2.0, 0.0, 1.0])
        self.obstacles_plane_points.append([self.obstacles_2d_plane_x_length / 2.0, -self.obstacles_2d_plane_y_length / 2.0, 0.0, 1.0])
        start_pos_on_plane = np.array([self.start_pos_workspace_plane[0], self.start_pos_workspace_plane[1], 0.0, 1.0])  # the start position on the plane
        target_pos_on_plane = np.array([self.target_pos_workspace_plane[0], self.target_pos_workspace_plane[1], 0.0, 1.0])  # the final position on the plane
        self.obstacles_plane_points.append(start_pos_on_plane.tolist())
        self.obstacles_plane_points.append(target_pos_on_plane.tolist())
        if self.obst_avoid_solver_menu_is_enabled and len(self.realws_path_control_law_output) != 0:  # if the control law has generated a path
            for k in range(len(self.realws_path_control_law_output)):  # for all the points of the control law output
                self.obstacles_plane_points.append([self.realws_path_control_law_output[k][0], self.realws_path_control_law_output[k][1], 0.0, 1.0])
        self.obstacles_plane_points = np.array(self.obstacles_plane_points, dtype = float)  # convert the points of the obstacles to a numpy array
        self.obstacles_plane_points_num = len(self.obstacles_plane_points)  # update the number of the obstacles points
        self.obstacles_plane_edges = self.obstacles_plane_points_num * [None]  # initialize the edges of the obstacles
        self.obstacles_plane_edges[0] = [[1, "black", self.workspace_2d_plane_size], [3, "black", self.workspace_2d_plane_size]]
        self.obstacles_plane_edges[2] = [[1, "black", self.workspace_2d_plane_size], [3, "black", self.workspace_2d_plane_size]]
        if self.obst_avoid_solver_menu_is_enabled and len(self.realws_path_control_law_output) != 0:  # if the control law has generated a path
            for k in range(len(self.realws_path_control_law_output)):  # for all the points of the control law output
                self.obstacles_plane_edges[6 + k] = [[7 + k, "red", self.workspace_obstacles_size]]
        # create the obstacles plane frame points and edges
        frame_axis_length = 0.05  # the length of the frame axis
        frame_axis_size = 2  # the size of the frame axis
        self.plane_frame_points = []  # initialize the points of the obstacles
        self.plane_frame_points.append([0.0, 0.0, 0.0, 1.0])
        self.plane_frame_points.append([frame_axis_length, 0.0, 0.0, 1.0])
        self.plane_frame_points.append([0.0, frame_axis_length, 0.0, 1.0])
        self.plane_frame_points.append([0.0, 0.0, frame_axis_length, 1.0])
        self.plane_frame_points = np.array(self.plane_frame_points, dtype = float)  # convert the points of the obstacles to a numpy array
        self.plane_frame_points_num = len(self.plane_frame_points)  # update the number of the obstacles points
        self.plane_frame_edges = self.plane_frame_points_num * [None]  # initialize the edges of the obstacles
        self.plane_frame_edges[0] = [[1, self.workspace_axis_colors[0], frame_axis_size], [2, self.workspace_axis_colors[1], frame_axis_size], [3, self.workspace_axis_colors[2], frame_axis_size]]
        # create the obstacles objects points and edges
        self.obstacles_objects_points = []  # initialize the points of the obstacles
        if self.obstacles_objects_enable_visualization == "shown" and len(self.obstacles_boundaries_for_solver) != 0:
            for boundary in self.obstacles_boundaries_for_solver:  # for all the detected obstacles boundaries
                for point in boundary:  # for all the points of the boundary
                    self.obstacles_objects_points.append([point[0], point[1], 0.0, 1.0])
        else:  # if the obstacles avoidance solver menu is disabled or the obstacles objects are hidden
            self.obstacles_objects_points.append([0.0, 0.0, 0.0, 1.0])
        self.obstacles_objects_points = np.array(self.obstacles_objects_points, dtype = float)  # convert the points of the obstacles to a numpy array
        self.obstacles_objects_points_num = len(self.obstacles_objects_points)  # update the number of the obstacles points
        self.obstacles_objects_edges = self.obstacles_objects_points_num * [None]  # initialize the edges of the obstacles
        if self.obstacles_objects_enable_visualization == "shown" and len(self.obstacles_boundaries_for_solver) != 0:
            boundaries_points_counter = 0  # the counter of the points of the boundaries
            for boundary in self.obstacles_boundaries_for_solver:  # for all the detected obstacles boundaries
                for k in range(len(boundary) - 1):  # for all the points of the boundary
                    self.obstacles_objects_edges[boundaries_points_counter + k] = [[boundaries_points_counter + k + 1, self.workspace_obstacles_color, self.workspace_obstacles_size]]
                boundaries_points_counter += len(boundary)  # update the counter of the points of the boundaries
        # create the camera device points and edges
        camera_device_length = 0.025  # the length of the camera device
        camera_device_size = 3  # the size of the camera device
        self.camera_device_points = []  # initialize the points of the camera
        self.camera_device_points.append([camera_device_length / 2, camera_device_length / 2, 0.0, 1.0])
        self.camera_device_points.append([-camera_device_length / 2, camera_device_length / 2, 0.0, 1.0])
        self.camera_device_points.append([-camera_device_length / 2, -camera_device_length / 2, 0.0, 1.0])
        self.camera_device_points.append([camera_device_length / 2, -camera_device_length / 2, 0.0, 1.0])
        self.camera_device_points = np.array(self.camera_device_points, dtype = float)  # convert the points of the camera to a numpy array
        self.camera_device_points_num = len(self.camera_device_points)  # update the number of the camera points
        self.camera_device_edges = self.camera_device_points_num * [None]  # initialize the edges of the camera
        self.camera_device_edges[0] = [[1, "orange", camera_device_size], [3, "orange", camera_device_size]]
        self.camera_device_edges[2] = [[1, "orange", camera_device_size], [3, "orange", camera_device_size]]
        # create the camera frame points and edges
        camera_frame_axis_length = 0.025  # the length of the frame axis
        camera_frame_axis_size = 2  # the size of the frame axis
        self.camera_frame_points = []  # initialize the points of the camera
        self.camera_frame_points.append([0.0, 0.0, 0.0, 1.0])
        self.camera_frame_points.append([camera_frame_axis_length, 0.0, 0.0, 1.0])
        self.camera_frame_points.append([0.0, camera_frame_axis_length, 0.0, 1.0])
        self.camera_frame_points.append([0.0, 0.0, camera_frame_axis_length, 1.0])
        self.camera_frame_points = np.array(self.camera_frame_points, dtype = float)  # convert the points of the camera to a numpy array
        self.camera_frame_points_num = len(self.camera_frame_points)  # update the number of the camera points
        self.camera_frame_edges = self.camera_frame_points_num * [None]  # initialize the edges of the camera
        self.camera_frame_edges[0] = [[1, self.workspace_axis_colors[0], camera_frame_axis_size], [2, self.workspace_axis_colors[1], camera_frame_axis_size], [3, self.workspace_axis_colors[2], camera_frame_axis_size]]
    def draw_next_workspace_canvas_frame(self):  # draw the next frame of the workspace
        self.canvas_destroy_counter += 1  # increase the counter of the canvas destroy
        if self.canvas_destroy_counter % (5.0 * self.canvas_fps) == 0:
            self.create_workspace_canvas()  # recreate the workspace canvas
            self.canvas_destroy_counter = 0  # reset the counter of the canvas destroy
        self.create_initial_workspace_canvas_points_connections()  # create the points and edges of the workspace (axis and the robotic manipulator)
        self.apply_workspace_transformation()  # apply the transformation defined by the proper transfer, rotation and scale variables to all the points of the workspace
        self.workspace_canvas.delete("all")  # clear the workspace canvas
        # choose the points and edges to draw
        a = self.axis_terrain_points_num; b = self.robotic_manipulator_points_num; c = self.end_effector_frame_points_num; d = self.obstacles_plane_points_num; e = self.plane_frame_points_num; f = self.obstacles_objects_points_num; g = self.camera_device_points_num; h = self.camera_frame_points_num
        self.canvas_moved_points = np.array(self.canvas_moved_points); self.canvas_moved_points = self.canvas_moved_points.tolist()  # convert the canvas moved points to a list
        self.axis_terrain_moved_points = self.canvas_moved_points[0:a]  # the moved points of the axis and terrain
        self.robotic_manipulator_moved_points = self.canvas_moved_points[a : a + b]  # the moved points of the robotic manipulator
        self.end_effector_frame_moved_points = self.canvas_moved_points[a + b : a + b + c]  # the moved points of the end-effector frame
        self.obstacles_plane_moved_points = self.canvas_moved_points[a + b + c: a + b + c + d]  # the moved points of the obstacles plane
        self.obstacles_frame_moved_points = self.canvas_moved_points[a + b + c + d : a + b + c + d + e]  # the moved points of the obstacles frame
        self.obstacles_objects_moved_points = self.canvas_moved_points[a + b + c + d + e : a + b + c + d + e + f]  # the moved points of the obstacles objects
        self.camera_device_moved_points = self.canvas_moved_points[a + b + c + d + e + f : a + b + c + d + e + f + g]  # the moved points of the camera device
        self.camera_frame_moved_points = self.canvas_moved_points[a + b + c + d + e + f + g : a + b + c + d + e + f + g + h]  # the moved points of the camera frame
        # prepare the points of the various objects to draw on the canvas
        # prepare the points of the axis and terrain object
        points_counter = 0  # initialize the count of the points of the workspace
        if self.axis_enable_visualization == "shown":  # if the axis is enabled to be visualized
            for k in range(3):  # iterate through the three axis
                self.axis_terrain_edges[0][k][1] = self.workspace_axis_colors[k]  # change the color of the axis connecting lines
            axis_terrain_moved_points_properties = [[self.axis_terrain_moved_points[k], "black", self.workspace_axis_sizes + 5] for k in range(len(self.axis_terrain_moved_points))]  # the points of the axis and terrain with their properties
            axis_terrain_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.axis_terrain_points_num  # update the count of the points of the workspace
        # prepare the points of the robotic manipulator object
        if self.robotic_manipulator_enable_visualization == "shown":  # if the robotic manipulator is enabled to be visualized
            robotic_manipulator_moved_points_properties = [[self.robotic_manipulator_moved_points[k], self.frames_origins_colors[k], self.frames_origins_sizes[k] + 3] for k in range(self.frames_number)]  # the points of the frames of the robotic manipulator with their properties
            robotic_manipulator_moved_points_properties += [[self.robotic_manipulator_moved_points[self.frames_number + k], self.frames_origins_colors[k + 1], self.frames_origins_sizes[k + 1] + 5] for k in range(self.joints_number)]  # the points of the joints of the robotic manipulator with their properties
            robotic_manipulator_moved_points_properties += [[self.robotic_manipulator_moved_points[self.joints_number + self.frames_number + k], "black", 3] for k in range(self.links_number + 1)]  # the points of the links of the robotic manipulator with their properties
            robotic_manipulator_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.robotic_manipulator_points_num  # update the count of the points of the workspace
        # prepare the points of the robotic manipulator end-effector object
        if self.end_effector_frame_enable_visualization == "shown":  # if the end-effector frame is enabled to be visualized
            end_effector_frame_moved_points_properties = [[self.end_effector_frame_moved_points[k], "black", 5] for k in range(len(self.end_effector_frame_moved_points))]  # the points of the end-effector frame with their properties
            end_effector_frame_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.end_effector_frame_points_num  # update the count of the points of the workspace
        # prepare the points of the obstacles plane object
        if self.obstacles_plane_enable_visualization == "shown":  # if the obstacles plane is enabled to be visualized
            obstacles_plane_moved_points_properties = [[self.obstacles_plane_moved_points[k], "black", self.workspace_2d_plane_size+5] for k in range(4)]  # the points of the obstacles plane with their properties
            obstacles_plane_moved_points_properties.append([self.obstacles_plane_moved_points[4], "magenta", [1, 7][[False, True].index(self.obst_avoid_solver_menu_is_enabled)]])  # the start position of the workspace plane
            obstacles_plane_moved_points_properties.append([self.obstacles_plane_moved_points[5], "green", [1, 7][[False, True].index(self.obst_avoid_solver_menu_is_enabled)]])  # the final position of the workspace plane
            obstacles_plane_moved_points_properties += [[self.obstacles_plane_moved_points[k], "brown", [1, 3][[False, True].index(self.obst_avoid_solver_menu_is_enabled)]] for k in range(6, len(self.obstacles_plane_moved_points))]  # the points of the path of the workspace plane
            obstacles_plane_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.obstacles_plane_points_num  # update the count of the points of the workspace
        # prepare the points of the obstacles frame object
        if self.plane_frame_enable_visualization == "frame":  # if the obstacles frame is enabled to be visualized
            plane_frame_moved_points_properties = [[self.obstacles_frame_moved_points[k], "black", 5] for k in range(len(self.obstacles_frame_moved_points))]  # the points of the obstacles frame with their properties
            plane_frame_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.plane_frame_points_num  # update the count of the points of the workspace
        # prepare the points of the obstacles objects
        if self.obstacles_objects_enable_visualization == "shown":  # if the obstacles objects are enabled to be visualized
            obstacles_objects_moved_points_properties = [[self.obstacles_objects_moved_points[k], self.workspace_obstacles_color, 1] for k in range(len(self.obstacles_objects_moved_points))]  # the points of the obstacles objects with their properties
            obstacles_objects_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.obstacles_objects_points_num  # update the count of the points of the workspace
        # prepare the points of the camera device object
        if self.camera_enable_visualization == "shown":  # if the camera device is enabled to be visualized
            camera_device_moved_points_properties = [[self.camera_device_moved_points[k], "black", 3] for k in range(len(self.camera_device_moved_points))]  # the points of the camera device with their properties
            camera_device_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.camera_device_points_num  # update the count of the points of the workspace
        # prepeare the points of the camera frame
        if self.camera_frame_enable_visualization == "frame":  # if the camera frame is enabled to be visualized
            camera_frame_moved_points_properties = [[self.camera_frame_moved_points[k], "black", 5] for k in range(len(self.camera_frame_moved_points))]  # the points of the camera frame with their properties
            camera_frame_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.camera_frame_points_num  # update the count of the points of the workspace
        # draw the various objects on the workspace canvas
        # draw the terrain and the world axis objects
        if self.terrain_enable_visualization == "shown":  # if the axis and terrain are enabled to be visualized
            first_terrain_point = 5; self.workspace_canvas.create_polygon([self.axis_terrain_moved_points[first_terrain_point][0], self.axis_terrain_moved_points[first_terrain_point][1], self.axis_terrain_moved_points[first_terrain_point+1][0], self.axis_terrain_moved_points[first_terrain_point+1][1], \
                                                                            self.axis_terrain_moved_points[first_terrain_point+2][0], self.axis_terrain_moved_points[first_terrain_point+2][1], self.axis_terrain_moved_points[first_terrain_point+3][0], self.axis_terrain_moved_points[first_terrain_point+3][1]], width = 1, fill = self.workspace_terrain_color, outline = "black")  # draw the terrain plane
        if self.axis_enable_visualization == "shown":  # if the axis is enabled to be visualized
            self.draw_workspace_object(axis_terrain_moved_points_properties, self.axis_terrain_edges, True, True)  # draw the axis and terrain objects
            self.workspace_canvas.create_text(self.axis_terrain_moved_points[0][0]-15, self.axis_terrain_moved_points[0][1], text = "O", font = "Calibri 15 bold", fill = "black")  # draw the letter "O" on the axis origin
            self.workspace_canvas.create_text(self.axis_terrain_moved_points[1][0]-15, self.axis_terrain_moved_points[1][1], text = "x", font = "Calibri 15 bold", fill = self.workspace_axis_colors[0])  # draw the letter "x" on the x axis
            self.workspace_canvas.create_text(self.axis_terrain_moved_points[2][0]+15, self.axis_terrain_moved_points[2][1], text = "y", font = "Calibri 15 bold", fill = self.workspace_axis_colors[1])  # draw the letter "y" on the y axis
            self.workspace_canvas.create_text(self.axis_terrain_moved_points[3][0]+15, self.axis_terrain_moved_points[3][1], text = "z", font = "Calibri 15 bold", fill = self.workspace_axis_colors[2])  # draw the letter "z" on the z axis
        # draw the robotic manipulator and the obstacles in the right order
        if self.drawing_order == "obstacles":  # if the robotic manipulator is drawn first
            if self.robotic_manipulator_enable_visualization == "shown":
                self.draw_workspace_object(robotic_manipulator_moved_points_properties, self.robotic_manipulator_edges, self.robot_joints_enable_visualization == "shown", self.robot_links_enable_visualization == "shown")
            if self.end_effector_frame_enable_visualization == "shown":
                self.draw_workspace_object(end_effector_frame_moved_points_properties, self.end_effector_frame_edges, self.robotic_manipulator_enable_visualization == "shown", self.robotic_manipulator_enable_visualization == "shown")
            if self.obstacles_plane_enable_visualization == "shown":
                first_2d_plane_point = 0; self.workspace_canvas.create_polygon([self.obstacles_plane_moved_points[first_2d_plane_point][0], self.obstacles_plane_moved_points[first_2d_plane_point][1], self.obstacles_plane_moved_points[first_2d_plane_point+1][0], self.obstacles_plane_moved_points[first_2d_plane_point+1][1], \
                                                                                self.obstacles_plane_moved_points[first_2d_plane_point+2][0], self.obstacles_plane_moved_points[first_2d_plane_point+2][1], self.obstacles_plane_moved_points[first_2d_plane_point+3][0], self.obstacles_plane_moved_points[first_2d_plane_point+3][1]], width = 1, fill = self.workspace_2d_plane_color, outline = "black")  # draw the 2D plane of the obstacles
                self.draw_workspace_object(obstacles_plane_moved_points_properties, self.obstacles_plane_edges, True, True)
            if self.plane_frame_enable_visualization == "frame":
                self.draw_workspace_object(plane_frame_moved_points_properties, self.plane_frame_edges, True, True)
            if self.obstacles_objects_enable_visualization == "shown":
                self.draw_workspace_object(obstacles_objects_moved_points_properties, self.obstacles_objects_edges, False, True)
        elif self.drawing_order == "robot":  # if the obstacles are drawn first
            if self.obstacles_plane_enable_visualization == "shown":
                first_2d_plane_point = 0; self.workspace_canvas.create_polygon([self.obstacles_plane_moved_points[first_2d_plane_point][0], self.obstacles_plane_moved_points[first_2d_plane_point][1], self.obstacles_plane_moved_points[first_2d_plane_point+1][0], self.obstacles_plane_moved_points[first_2d_plane_point+1][1], \
                                                                                self.obstacles_plane_moved_points[first_2d_plane_point+2][0], self.obstacles_plane_moved_points[first_2d_plane_point+2][1], self.obstacles_plane_moved_points[first_2d_plane_point+3][0], self.obstacles_plane_moved_points[first_2d_plane_point+3][1]], width = 1, fill = self.workspace_2d_plane_color, outline = "black")  # draw the 2D plane of the obstacles
                self.draw_workspace_object(obstacles_plane_moved_points_properties, self.obstacles_plane_edges, True, True)
            if self.plane_frame_enable_visualization == "frame":
                self.draw_workspace_object(plane_frame_moved_points_properties, self.plane_frame_edges, True, True)
            if self.obstacles_objects_enable_visualization == "shown":
                self.draw_workspace_object(obstacles_objects_moved_points_properties, self.obstacles_objects_edges, False, True)
            if self.robotic_manipulator_enable_visualization == "shown":
                self.draw_workspace_object(robotic_manipulator_moved_points_properties, self.robotic_manipulator_edges, self.robot_joints_enable_visualization == "shown", self.robot_links_enable_visualization == "shown")
            if self.end_effector_frame_enable_visualization == "shown":
                self.draw_workspace_object(end_effector_frame_moved_points_properties, self.end_effector_frame_edges, self.robotic_manipulator_enable_visualization == "shown", self.robotic_manipulator_enable_visualization == "shown")
        # draw the camera device object
        if self.camera_enable_visualization == "shown":  # if the camera device is enabled to be visualized
            self.draw_workspace_object(camera_device_moved_points_properties, self.camera_device_edges, True, True)
        # draw the camera frame object
        if self.camera_frame_enable_visualization == "frame":  # if the camera frame is enabled to be visualized
            self.draw_workspace_object(camera_frame_moved_points_properties, self.camera_frame_edges, True, True)
        # write the names of the current and the pending robotic manipulator models on the workspace canvas
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            self.workspace_canvas.create_text(self.workspace_canvas_width / 2, 20, text = f"Current robotic manipulator:   {self.built_robotic_manipulator_info['name']}", font = "Calibri 12 bold", fill = "black")  # write the name of the built robotic manipulator model on the workspace canvas
            if self.robotic_manipulator_model_name != "" and self.robotic_manipulator_model_name != self.built_robotic_manipulator_info["name"]:
                self.workspace_canvas.create_text(self.workspace_canvas_width / 2, 40, text = f"Pending:   {self.robotic_manipulator_model_name}", font = "Calibri 12 bold", fill = "red")  # write the name of the pending robotic manipulator model on the workspace canvas
        else:
            if self.robotic_manipulator_model_name != "":
                self.workspace_canvas.create_text(self.workspace_canvas_width / 2, 20, text = f"Pending:   {self.robotic_manipulator_model_name}", font = "Calibri 12 bold", fill = "red")  # write the name of the pending robotic manipulator model on the workspace canvas
        # write the values of the visualized variables on the workspace canvas
        self.choose_visualized_variables_button.configure(text = self.control_or_kinematics_variables_visualization)  # update the text of the button that allows the user to choose the visualized variables
        joints_configuration_columns_indicator = 1  # the number of rows of the joints configuration indicator
        joints_configuration = ""  # the configuration of the joints of the robotic manipulator
        visualized_joints_values = [self.control_joints_variables, self.forward_kinematics_variables][[False, True].index(self.control_or_kinematics_variables_visualization == "kinematics")]  # the values of the visualized joints
        for k in range(self.joints_number):
            joints_configuration += f"{k + 1}" + [f"(°): {np.rad2deg(visualized_joints_values[k]):.1f}", f"(m): {visualized_joints_values[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])]  # the joints configuration indicator of the robotic manipulator
            if k <= self.joints_number - 1: joints_configuration += "   "
            if (k + 1) % joints_configuration_columns_indicator == 0 and (k + 1) != self.joints_number: joints_configuration += "\n"
        self.visualized_variables_values_indicator.configure(text = joints_configuration)  # update the text of the indicator that shows the values of the visualized variables
        # bind the points of the workspace to show their coordinates when the user's cursor is pointing to them
        for point in range(len(self.workspace_canvas_points)):
            self.workspace_canvas.tag_unbind(f"point{point}", "<Enter>")
            self.workspace_canvas.tag_bind(f"point{point}", "<Enter>", self.show_point_coordinates_helper(point))
        # loop the visualization function
        self.workspace_canvas.after(int(1000.0 / self.canvas_fps), self.draw_next_workspace_canvas_frame)  # loop the visualization function
    def draw_workspace_object(self, points, edges, points_are_shown, edges_are_shown):  # draw an object in the workspace
        # draw the edges (connecting lines) between every point and its chosen neighbours
        if edges_are_shown:  # if the edges are shown
            for start_point in range(len(edges)):
                if edges[start_point] != None:
                    for [end_point, line_color, line_width] in edges[start_point]:
                        try: self.workspace_canvas.create_line(points[start_point][0][0], points[start_point][0][1], points[end_point][0][0], points[end_point][0][1], width = line_width, fill = line_color, activefill = "white")
                        except: pass
        # draw the points
        counter = points[-1]  # the count of the points of the workspace
        points = points[:-1]
        if points_are_shown:  # if the points are shown
            for k in range(len(points)):
                if points[k][0] != None:
                    try: self.workspace_canvas.create_line(points[k][0][0], points[k][0][1], points[k][0][0], points[k][0][1], width = points[k][2], fill = points[k][1], capstyle = "round", activefill = "white", tags = f"point{counter + k}")
                    except: pass
    def show_point_coordinates_helper(self, point):  # helper function that returns the function that shows the coordinates of the point the user's cursor is pointing to
        return lambda event: self.show_point_coordinates(point, event)
    def show_point_coordinates(self, point, event = None):  # show the coordinates of the point the user's cursor is pointing to
        self.pointing_to_point = f"({self.workspace_canvas_points[point][0]:.3f}, {self.workspace_canvas_points[point][1]:.3f}, {self.workspace_canvas_points[point][2]:.3f})"
        self.workspace_canvas.create_text(self.workspace_canvas_width / 2, self.workspace_canvas_height - 20, text = f"Pointing to (m): {self.pointing_to_point}", font = "Calibri 12 bold", fill = "black")

    # functions for the creation of the menus
    def clear_menus_background(self, event = None):  # clear the menus background
        try: self.menus_background.destroy();   # destroy the previous sub menus of the current main menu
        except: pass
        self.menus_background = tk.Frame(self.menus_area, width = self.menus_background_width, height = self.menus_background_height, bg = self.menus_area_color, bd = self.workspace_menus_bd_width, relief = "groove")  # recreate the menus background
        self.menus_background.grid(row = 1, column = 0, sticky = tk.NSEW)  # place the menus background in the menus area
    def change_main_menu(self, main_menu_num, event = None):  # change the main menu
        self.main_menu_choice = main_menu_num % len(self.main_menus_build_details)  # change the main menu choice
        if self.main_menu_choice != 0:  # if the main menu number is not 0
            self.reload_built_robotic_manipulator_info_data()  # reload the data of the built robotic manipulator model, if any
        if self.main_menu_choice == len(self.main_menus_build_details)-1:  # if the main menu is the last main menu
            self.obst_avoid_solver_menu_is_enabled = True  # set the proper flag to True
        else:
            self.obst_avoid_solver_menu_is_enabled = False  # set the proper flag to False
        new_main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title']  # the title of the new main menu
        self.main_menu_label.configure(text = new_main_menu_title + f" ({self.main_menu_choice+1}/{len(self.main_menus_build_details)})")  # change the main menu title
        self.main_menus_build_details[self.main_menu_choice]['build_function'](self.main_menus_build_details[self.main_menu_choice]['submenus_titles'], self.main_menus_build_details[self.main_menu_choice]['submenus_descriptions'])  # build the sub menus for the new main menu
    def create_static_menu_frame(self, menu_properties, event = None):
        menu = tk.Frame(self.menus_background, width = menu_properties['width'], height = menu_properties['height'], bg = menu_properties['bg_color'], highlightbackground = "red", highlightcolor = "red", highlightthickness = 2)
        menu.grid(row = menu_properties['row'], column = menu_properties['column'], sticky = tk.NS)
        return menu  # return the static menu frame created
    def build_construct_robotic_manipulator_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus for the main menu that constructs the robotic manipulator
        self.clear_menus_background()  # clear the menus background
        # create the define parameters sub menu
        parameters_menu_title = submenus_titles[0]
        parameters_menu_info = f"--- {parameters_menu_title} ---\n" + submenus_descriptions[0]
        parameters_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = parameters_menu_title, info = parameters_menu_info, row = 0, column = 0, width = self.menus_background_width / 2, height = self.menus_background_height, \
                                            rows = 21, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # create the adjust visualization sub menu
        visualization_menu_title = submenus_titles[1]
        visualization_menu_info = f"--- {visualization_menu_title} ---\n" + submenus_descriptions[1]
        visualization_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = visualization_menu_title, info = visualization_menu_info, row = 0, column = 1, width = self.menus_background_width / 2, height = self.menus_background_height, \
                                                rows = 18, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_define_parameters_menu(self.create_static_menu_frame(parameters_menu_properties), parameters_menu_properties)
        self.generate_adjust_visualization_menu(self.create_static_menu_frame(visualization_menu_properties), visualization_menu_properties)
    def generate_define_parameters_menu(self, menu_frame, menu_properties, event = None):  # build the define parameters menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        model_name_label_ord = 2
        joints_number_model_label_ord = 3
        den_har_parameters_label_ord = joints_number_model_label_ord+1
        choose_joint_number_model_label_ord = den_har_parameters_label_ord+1
        choose_joint_type_label_ord = choose_joint_number_model_label_ord+1
        a_parameter_label_ord = choose_joint_type_label_ord+1
        alpha_parameter_label_ord = a_parameter_label_ord+1
        d_parameter_label_ord = alpha_parameter_label_ord+1
        theta_parameter_label_ord = d_parameter_label_ord+1
        var_limits_label_ord = theta_parameter_label_ord+1.4
        var_min_limit_label_ord = var_limits_label_ord-0.4
        var_max_limit_label_ord = var_limits_label_ord+0.4
        base_end_effector_label_ord = var_limits_label_ord+1.4
        base_label_ord = base_end_effector_label_ord+1.4
        base_pos_button_ord = base_label_ord-0.4
        base_orient_button_ord = base_label_ord+0.4
        end_effector_label_ord = base_label_ord+1.6
        end_effector_pos_button_ord = end_effector_label_ord-0.4
        end_effector_orient_button_ord = end_effector_label_ord+0.4
        robot_control_operations_label_ord = end_effector_label_ord+1.4
        build_robot_button_ord = robot_control_operations_label_ord+1.0
        destroy_robot_button_ord = build_robot_button_ord+0.8
        show_model_info_button_ord = build_robot_button_ord+0.4
        save_model_button_ord = build_robot_button_ord+1.8
        delete_model_button_ord = save_model_button_ord+1.0
        load_model_label_ord = save_model_button_ord
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        model_name_label_x = 1/4; gbl.menu_label(menu_frame, "Model name:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], model_name_label_x * menu_properties['width'], model_name_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.model_name_entrybox = ttk.Entry(menu_frame, font = f"Calibri {menu_properties['options_font']}", width = 16, justify = "center")
        model_name_entrybox_x = 2/3; self.model_name_entrybox.place(x = model_name_entrybox_x * menu_properties['width'], y = model_name_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.model_name_entrybox.insert(0, self.robotic_manipulator_model_name)
        self.model_name_entrybox.bind("<Return>", self.change_robotic_manipulator_model_name)
        joints_number_model_label_x = 1/3; gbl.menu_label(menu_frame, "Joints number (n):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_number_model_label_x * menu_properties['width'], joints_number_model_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_number_button_ord = joints_number_model_label_ord; joints_number_button_x = 2/3; self.joints_number_button = gbl.menu_button(menu_frame, self.joints_number, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], joints_number_button_x * menu_properties['width'], joints_number_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_joints_number).button
        den_har_parameters_label_x = 1/2; gbl.menu_label(menu_frame, "Standard Denavit - Hartenberg parameters:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], den_har_parameters_label_x * menu_properties['width'], den_har_parameters_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_joint_number_model_label_x = 1/3; gbl.menu_label(menu_frame, "Joint number:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_number_model_label_x * menu_properties['width'], choose_joint_number_model_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_number_model_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = [f"joint {joint}" for joint in range(1, self.joints_number + 1)], justify = "center")
        choose_joint_number_model_combobox_x = 3/4; self.choose_joint_number_model_combobox.place(x = choose_joint_number_model_combobox_x * menu_properties['width'], y = choose_joint_number_model_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_number_model_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_model)
        choose_joint_type_label_x = 1/3; gbl.menu_label(menu_frame, "Joint type:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_type_label_x * menu_properties['width'], choose_joint_type_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.joints_types_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = self.joints_types_list, justify = "center")
        joints_types_combobox_x = 3/4; self.joints_types_combobox.place(x = joints_types_combobox_x * menu_properties['width'], y = choose_joint_type_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.joints_types_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_type)
        a_den_har_parameter_label_x = 1/3; gbl.menu_label(menu_frame, "a (meters):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], a_den_har_parameter_label_x * menu_properties['width'], a_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        a_den_har_parameter_button_ord = a_parameter_label_ord; a_den_har_parameter_button_x = 3/4; self.a_den_har_parameter_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], a_den_har_parameter_button_x * menu_properties['width'], a_den_har_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_a_den_har_parameter).button
        alpha_den_har_parameter_label_x = 1/3; gbl.menu_label(menu_frame, "alpha (degrees):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], alpha_den_har_parameter_label_x * menu_properties['width'], alpha_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        alpha_den_har_parameter_button_ord = alpha_parameter_label_ord; alpha_den_har_parameter_button_x = 3/4; self.alpha_den_har_parameter_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], alpha_den_har_parameter_button_x * menu_properties['width'], alpha_den_har_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_alpha_den_har_parameter).button
        d_den_har_parameter_label_x = 1/3; gbl.menu_label(menu_frame, "d (meters):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], d_den_har_parameter_label_x * menu_properties['width'], d_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        d_den_har_parameter_button_ord = d_parameter_label_ord; d_den_har_parameter_button_x = 3/4; self.d_den_har_parameter_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], d_den_har_parameter_button_x * menu_properties['width'], d_den_har_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_d_den_har_parameter).button
        theta_den_har_parameter_label_x = 1/3; gbl.menu_label(menu_frame, "theta (degrees):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], theta_den_har_parameter_label_x * menu_properties['width'], theta_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        theta_den_har_parameter_button_ord = theta_parameter_label_ord; theta_den_har_parameter_button_x = 3/4; self.theta_den_har_parameter_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], theta_den_har_parameter_button_x * menu_properties['width'], theta_den_har_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_theta_den_har_parameter).button
        var_limits_label_x = 1/3; self.var_limits_label = gbl.menu_label(menu_frame, "variable limits:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], var_limits_label_x * menu_properties['width'], var_limits_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        var_min_limit_label_x = 3/5; gbl.menu_label(menu_frame, "min:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], var_min_limit_label_x * menu_properties['width'], var_min_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        var_min_limit_button_x = 4/5; self.var_min_limit_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], var_min_limit_button_x * menu_properties['width'], var_min_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_joint_variable_min_limit).button
        var_max_limit_label_x = 3/5; gbl.menu_label(menu_frame, "max:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], var_max_limit_label_x * menu_properties['width'], var_max_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        var_max_limit_button_x = 4/5; self.var_max_limit_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], var_max_limit_button_x * menu_properties['width'], var_max_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_joint_variable_max_limit).button
        base_end_effector_label_x = 1/2; gbl.menu_label(menu_frame, "Base and end-effector systems:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], base_end_effector_label_x * menu_properties['width'], base_end_effector_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        base_label_x = 1/3; gbl.menu_label(menu_frame, "World ⭢ Base ⭢\nFrame \"0\"", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], base_label_x * menu_properties['width'], base_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        base_pos_button_x = 2/3; self.base_pos_button = gbl.menu_button(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], base_pos_button_x * menu_properties['width'], base_pos_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_base_zero_frame_positions).button
        base_orient_button_x = 2/3; self.base_orient_button = gbl.menu_button(menu_frame, "orientation", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], base_orient_button_x * menu_properties['width'], base_orient_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_base_zero_frame_orientations).button
        end_effector_label_x = 1/3; gbl.menu_label(menu_frame, "Frame \"n\" ⭢\nEnd-effector:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_label_x * menu_properties['width'], end_effector_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_pos_button_x = 2/3; self.end_effector_pos_button = gbl.menu_button(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], end_effector_pos_button_x * menu_properties['width'], end_effector_pos_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_end_effector_position).button
        end_effector_orient_button_x = 2/3; self.end_effector_orient_button = gbl.menu_button(menu_frame, "orientation", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], end_effector_orient_button_x * menu_properties['width'], end_effector_orient_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_end_effector_orientation).button
        robot_control_operations_label_x = 1/2; gbl.menu_label(menu_frame, "Robot control operations:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], robot_control_operations_label_x * menu_properties['width'], robot_control_operations_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_model_info_button_x = 4/16; self.show_model_info_button = gbl.menu_button(menu_frame, "show current\nrobot info", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_model_info_button_x * menu_properties['width'], show_model_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_robotic_manipulator_model_info).button
        save_model_button_ord_x = 4/16; self.save_robotic_manipulator_model_file_button = gbl.menu_button(menu_frame, "save robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], save_model_button_ord_x * menu_properties['width'], save_model_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.save_robotic_manipulator_model_file).button
        delete_model_button_x = 4/16; self.delete_robotic_amr_model_button = gbl.menu_button(menu_frame, "delete robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], delete_model_button_x * menu_properties['width'], delete_model_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.delete_robotic_manipulator_model_file).button
        build_robot_button_x = 11/16; self.build_robot_button = gbl.menu_button(menu_frame, "build robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], build_robot_button_x * menu_properties['width'], build_robot_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.build_robotic_manipulator_model).button
        destroy_robot_button_x = 11/16; self.destroy_robot_button = gbl.menu_button(menu_frame, "destroy robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], destroy_robot_button_x * menu_properties['width'], destroy_robot_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.destroy_robotic_manipulator_model).button
        load_model_label_x = 11/16; gbl.menu_label(menu_frame, "Load robot model:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], load_model_label_x * menu_properties['width'], load_model_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.load_model_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 15, values = self.saved_robots_models_files_list, justify = "center")
        load_model_combobox_x = load_model_label_x; self.load_model_combobox.place(x = load_model_combobox_x * menu_properties['width'], y = (load_model_label_ord + 0.8) * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.load_model_combobox.bind("<<ComboboxSelected>>", lambda event: self.load_robotic_manipulator_model("", event))
        self.load_model_combobox.bind("<Return>", lambda event: self.load_robotic_manipulator_model("", event))
    def generate_adjust_visualization_menu(self, menu_frame, menu_properties, event = None):  # build the adjust visualization menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        robotic_manipulator_visualization_label_ord = 2
        choose_frame_visualization_label_ord = robotic_manipulator_visualization_label_ord+1
        choose_frame_color_label_ord = choose_frame_visualization_label_ord+1
        change_frame_color_button_ord = choose_frame_color_label_ord
        apply_color_all_frames_button_ord = choose_frame_color_label_ord
        choose_frame_size_label_ord = choose_frame_color_label_ord+1
        change_frame_size_button_ord = choose_frame_size_label_ord
        apply_size_all_frames_button_ord = choose_frame_size_label_ord
        choose_joint_position_label_ord = apply_size_all_frames_button_ord+1
        change_joint_position_button_ord = choose_joint_position_label_ord
        choose_link_number_visualization_label_ord = change_joint_position_button_ord+1
        choose_link_color_label_ord = choose_link_number_visualization_label_ord+1
        change_link_color_button_ord = choose_link_color_label_ord
        apply_color_all_links_button_ord = choose_link_color_label_ord
        choose_link_size_label_ord = choose_link_color_label_ord+1
        change_link_size_button_ord = choose_link_size_label_ord
        apply_size_all_links_button_ord = choose_link_size_label_ord
        link_length_label_ord = apply_size_all_links_button_ord+1
        link_length_indicator_ord = link_length_label_ord
        workspace_visualization_label_ord = link_length_indicator_ord+1
        canvas_visual_label_ord = workspace_visualization_label_ord+1
        canvas_color_button_ord = canvas_visual_label_ord
        terrain_visual_label_ord = canvas_visual_label_ord+1
        terrain_color_button_ord = terrain_visual_label_ord
        axis_visual_label_ord = terrain_visual_label_ord+1
        axis_color_button_ord = axis_visual_label_ord
        axis_size_button_ord = axis_visual_label_ord
        obstacles_2d_plane_visual_label_ord = axis_visual_label_ord+1
        obstacles_2d_plane_color_button_ord = obstacles_2d_plane_visual_label_ord
        obstacles_2d_plane_size_button_ord = obstacles_2d_plane_visual_label_ord
        obstacles_objects_visual_label_ord = obstacles_2d_plane_visual_label_ord+1
        obstacles_objects_color_button_ord = obstacles_objects_visual_label_ord
        obstacles_objects_size_button_ord = obstacles_objects_visual_label_ord
        other_simulators_label_ord = obstacles_objects_visual_label_ord+1
        matplotlib_simulator_button_ord = other_simulators_label_ord+1
        swift_simulator_button_ord = matplotlib_simulator_button_ord
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        robotic_manipulator_visualization_label_x = 1/2; gbl.menu_label(menu_frame, "Robotic manipulator visualization:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], robotic_manipulator_visualization_label_x * menu_properties['width'], robotic_manipulator_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_frame_visualization_label_x = 1/3; gbl.menu_label(menu_frame, "Frame:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_frame_visualization_label_x * menu_properties['width'], choose_frame_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_frame_visualization_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 12, values = ["base"] + [f"frame {frame}" for frame in range(self.links_number)] + ["end-effector"], justify = "center")
        choose_frame_visualization_combobox_x = 2/3; self.choose_frame_visualization_combobox.place(x = choose_frame_visualization_combobox_x * menu_properties['width'], y = choose_frame_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_frame_visualization_combobox.bind("<<ComboboxSelected>>", self.change_chosen_frame_visualization)
        choose_joint_position_label_x = 1/3; gbl.menu_label(menu_frame, "Joint position (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_position_label_x * menu_properties['width'], choose_joint_position_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_joint_position_button_x = 2/3; self.change_joint_position_button = gbl.menu_button(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_joint_position_button_x * menu_properties['width'], change_joint_position_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_joint_position).button
        choose_frame_color_label_x = 1/4; gbl.menu_label(menu_frame, "Frame color:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_frame_color_label_x * menu_properties['width'], choose_frame_color_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_frame_color_button_x = 2/4; self.change_frame_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_frame_color_button_x * menu_properties['width'], change_frame_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_frame_color).button
        apply_color_all_frames_button_x = 3/4; self.apply_color_all_frames_button = gbl.menu_button(menu_frame, "all frames", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_color_all_frames_button_x * menu_properties['width'], apply_color_all_frames_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_color_to_all_frames).button
        choose_frame_size_label_x = 1/4; gbl.menu_label(menu_frame, "Frame size:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_frame_size_label_x * menu_properties['width'], choose_frame_size_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_frame_size_button_x = 2/4; self.change_frame_size_button = gbl.menu_button(menu_frame, "size", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_frame_size_button_x * menu_properties['width'], change_frame_size_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_frame_size).button
        apply_size_all_frames_button_x = 3/4; self.apply_size_all_frames_button = gbl.menu_button(menu_frame, "all frames", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_size_all_frames_button_x * menu_properties['width'], apply_size_all_frames_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_size_to_all_frames).button
        choose_link_number_visualization_label_x = 1/3; gbl.menu_label(menu_frame, "Link:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_link_number_visualization_label_x * menu_properties['width'], choose_link_number_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_link_visualization_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 12, values = ["base"] + [f"link {link}" for link in range(1, self.links_number)], justify = "center")
        choose_link_visualization_combobox_x = 2/3; self.choose_link_visualization_combobox.place(x = choose_link_visualization_combobox_x * menu_properties['width'], y = choose_link_number_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_link_visualization_combobox.bind("<<ComboboxSelected>>", self.change_chosen_link_visualization)
        link_length_label_x = 1/3; gbl.menu_label(menu_frame, "Link length (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], link_length_label_x * menu_properties['width'], link_length_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        link_length_indicator_x = 2/3; self.link_length_indicator = gbl.menu_label(menu_frame, "length", f"Calibri {menu_properties['options_font']} bold", "yellow", menu_properties['bg_color'], link_length_indicator_x * menu_properties['width'], link_length_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        choose_link_color_label_x = 1/4; gbl.menu_label(menu_frame, "Link color:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_link_color_label_x * menu_properties['width'], choose_link_color_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_link_color_button_x = 2/4; self.change_link_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_link_color_button_x * menu_properties['width'], change_link_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_link_color).button
        apply_color_all_links_button_x = 3/4; self.apply_color_all_links_button = gbl.menu_button(menu_frame, "all links", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_color_all_links_button_x * menu_properties['width'], apply_color_all_links_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_color_to_all_links).button
        choose_link_size_label_x = 1/4; gbl.menu_label(menu_frame, "Link size:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_link_size_label_x * menu_properties['width'], choose_link_size_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_link_size_button_x = 2/4; self.change_link_size_button = gbl.menu_button(menu_frame, "size", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_link_size_button_x * menu_properties['width'], change_link_size_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_link_size).button
        apply_size_all_links_button_x = 3/4; self.apply_size_all_links_button = gbl.menu_button(menu_frame, "all links", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_size_all_links_button_x * menu_properties['width'], apply_size_all_links_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_size_to_all_links).button
        workspace_visualization_label_x = 1/2; gbl.menu_label(menu_frame, "Workspace visualization:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], workspace_visualization_label_x * menu_properties['width'], workspace_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        canvas_visual_label_x = 1/3; gbl.menu_label(menu_frame, "Canvas:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], canvas_visual_label_x * menu_properties['width'], canvas_visual_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        canvas_color_button_x = 2/3; self.canvas_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], canvas_color_button_x * menu_properties['width'], canvas_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_canvas_color).button
        terrain_visual_label_x = 1/3; gbl.menu_label(menu_frame, "Terrain:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], terrain_visual_label_x * menu_properties['width'], terrain_visual_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        terrain_color_button_x = 2/3; self.terrain_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], terrain_color_button_x * menu_properties['width'], terrain_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_terrain_color).button
        axis_visual_label_x = 1/4; gbl.menu_label(menu_frame, "Axis:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], axis_visual_label_x * menu_properties['width'], axis_visual_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        axis_color_button_x = 2/4; self.axis_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], axis_color_button_x * menu_properties['width'], axis_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_axis_colors).button
        axis_size_button_x = 3/4; self.axis_size_button = gbl.menu_button(menu_frame, "size", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], axis_size_button_x * menu_properties['width'], axis_size_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_axis_size).button
        obstacles_2d_plane_label_x = 1/4; gbl.menu_label(menu_frame, "2D plane:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], obstacles_2d_plane_label_x * menu_properties['width'], obstacles_2d_plane_visual_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        obstacles_2d_plane_color_button_x = 2/4; self.obstacles_2d_plane_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], obstacles_2d_plane_color_button_x * menu_properties['width'], obstacles_2d_plane_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_obstacles_2d_plane_color).button
        obstacles_2d_plane_size_button_x = 3/4; self.obstacles_2d_plane_size_button = gbl.menu_button(menu_frame, "size", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], obstacles_2d_plane_size_button_x * menu_properties['width'], obstacles_2d_plane_size_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_obstacles_2d_plane_size).button
        obstacles_objects_label_x = 1/4; gbl.menu_label(menu_frame, "Obstacles:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], obstacles_objects_label_x * menu_properties['width'], obstacles_objects_visual_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        obstacles_objects_color_button_x = 2/4; self.obstacles_objects_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], obstacles_objects_color_button_x * menu_properties['width'], obstacles_objects_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_obstacles_objects_color).button
        obstacles_objects_size_button_x = 3/4; self.obstacles_objects_size_button = gbl.menu_button(menu_frame, "size", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], obstacles_objects_size_button_x * menu_properties['width'], obstacles_objects_size_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_obstacles_objects_size).button
        other_simulators_label_x = 1/2; gbl.menu_label(menu_frame, "Simulators:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], other_simulators_label_x * menu_properties['width'], other_simulators_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        matplotlib_simulator_button_x = 1/3; self.matplotlib_simulator_button = gbl.menu_button(menu_frame, "Matplotlib", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], matplotlib_simulator_button_x * menu_properties['width'], matplotlib_simulator_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.open_matplotlib_simulator).button
        swift_simulator_button_x = 2/3; self.swift_simulator_button = gbl.menu_button(menu_frame, "Swift", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], swift_simulator_button_x * menu_properties['width'], swift_simulator_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.open_close_swift_simulator).button
        # create an option for the item appearing as the end-effector of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def build_robotic_manipulator_forward_kinematics_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus of the main menu that analyzes the forward kinematics (and inverse kinematics) of the robotic manipulator
        self.clear_menus_background()  # clear the menus background
        # create the forward kinematics sub menu
        fkine_menu_title = submenus_titles[0]
        fkine_menu_info = f"--- {fkine_menu_title} ---\n" + submenus_descriptions[0]
        fkine_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = fkine_menu_title, info = fkine_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 11 / 20, 
                                        rows = 12, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # create the inverse kinematics sub menu
        invkine_menu_title = submenus_titles[1]
        invkine_menu_info = f"--- {invkine_menu_title} ---\n" + submenus_descriptions[1]
        invkine_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = invkine_menu_title, info = invkine_menu_info, row = 2, column = 0, width = self.menus_background_width, height = self.menus_background_height * 9 / 20, \
                                        rows = 9, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_forward_kinematics_menu(self.create_static_menu_frame(fkine_menu_properties), fkine_menu_properties)
        self.generate_inverse_kinematics_menu(self.create_static_menu_frame(invkine_menu_properties), invkine_menu_properties)
    def generate_forward_kinematics_menu(self, menu_frame, menu_properties, event = None):  # build the forward kinematics menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        choose_joint_number_fkine_label_ord = 2.5
        choose_joint_number_fkine_combobox_ord = choose_joint_number_fkine_label_ord
        choose_fkine_variable_value_label_ord = choose_joint_number_fkine_label_ord-0.4
        fkine_value_unit_indicator_ord = choose_joint_number_fkine_label_ord+0.4
        joint_fkine_value_combobox_ord = choose_joint_number_fkine_label_ord
        get_control_values_button_ord = choose_joint_number_fkine_label_ord
        choose_frame_label_ord = choose_joint_number_fkine_label_ord+6.5
        choose_frame_combobox_ord = choose_frame_label_ord+0.8
        frame_position_label_ord = choose_frame_label_ord
        frame_position_indicator_ord = frame_position_label_ord+0.8
        frame_orientation_label_ord = choose_frame_label_ord
        frame_orientation_representation_combobox_ord = frame_orientation_label_ord+0.8
        frame_orientation_indicator_ord = choose_frame_label_ord+0.5
        find_reachable_workspace_label_ord = choose_frame_label_ord+2.5
        joints_range_divisions_label_ord = find_reachable_workspace_label_ord
        joints_range_divisions_button_ord = find_reachable_workspace_label_ord
        compute_reachable_workspace_button_ord = find_reachable_workspace_label_ord
        show_fkine_info_button_ord = menu_properties['rows']-1.0
        fkine_variables_sliders_rows = 3; fkine_variables_sliders_ord_list = []
        for k in range(self.joints_number):
            fkine_variables_sliders_ord_list.append(choose_joint_number_fkine_label_ord+1.7 + 1.5*(k % fkine_variables_sliders_rows))
            # fkine_variables_sliders_ord_list.append(choose_joint_number_fkine_label_ord+2.5)
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        choose_joint_number_fkine_label_x = 1/6; gbl.menu_label(menu_frame, "Joint\nnumber:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_number_fkine_label_x * menu_properties['width'], choose_joint_number_fkine_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_number_fkine_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = [f"joint {joint}" for joint in range(1, self.joints_number + 1)], justify = "center")
        choose_joint_number_fkine_combobox_x = 2/6; self.choose_joint_number_fkine_combobox.place(x = choose_joint_number_fkine_combobox_x * menu_properties['width'], y = choose_joint_number_fkine_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_number_fkine_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_fkine)
        choose_fkine_variable_value_label_x = 3/6; gbl.menu_label(menu_frame, "Joint value:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_fkine_variable_value_label_x * menu_properties['width'], choose_fkine_variable_value_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        fkine_value_unit_indicator_x = 3/6; self.fkine_value_unit_indicator = gbl.menu_label(menu_frame, self.joints_types[self.chosen_joint_number_fkine - 1] + " " + ["(°)", "(m)"][self.joints_types_list.index(self.joints_types[self.chosen_joint_number_fkine - 1])], f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], fkine_value_unit_indicator_x * menu_properties['width'], fkine_value_unit_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        self.joint_fkine_value_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 8, values = "", justify = "center")
        joint_fkine_value_combobox_x = 4/6; self.joint_fkine_value_combobox.place(x = joint_fkine_value_combobox_x * menu_properties['width'], y = joint_fkine_value_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.joint_fkine_value_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_fkine_2)
        self.joint_fkine_value_combobox.bind("<Return>", self.change_chosen_fkine_variable)
        get_control_values_button_x = 5/6; self.get_control_values_button = gbl.menu_button(menu_frame, "get control\nvalues", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], get_control_values_button_x * menu_properties['width'], get_control_values_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.copy_control_to_fkine_values).button
        choose_frame_label_x = 1/7; gbl.menu_label(menu_frame, "Frame:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_frame_label_x * menu_properties['width'], choose_frame_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_frame_fkine_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 12, values = ["base"] + [f"frame {frame}" for frame in range(self.links_number)] + ["end-effector"], justify = "center")
        choose_frame_combobox_x = 1/7; self.choose_frame_fkine_combobox.place(x = choose_frame_combobox_x * menu_properties['width'], y = choose_frame_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_frame_fkine_combobox.bind("<<ComboboxSelected>>", self.change_chosen_frame_fkine)
        frame_position_label_x = 2/6; gbl.menu_label(menu_frame, "Position (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], frame_position_label_x * menu_properties['width'], frame_position_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        frame_position_indicator_x = 2/6; self.frame_position_indicator = gbl.menu_label(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], frame_position_indicator_x * menu_properties['width'], frame_position_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        frame_orientation_label_x = 9/16; gbl.menu_label(menu_frame, "Orientation:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], frame_orientation_label_x * menu_properties['width'], frame_orientation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.frame_orientation_representation_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 12, values = self.orientation_representations_fkine_list, justify = "center")
        frame_orientation_representation_combobox_x = 9/16; self.frame_orientation_representation_combobox.place(x = frame_orientation_representation_combobox_x * menu_properties['width'], y = frame_orientation_representation_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.frame_orientation_representation_combobox.bind("<<ComboboxSelected>>", self.change_orientation_representation_fkine)
        frame_orientation_indicator_x = 31/40; self.frame_orientation_indicator = gbl.menu_label(menu_frame, "orientation", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], frame_orientation_indicator_x * menu_properties['width'], frame_orientation_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        find_reachable_workspace_label_x = 1/5; gbl.menu_label(menu_frame, "Find reachable workspace:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], find_reachable_workspace_label_x * menu_properties['width'], find_reachable_workspace_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_range_divisions_label_x = 4/9; gbl.menu_label(menu_frame, "Joints range\ndivisions:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_range_divisions_label_x * menu_properties['width'], joints_range_divisions_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_range_divisions_button_x = 5/9; self.joints_range_divisions_button = gbl.menu_button(menu_frame, "set\ndivs", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], joints_range_divisions_button_x * menu_properties['width'], joints_range_divisions_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.set_joints_range_divisions).button
        compute_reachable_workspace_button_x = 3/4; self.compute_reachable_workspace_button = gbl.menu_button(menu_frame, "compute and plot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], compute_reachable_workspace_button_x * menu_properties['width'], compute_reachable_workspace_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.compute_plot_reachable_workspace).button
        # show_fkine_info_button_x = 19/20; self.show_fkine_info_button = gbl.menu_button(menu_frame, "🕮", f"Calibri {menu_properties['options_font']+5} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_fkine_info_button_x * menu_properties['width'], show_fkine_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_fkine_info).button
        self.fkine_variables_sliders = []
        for k in range(self.joints_number):
            joint_type_index = self.joints_types_list.index(self.joints_types[k])  # the index of the current joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
            fkine_variables_limits = self.control_joints_variables_limits[k][joint_type_index]  # the control variable limits of the current joint
            self.fkine_variables_sliders.append(tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font'] - 3}", orient = tk.HORIZONTAL, from_ = fkine_variables_limits[0], to = fkine_variables_limits[1], resolution = [10**(-self.angles_precision), 10**(-self.distances_precision)][joint_type_index], length = menu_properties['width'] / 7.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = lambda event, slider_num = k: self.change_fkine_variable_slider(slider_num, event)))
            # fkine_variables_sliders_x = 1/10 * (k + 1); self.fkine_variables_sliders.append(tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font'] - 3}", orient = tk.VERTICAL, length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = lambda event, slider_num = k: self.change_fkine_variable_slider(slider_num, event)))
            fkine_variables_sliders_x = 1/(np.ceil(self.joints_number / fkine_variables_sliders_rows) + 1) * (k // fkine_variables_sliders_rows + 1); self.fkine_variables_sliders[k].place(x = fkine_variables_sliders_x * menu_properties['width'], y = fkine_variables_sliders_ord_list[k] * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
            fkine_variables_labels_x = 1/(np.ceil(self.joints_number / fkine_variables_sliders_rows) + 1) * (k // fkine_variables_sliders_rows + 1) - 2/3*1/7; gbl.menu_label(menu_frame, f"{k + 1}\n{['(°)', '(m)'][joint_type_index]}:", f"Calibri {menu_properties['options_font']-3} bold", menu_properties['labels_color'], menu_properties['bg_color'], fkine_variables_labels_x * menu_properties['width'], fkine_variables_sliders_ord_list[k] * menu_properties['height'] / (menu_properties['rows'] + 1) - 0.8)
        self.update_forward_kinematics_indicators()  # update the indicators of the forward kinematics of the robotic manipulator
    def generate_inverse_kinematics_menu(self, menu_frame, menu_properties, event = None):  # build the inverse kinematics menus
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        get_fkine_result_button_ord = 2.5
        send_invkine_result_button_ord = get_fkine_result_button_ord
        choose_end_effector_position_label = get_fkine_result_button_ord+1.8
        choose_end_effector_position_button = choose_end_effector_position_label
        choose_end_effector_orientation_label = choose_end_effector_position_label
        choose_end_effector_orientation_button = choose_end_effector_position_label
        choose_invkine_tolerance_label_ord = choose_end_effector_position_label+1.8
        choose_invkine_tolerance_button_ord = choose_invkine_tolerance_label_ord
        joints_configuration_label_ord = choose_end_effector_position_label+3.5
        joints_configuration_indicator_ord = joints_configuration_label_ord
        show_invkine_info_button_ord = menu_properties['rows']-1.0
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        get_fkine_result_button_x = 1/3; self.get_fkine_result_button = gbl.menu_button(menu_frame, "↓ get forward kinematics\nanalysis result", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], get_fkine_result_button_x * menu_properties['width'], get_fkine_result_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.get_fkine_pose_result).button
        send_invkine_result_button_x = 2/3; self.send_invkine_result_button = gbl.menu_button(menu_frame, "↑ send inverse kinematics\nanalysis result", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_invkine_result_button_x * menu_properties['width'], send_invkine_result_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_invkine_joints_config_result).button
        choose_end_effector_position_label_x = 1/5; gbl.menu_label(menu_frame, "End-effector\nposition (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_end_effector_position_label_x * menu_properties['width'], choose_end_effector_position_label * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_end_effector_position_button_x = 2/5; self.choose_end_effector_position_button = gbl.menu_button(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_end_effector_position_button_x * menu_properties['width'], choose_end_effector_position_button * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_end_effector_position_invkine).button
        choose_end_effector_orientation_label_x = 3/5; gbl.menu_label(menu_frame, "End-effector\norientation (°):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_end_effector_orientation_label_x * menu_properties['width'], choose_end_effector_orientation_label * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_end_effector_orientation_button_x = 4/5; self.choose_end_effector_orientation_button = gbl.menu_button(menu_frame, "orientation", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_end_effector_orientation_button_x * menu_properties['width'], choose_end_effector_orientation_button * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_end_effector_orientation_invkine).button
        choose_invkine_tolerance_label_x = 2/5; gbl.menu_label(menu_frame, "Numerical solver maximum allowed error (tolerance):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_invkine_tolerance_label_x * menu_properties['width'], choose_invkine_tolerance_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_invkine_tolerance_button_x = 3/4; self.choose_invkine_tolerance_button = gbl.menu_button(menu_frame, self.invkine_tolerance, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_invkine_tolerance_button_x * menu_properties['width'], choose_invkine_tolerance_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_invkine_tolerance).button
        joints_configuration_label_x = 1/6; gbl.menu_label(menu_frame, "Joints\nconfiguration:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_configuration_label_x * menu_properties['width'], joints_configuration_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_configuration_indicator_x = 3/5; self.joints_configuration_indicator = gbl.menu_label(menu_frame, "joints configuration", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], joints_configuration_indicator_x * menu_properties['width'], joints_configuration_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        # show_invkine_info_button_x = 19/20; self.show_invkine_info_button = gbl.menu_button(menu_frame, "🕮", f"Calibri {menu_properties['options_font']+5} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_invkine_info_button_x * menu_properties['width'], show_invkine_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_invkine_info).button
        self.update_inverse_kinematics_indicators()  # update the indicators of the inverse kinematics of the robotic manipulator
    def build_robotic_manipulator_differential_kinematics_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus of the main menu that analyzes the differential kinematics of the robotic manipulator
        self.clear_menus_background()  # clear the menus background
        # create the differential kinematics sub menu
        diffkine_menu_title = submenus_titles[0]
        diffkine_menu_info = f"--- {diffkine_menu_title} ---\n" + submenus_descriptions[0]
        diffkine_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = diffkine_menu_title, info = diffkine_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 2, \
                                        rows = 10, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # create the inverse differential kinematics sub menu
        invdiffkine_menu_title = submenus_titles[1]
        invdiffkine_menu_info = f"--- {invdiffkine_menu_title} ---\n" + submenus_descriptions[1]
        invdiffkine_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = invdiffkine_menu_title, info = invdiffkine_menu_info, row = 2, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 2, \
                                            rows = 9, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_differential_kinematics_menu(self.create_static_menu_frame(diffkine_menu_properties), diffkine_menu_properties)
        self.generate_inverse_differential_kinematics_menu(self.create_static_menu_frame(invdiffkine_menu_properties), invdiffkine_menu_properties)
    def generate_differential_kinematics_menu(self, menu_frame, menu_properties, event = None):  # build the differential kinematics menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        choose_joint_number_diffkine_label_ord = 2.5
        joints_configuration_diffkine_label_ord = choose_joint_number_diffkine_label_ord-0.4
        joints_configuration_diffkine_indicator_ord = joints_configuration_diffkine_label_ord+0.8
        choose_joint_number_diffkine_combobox_ord = choose_joint_number_diffkine_label_ord
        choose_diffkine_velocity_label_ord = choose_joint_number_diffkine_label_ord-0.3
        diffkine_value_unit_indicator_ord = choose_joint_number_diffkine_label_ord+0.3
        joint_diffkine_velocity_combobox_ord = choose_joint_number_diffkine_label_ord
        end_effector_linear_vel_label_ord = choose_joint_number_diffkine_label_ord+6.5
        end_effector_linear_vel_indicator_ord = end_effector_linear_vel_label_ord+0.8
        end_effector_angular_vel_label_ord = end_effector_linear_vel_label_ord
        end_effector_angular_vel_indicator_ord = end_effector_angular_vel_label_ord+0.8
        diffkine_wrt_frame_label_ord = end_effector_linear_vel_label_ord
        diffkine_wrt_frame_button_ord = end_effector_linear_vel_label_ord+0.8
        show_diffkine_info_button_ord = menu_properties['rows']-1.0
        difffkine_variables_sliders_rows = 3; diffkine_variables_sliders_ord_list = []
        for k in range(self.joints_number):
            diffkine_variables_sliders_ord_list.append(choose_joint_number_diffkine_label_ord+1.5 + 1.5*(k % difffkine_variables_sliders_rows))
            # fkine_variables_sliders_ord_list.append(choose_joint_number_fkine_label_ord+2.5)
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        joints_configuration_diffkine_label_x = 6/7; gbl.menu_label(menu_frame, "Configuration:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_configuration_diffkine_label_x * menu_properties['width'], joints_configuration_diffkine_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_configuration_diffkine_indicator_x = joints_configuration_diffkine_label_x; self.joints_configuration_diffkine_indicator = gbl.menu_label(menu_frame, self.control_or_kinematics_variables_visualization, f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], joints_configuration_diffkine_indicator_x * menu_properties['width'], joints_configuration_diffkine_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        choose_joint_number_diffkine_label_x = 1/6; gbl.menu_label(menu_frame, "Joint\nnumber:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_number_diffkine_label_x * menu_properties['width'], choose_joint_number_diffkine_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_number_diffkine_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = [f"joint {joint}" for joint in range(1, self.joints_number + 1)], justify = "center")
        choose_joint_number_diffkine_combobox_x = 2/6; self.choose_joint_number_diffkine_combobox.place(x = choose_joint_number_diffkine_combobox_x * menu_properties['width'], y = choose_joint_number_diffkine_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_number_diffkine_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_diffkine)
        choose_diffkine_velocity_label_x = 3/6; gbl.menu_label(menu_frame, "Joint velocity:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_diffkine_velocity_label_x * menu_properties['width'], choose_diffkine_velocity_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        diffkine_value_unit_indicator_x = 3/6; self.diffkine_value_unit_indicator = gbl.menu_label(menu_frame, ["(rad/s)", "(m/s)"][self.joints_types_list.index(self.joints_types[self.chosen_joint_number_diffkine - 1])], f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], diffkine_value_unit_indicator_x * menu_properties['width'], diffkine_value_unit_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        self.joint_diffkine_velocity_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 8, values = "", justify = "center")
        joint_diffkine_velocity_combobox_x = 4/6; self.joint_diffkine_velocity_combobox.place(x = joint_diffkine_velocity_combobox_x * menu_properties['width'], y = joint_diffkine_velocity_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.joint_diffkine_velocity_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_diffkine_2)
        self.joint_diffkine_velocity_combobox.bind("<Return>", self.change_chosen_diffkine_velocity)
        end_effector_linear_vel_label_x = 1/5; gbl.menu_label(menu_frame, "End-effector linear velocity (m/s):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_linear_vel_label_x * menu_properties['width'], end_effector_linear_vel_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_linear_vel_indicator_x = end_effector_linear_vel_label_x; self.end_effector_linear_vel_indicator = gbl.menu_label(menu_frame, "linear velocity", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], end_effector_linear_vel_indicator_x * menu_properties['width'], end_effector_linear_vel_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        end_effector_angular_vel_label_x = 11/20; gbl.menu_label(menu_frame, "End-effector angular velocity (°/s):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_angular_vel_label_x * menu_properties['width'], end_effector_angular_vel_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_angular_vel_indicator_x = end_effector_angular_vel_label_x; self.end_effector_angular_vel_indicator = gbl.menu_label(menu_frame, "angular velocity", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], end_effector_angular_vel_indicator_x * menu_properties['width'], end_effector_angular_vel_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        diffkine_wrt_frame_label_x = 5/6; gbl.menu_label(menu_frame, "W.r.t. frame:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], diffkine_wrt_frame_label_x * menu_properties['width'], diffkine_wrt_frame_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        diffkine_wrt_frame_button_x = diffkine_wrt_frame_label_x; self.diffkine_wrt_frame_button = gbl.menu_button(menu_frame, self.diffkine_wrt_frame, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], diffkine_wrt_frame_button_x * menu_properties['width'], diffkine_wrt_frame_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_diffkine_wrt_frame).button
        # show_diffkine_info_button_x = 19/20; self.show_diffkine_info_button = gbl.menu_button(menu_frame, "🕮", f"Calibri {menu_properties['options_font']+5} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_diffkine_info_button_x * menu_properties['width'], show_diffkine_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_diffkine_info).button
        self.diffkine_variables_sliders = []
        for k in range(self.joints_number):
            joint_type_index = self.joints_types_list.index(self.joints_types[k])  # the index of the current joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
            diffkine_velocities_limits = self.diffkine_velocities_limits[k][joint_type_index]  # the control variable limits of the current joint
            self.diffkine_variables_sliders.append(tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font'] - 3}", orient = tk.HORIZONTAL, from_ = diffkine_velocities_limits[0], to = diffkine_velocities_limits[1], resolution = [10**(-self.angles_precision), 10**(-self.distances_precision)][joint_type_index], length = menu_properties['width'] / 7.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = lambda event, slider_num = k: self.change_diffkine_variable_slider(slider_num, event)))
            # fkine_variables_sliders_x = 1/10 * (k + 1); self.fkine_variables_sliders.append(tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font'] - 3}", orient = tk.VERTICAL, length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = lambda event, slider_num = k: self.change_fkine_variable_slider(slider_num, event)))
            diffkine_variables_sliders_x = 1/(np.ceil(self.joints_number / difffkine_variables_sliders_rows) + 1) * (k // difffkine_variables_sliders_rows + 1); self.diffkine_variables_sliders[k].place(x = diffkine_variables_sliders_x * menu_properties['width'], y = diffkine_variables_sliders_ord_list[k] * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
            diffkine_variables_labels_x = 1/(np.ceil(self.joints_number / difffkine_variables_sliders_rows) + 1) * (k // difffkine_variables_sliders_rows + 1) - 2/3*1/7; gbl.menu_label(menu_frame, f"{k + 1}\n{['(°/s)', '(m/s)'][joint_type_index]}:", f"Calibri {menu_properties['options_font']-3} bold", menu_properties['labels_color'], menu_properties['bg_color'], diffkine_variables_labels_x * menu_properties['width'], diffkine_variables_sliders_ord_list[k] * menu_properties['height'] / (menu_properties['rows'] + 1) - 0.8)
        self.update_differential_kinematics_indicators()  # update the indicators of the differential kinematics of the robotic manipulator
    def generate_inverse_differential_kinematics_menu(self, menu_frame, menu_properties, event = None):  # build the inverse differential kinematics menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        get_diffkine_result_button_ord = 2.5
        joints_configuration_invkine_label_ord = get_diffkine_result_button_ord-0.4
        joints_configuration_invkine_indicator_ord = joints_configuration_invkine_label_ord+0.8
        send_invdiffkine_result_button_ord = get_diffkine_result_button_ord
        end_effector_linear_velocity_label_ord = get_diffkine_result_button_ord+1.8
        end_effector_linear_velocity_button_ord = end_effector_linear_velocity_label_ord
        end_effector_angular_velocity_label_ord = end_effector_linear_velocity_label_ord
        end_effector_angular_velocity_button_ord = end_effector_angular_velocity_label_ord
        invdiffkine_wrt_frame_label_ord = end_effector_linear_velocity_label_ord+1.8
        invdiffkine_wrt_frame_button_ord = invdiffkine_wrt_frame_label_ord
        joints_velocities_label_ord = end_effector_linear_velocity_label_ord+3.5
        joints_velocities_indicator_ord = joints_velocities_label_ord
        show_invdiffkine_info_button_ord = menu_properties['rows']-1.0
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        joints_configuration_invkine_label_x = 6/7; gbl.menu_label(menu_frame, "Configuration:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_configuration_invkine_label_x * menu_properties['width'], joints_configuration_invkine_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_configuration_invkine_indicator_x = joints_configuration_invkine_label_x; self.joints_configuration_invkine_indicator = gbl.menu_label(menu_frame, self.control_or_kinematics_variables_visualization, f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], joints_configuration_invkine_indicator_x * menu_properties['width'], joints_configuration_invkine_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        get_diffkine_result_button_x = 1/4; self.get_diffkine_result_button = gbl.menu_button(menu_frame, "↓ get differential kinematics\nanalysis result", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], get_diffkine_result_button_x * menu_properties['width'], get_diffkine_result_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.get_diffkine_velocities_result).button
        send_invdiffkine_result_button_x = 3/5; self.send_invdiffkine_result_button = gbl.menu_button(menu_frame, "↑ send inverse differential\nkinematics analysis result", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_invdiffkine_result_button_x * menu_properties['width'], send_invdiffkine_result_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_invdiffkine_joints_velocities_result).button
        end_effector_linear_velocity_label_x = 5/35; gbl.menu_label(menu_frame, "End-effector\nlinear velocity (m/s):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_linear_velocity_label_x * menu_properties['width'], end_effector_linear_velocity_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_linear_velocity_button_x = 13/35; self.end_effector_linear_velocity_button = gbl.menu_button(menu_frame, "linear\nvelocity", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], end_effector_linear_velocity_button_x * menu_properties['width'], end_effector_linear_velocity_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_end_effector_linear_velocity).button
        end_effector_angular_velocity_label_x = 21/35; gbl.menu_label(menu_frame, "End-effector\nangular velocity (°/s):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_angular_velocity_label_x * menu_properties['width'], end_effector_angular_velocity_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_angular_velocity_button_x = 29/35; self.end_effector_angular_velocity_button = gbl.menu_button(menu_frame, "angular\nvelocity", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], end_effector_angular_velocity_button_x * menu_properties['width'], end_effector_angular_velocity_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_end_effector_angular_velocity).button
        invdiffkine_wrt_frame_label_x = 2/5; gbl.menu_label(menu_frame, "Velocities defined w.r.t. frame:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], invdiffkine_wrt_frame_label_x * menu_properties['width'], invdiffkine_wrt_frame_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        invdiffkine_wrt_frame_button_x = 7/10; self.invdiffkine_wrt_frame_button = gbl.menu_button(menu_frame, self.invdiffkine_wrt_frame, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], invdiffkine_wrt_frame_button_x * menu_properties['width'], invdiffkine_wrt_frame_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_invdiffkine_wrt_frame).button
        joints_velocities_label_x = 1/6; gbl.menu_label(menu_frame, "Joints\nvelocities:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_velocities_label_x * menu_properties['width'], joints_velocities_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_velocities_indicator_x = 3/5; self.joints_velocities_indicator = gbl.menu_label(menu_frame, "joints velocities", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], joints_velocities_indicator_x * menu_properties['width'], joints_velocities_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        # show_invdiffkine_info_button_x = 19/20; self.show_invdiffkine_info_button = gbl.menu_button(menu_frame, "🕮", f"Calibri {menu_properties['options_font']+5} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_invdiffkine_info_button_x * menu_properties['width'], show_invdiffkine_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_invdiffkine_info).button
        self.update_inverse_differential_kinematics_indicators()  # update the indicators of the inverse differential kinematics of the robotic manipulator
    def build_control_robotic_manipulator_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus for the main menu that controls the robotic manipulator
        self.clear_menus_background()  # clear the menus background
        connection_menu_title = submenus_titles[0]
        connection_menu_info = f"--- {connection_menu_title} ---\n" + submenus_descriptions[0]
        monitor_menu_title = submenus_titles[1]
        monitor_menu_info = f"--- {monitor_menu_title} ---\n" + submenus_descriptions[1]
        control_menu_title = submenus_titles[2]
        control_menu_info = f"--- {control_menu_title} ---\n" + submenus_descriptions[2]
        if not self.expanded_serial_monitor:  # if the user does not choose the expanded serial monitor
            # create the establish connection sub menu
            connection_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = connection_menu_title, info = connection_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 6, \
                                                rows = 3, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # create the serial monitor menu
            monitor_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = monitor_menu_title, info = monitor_menu_info, row = 2, column = 0, width = self.menus_background_width, height = self.menus_background_height * 2 / 6, \
                                            rows = 8, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # create the control joints sub menu
            control_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = control_menu_title, info = control_menu_info, row = 3, column = 0, width = self.menus_background_width, height = self.menus_background_height * 3 / 6, \
                                            rows = 10, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # generate the sub menus
            self.generate_establish_connection_menu(self.create_static_menu_frame(connection_menu_properties), connection_menu_properties)
            self.generate_serial_monitor_menu(self.create_static_menu_frame(monitor_menu_properties), monitor_menu_properties)
            self.generate_control_joints_menu(self.create_static_menu_frame(control_menu_properties), control_menu_properties)
        else:  # if the user chooses the expanded serial monitor
            # create the serial monitor menu
            monitor_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = monitor_menu_title, info = monitor_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 2, \
                                            rows = 8, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # create the control joints sub menu
            control_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = control_menu_title, info = control_menu_info, row = 2, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 2, \
                                            rows = 10, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # generate the sub menus
            self.generate_serial_monitor_menu(self.create_static_menu_frame(monitor_menu_properties), monitor_menu_properties)
            self.generate_control_joints_menu(self.create_static_menu_frame(control_menu_properties), control_menu_properties)
    def generate_establish_connection_menu(self, menu_frame, menu_properties, event = None):  # build the establish connection menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        serial_port_label_ord = 2
        baudrate_label_ord = 3
        serial_ports_search_button_ord = serial_port_label_ord
        serial_connection_state_label_ord = 2
        serial_connection_indicator_ord = 3
        serial_connect_button_ord = 2.5
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        serial_port_label_x = 1/7; gbl.menu_label(menu_frame, "Serial port:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], serial_port_label_x * menu_properties['width'], serial_port_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.serial_ports_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 7, values = self.available_serial_ports, justify = "center")
        serial_ports_combobox_x = 2/7; self.serial_ports_combobox.place(x = serial_ports_combobox_x * menu_properties['width'], y = serial_port_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.serial_ports_combobox.bind("<<ComboboxSelected>>", self.change_serial_port)
        serial_ports_search_button_x = 2.7/7; self.serial_ports_search_button = gbl.menu_button(menu_frame, "⟲", f"Calibri {menu_properties['options_font']-2} bold", menu_properties['buttons_color'], menu_properties['bg_color'], serial_ports_search_button_x * menu_properties['width'], serial_ports_search_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.obtain_serial_ports).button
        baudrate_label_x = serial_port_label_x; gbl.menu_label(menu_frame, "Baud rate:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], baudrate_label_x * menu_properties['width'], baudrate_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.baudrates_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 7, values = self.baudrates_list, justify = "center")
        baudrates_combobox_x = serial_ports_combobox_x; self.baudrates_combobox.place(x = baudrates_combobox_x * menu_properties['width'], y = baudrate_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.baudrates_combobox.bind("<<ComboboxSelected>>", self.change_baudrate)
        serial_connection_state_label_x = 4/7; gbl.menu_label(menu_frame, "Serial connection state:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], serial_connection_state_label_x * menu_properties['width'], serial_connection_state_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        serial_connection_indicator_x = 4/7; self.serial_connection_indicator = gbl.menu_label(menu_frame, self.serial_connection_state, f"Calibri {menu_properties['options_font']} bold", "black", self.serial_connection_indicator_colors[self.serial_connection_states_list.index(self.serial_connection_state)], serial_connection_indicator_x * menu_properties['width'], serial_connection_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        serial_connect_button_x = 6/7; self.serial_connect_button = gbl.menu_button(menu_frame, self.serial_connect_disconnect_command, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], serial_connect_button_x * menu_properties['width'], serial_connect_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.serial_connect_disconnect).button
        self.update_serial_connection_indicators(self.serial_connection_state)  # update the serial connection indicator according to the current serial connection state
    def generate_serial_monitor_menu(self, menu_frame, menu_properties, event = None):  # build the serial monitor menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        console_serial_monitor_ord = int(menu_properties['rows'] / 2) + 0.5
        command_entrybox_label_ord = menu_properties['rows']
        command_entrybox_ord = command_entrybox_label_ord
        send_command_button_ord = command_entrybox_label_ord
        command_starting_text_label_ord = 3
        command_starting_text_entrybox_ord = command_starting_text_label_ord+1
        command_ending_text_label_ord = command_starting_text_label_ord+2
        command_ending_text_entrybox_ord = command_ending_text_label_ord+1
        show_hide_ok_label_ord = 1
        show_hide_ok_button_ord = 1
        show_hide_status_label_ord = 2
        show_hide_status_button_ord = 2
        expand_serial_monitor_menu_ord = 1
        clear_serial_monitor_ord = 2
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        self.console_serial_monitor = tk.Text(menu_frame, font = f"Calibri 10", width = 60, height = [8, 14][[False, True].index(self.expanded_serial_monitor)], bg = "white", fg = "black", wrap = "word")
        console_serial_monitor_x = 3/8; self.console_serial_monitor.place(x = console_serial_monitor_x * menu_properties['width'], y = console_serial_monitor_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.console_serial_monitor.insert(tk.END, self.serial_monitor_text)  # insert the saved serial monitor text to the console window
        self.console_serial_monitor.see("end")  # make the console serial monitor to scroll to the end
        command_entrybox_label_x = 1/10; gbl.menu_label(menu_frame, "Command:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], command_entrybox_label_x * menu_properties['width'], command_entrybox_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.command_entrybox = ttk.Entry(menu_frame, font = f"Calibri 10", width = 60, justify = "left")
        command_entrybox_x = 1/2; self.command_entrybox.place(x = command_entrybox_x * menu_properties['width'], y = command_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.command_entrybox.bind("<Return>", lambda event: self.send_serial_command(self.command_starting_text + self.command_entrybox.get() + self.command_ending_text, event))
        send_command_button_x = 9/10; self.send_command_button = gbl.menu_button(menu_frame, "Send", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_command_button_x * menu_properties['width'], send_command_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: self.send_serial_command(self.command_starting_text + self.command_entrybox.get() + self.command_ending_text, event)).button
        command_starting_text_label_x = 5/6; gbl.menu_label(menu_frame, "Command starting text:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], command_starting_text_label_x * menu_properties['width'], command_starting_text_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.command_starting_text_entrybox = ttk.Entry(menu_frame, font = f"Calibri 10", width = 15, justify = "left")
        command_starting_text_entrybox_x = 5/6; self.command_starting_text_entrybox.place(x = command_starting_text_entrybox_x * menu_properties['width'], y = command_starting_text_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.command_starting_text_entrybox.insert(0, self.command_starting_text)  # insert the saved command starting text to the entry box
        self.command_starting_text_entrybox.bind("<Return>", self.change_command_starting_text)
        command_ending_text_label_x = 5/6; gbl.menu_label(menu_frame, "Command ending text:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], command_ending_text_label_x * menu_properties['width'], command_ending_text_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.command_ending_text_entrybox = ttk.Entry(menu_frame, font = f"Calibri 10", width = 15, justify = "left")
        command_ending_text_entrybox_x = 5/6; self.command_ending_text_entrybox.place(x = command_ending_text_entrybox_x * menu_properties['width'], y = command_ending_text_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.command_ending_text_entrybox.insert(0, self.command_ending_text)  # insert the saved command ending text to the entry box
        self.command_ending_text_entrybox.bind("<Return>", self.change_command_ending_text)
        show_hide_ok_label_x = 8/10; gbl.menu_label(menu_frame, "OK:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], show_hide_ok_label_x * menu_properties['width'], show_hide_ok_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_hide_ok_button_x = 13/15; self.show_hide_ok_button = gbl.menu_button(menu_frame, self.show_responses_indicators[[False, True].index(self.show_ok_responses)], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_hide_ok_button_x * menu_properties['width'], show_hide_ok_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_hide_ok_responses_on_console).button
        show_hide_status_label_x = 8/10; gbl.menu_label(menu_frame, "Status:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], show_hide_status_label_x * menu_properties['width'], show_hide_status_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_hide_status_button_x = 13/15; self.show_hide_status_button = gbl.menu_button(menu_frame, self.show_responses_indicators[[False, True].index(self.show_status_responses)], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_hide_status_button_x * menu_properties['width'], show_hide_status_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_hide_status_responses_on_console).button
        expand_serial_monitor_menu_x = 19/20; self.expand_serial_monitor_menu_button = gbl.menu_button(menu_frame, ["⇱", "⇲"][[False, True][self.expanded_serial_monitor]], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], expand_serial_monitor_menu_x * menu_properties['width'], expand_serial_monitor_menu_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.expand_serial_monitor_menu).button
        clear_serial_monitor_x = 19/20; self.clear_serial_monitor_button = gbl.menu_button(menu_frame, "🗑", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], clear_serial_monitor_x * menu_properties['width'], clear_serial_monitor_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.clear_serial_monitor).button
    def generate_control_joints_menu(self, menu_frame, menu_properties, event = None):  # build the control joints menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        choose_joint_number_control_label_ord = 2.5
        joint_type_indicator_ord = choose_joint_number_control_label_ord
        joint_control_variable_combobox_ord = choose_joint_number_control_label_ord
        get_kinematics_values_button_ord = joint_control_variable_combobox_ord
        choose_joint_motors_label_ord = choose_joint_number_control_label_ord+1.5
        increase_joint_motors_button_ord = choose_joint_motors_label_ord-0.4
        decrease_joints_motors_button_ord = choose_joint_motors_label_ord+0.4
        choose_motors_mult_factors_label_ord = choose_joint_motors_label_ord
        joint_variable_slider_label_ord = choose_joint_motors_label_ord+1.5
        joint_variable_slider_ord = joint_variable_slider_label_ord+0.5
        set_joints_variables_to_zero_button_ord = joint_variable_slider_label_ord+1.0
        increase_joint_var_1_button_ord = joint_variable_slider_ord-0.4
        decrease_joint_var_1_button_ord = joint_variable_slider_ord+0.4
        increase_joint_var_2_button_ord = joint_variable_slider_ord-0.4
        decrease_joint_var_2_button_ord = joint_variable_slider_ord+0.4
        increase_joint_var_3_button_ord = joint_variable_slider_ord-0.4
        decrease_joint_var_3_button_ord = joint_variable_slider_ord+0.4
        end_effector_slider_label_ord = joint_variable_slider_label_ord+2
        end_effector_slider_ord = end_effector_slider_label_ord+0.5
        set_end_effector_to_zero_button_ord = end_effector_slider_label_ord+1
        choose_end_effector_motor_label_ord = end_effector_slider_label_ord
        choose_end_effector_motor_entrybox_ord = choose_end_effector_motor_label_ord+1
        choose_end_effector_mult_factor_label_ord = end_effector_slider_label_ord
        choose_end_effector_mult_factor_entrybox_ord = choose_end_effector_mult_factor_label_ord+1
        choose_control_mode_label_ord = end_effector_slider_label_ord+2.5
        choose_control_mode_button_ord = choose_control_mode_label_ord
        send_command_end_effector_button_ord = choose_control_mode_label_ord
        send_command_all_motors_button_ord = choose_control_mode_label_ord
        send_command_joint_motors_button_ord = choose_control_mode_label_ord
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        choose_joint_number_control_label_x = 4/30; gbl.menu_label(menu_frame, "Joint number:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_number_control_label_x * menu_properties['width'], choose_joint_number_control_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_number_control_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = [f"joint {joint}" for joint in range(1, self.joints_number + 1)], justify = "center")
        choose_joint_number_control_combobox_x = 10/30; self.choose_joint_number_control_combobox.place(x = choose_joint_number_control_combobox_x * menu_properties['width'], y = choose_joint_number_control_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_number_control_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_control)
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        joint_type_indicator_x = 16/30; self.joint_type_indicator = gbl.menu_label(menu_frame, self.joints_types[self.chosen_joint_number_control - 1] + [" (degrees)", " (meters)"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", "black", "orange", joint_type_indicator_x * menu_properties['width'], joint_type_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        self.joint_control_variable_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 8, values = "", justify = "center")
        joint_control_variable_combobox_x = 22/30; self.joint_control_variable_combobox.place(x = joint_control_variable_combobox_x * menu_properties['width'], y = joint_control_variable_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.joint_control_variable_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_control_2)
        self.joint_control_variable_combobox.bind("<Return>", self.change_chosen_control_variable)
        get_kinematics_values_button_x = 27/30; self.get_kinematics_values_button = gbl.menu_button(menu_frame, "get\nkinematics\nvalues", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], get_kinematics_values_button_x * menu_properties['width'], get_kinematics_values_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.copy_fkine_to_control_values).button
        choose_joint_motors_label_x = 4/30; gbl.menu_label(menu_frame, "Joint motors:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_motors_label_x * menu_properties['width'], choose_joint_motors_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_motors_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = self.joints_motors_list[self.chosen_joint_number_control - 1], justify = "center")
        choose_joint_motors_combobox_x = 10/30; self.choose_joint_motors_combobox.place(x = choose_joint_motors_combobox_x * menu_properties['width'], y = choose_joint_motors_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_motors_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_motors)
        increase_joint_motors_button_x = 13/30; self.increase_joint_motors_button = gbl.menu_button(menu_frame, "+", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], increase_joint_motors_button_x * menu_properties['width'], increase_joint_motors_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.increase_chosen_joint_motors).button
        decrease_joints_motors_button_x = 13/30; self.decrease_joints_motors_button = gbl.menu_button(menu_frame, "-", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], decrease_joints_motors_button_x * menu_properties['width'], decrease_joints_motors_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.decrease_chosen_joint_motors).button
        choose_motors_mult_factors_label_x = 18/30; gbl.menu_label(menu_frame, "Motors factors\n(for commands):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_motors_mult_factors_label_x * menu_properties['width'], choose_motors_mult_factors_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.motors_mult_factors_entrybox = ttk.Entry(menu_frame, font = f"Calibri {menu_properties['options_font']}", width = 8, justify = "center")
        motors_mult_factors_entrybox_x = 23/30; self.motors_mult_factors_entrybox.place(x = motors_mult_factors_entrybox_x * menu_properties['width'], y = choose_motors_mult_factors_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.motors_mult_factors_entrybox.bind("<Return>", self.change_motors_mult_factors_entrybox)
        joint_variable_slider_label_x = 1/6; gbl.menu_label(menu_frame, "Joint variable control:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joint_variable_slider_label_x * menu_properties['width'], joint_variable_slider_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.joint_variable_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']}", orient = tk.HORIZONTAL, length = menu_properties['width'] / 3.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_control_variable_slider)
        joint_variable_slider_x = 1/2; self.joint_variable_slider.place(x = joint_variable_slider_x * menu_properties['width'], y = joint_variable_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        set_joints_variables_to_zero_button_x = 1/6; self.set_joints_variables_to_zero_button = gbl.menu_button(menu_frame, "set joints to 0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], set_joints_variables_to_zero_button_x * menu_properties['width'], set_joints_variables_to_zero_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.set_joints_variables_to_zero).button
        increase_joint_var_1_button_x = 9/12; self.increase_joint_var_1_button = gbl.menu_button(menu_frame, ["+0.1", "+0.001"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], increase_joint_var_1_button_x * menu_properties['width'], increase_joint_var_1_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = 1: self.increase_decrease_control_variable(change_type)).button
        decrease_joint_var_1_button_x = 9/12; self.decrease_joint_var_1_button = gbl.menu_button(menu_frame, ["-0.1", "-0.001"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], decrease_joint_var_1_button_x * menu_properties['width'], decrease_joint_var_1_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = -1: self.increase_decrease_control_variable(change_type)).button
        increase_joint_var_2_button_x = 10/12; self.increase_joint_var_2_button = gbl.menu_button(menu_frame, ["+1", "+0.01"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], increase_joint_var_2_button_x * menu_properties['width'], increase_joint_var_2_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = 2: self.increase_decrease_control_variable(change_type)).button
        decrease_joint_var_2_button_x = 10/12; self.decrease_joint_var_2_button = gbl.menu_button(menu_frame, ["-1", "-0.01"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], decrease_joint_var_2_button_x * menu_properties['width'], decrease_joint_var_2_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = -2: self.increase_decrease_control_variable(change_type)).button
        increase_joint_var_3_button_x = 11/12; self.increase_joint_var_3_button = gbl.menu_button(menu_frame, ["+10", "+0.1"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], increase_joint_var_3_button_x * menu_properties['width'], increase_joint_var_3_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = 3: self.increase_decrease_control_variable(change_type)).button
        decrease_joint_var_3_button_x = 11/12; self.decrease_joint_var_3_button = gbl.menu_button(menu_frame, ["-10", "-0.1"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], decrease_joint_var_3_button_x * menu_properties['width'], decrease_joint_var_3_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = -3: self.increase_decrease_control_variable(change_type)).button
        end_effector_slider_label_x = 1/6; gbl.menu_label(menu_frame, "End-effector control:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_slider_label_x * menu_properties['width'], end_effector_slider_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.end_effector_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']}", orient = tk.HORIZONTAL, from_ = self.control_end_effector_limits[0], to = self.control_end_effector_limits[1], length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_control_end_effector_slider)
        end_effector_slider_x = 3/7; self.end_effector_slider.place(x = end_effector_slider_x * menu_properties['width'], y = end_effector_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        set_end_effector_to_zero_button_x = 1/6; self.set_end_effector_to_zero_button = gbl.menu_button(menu_frame, "set end-effector to 0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], set_end_effector_to_zero_button_x * menu_properties['width'], set_end_effector_to_zero_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.set_end_effector_to_zero).button
        choose_end_effector_motor_label_x = 4/6; gbl.menu_label(menu_frame, "Motor:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_end_effector_motor_label_x * menu_properties['width'], choose_end_effector_motor_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.end_effector_motor_entrybox = ttk.Entry(menu_frame, font = f"Calibri {menu_properties['options_font']}", width = 8, justify = "center")
        end_effector_motor_entrybox_x = 4/6; self.end_effector_motor_entrybox.place(x = end_effector_motor_entrybox_x * menu_properties['width'], y = choose_end_effector_motor_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.end_effector_motor_entrybox.bind("<Return>", self.change_end_effector_motor)
        choose_end_effector_mult_factor_label_x = 6/7; gbl.menu_label(menu_frame, "Motor factor:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_end_effector_mult_factor_label_x * menu_properties['width'], choose_end_effector_mult_factor_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.end_effector_mult_factor_entrybox = ttk.Entry(menu_frame, font = f"Calibri {menu_properties['options_font']}", width = 8, justify = "center")
        end_effector_mult_factor_x = 6/7; self.end_effector_mult_factor_entrybox.place(x = end_effector_mult_factor_x * menu_properties['width'], y = choose_end_effector_mult_factor_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.end_effector_mult_factor_entrybox.bind("<Return>", self.change_end_effector_mult_factor)
        choose_control_mode_label_x = 1/6; gbl.menu_label(menu_frame, "Control mode:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_control_mode_label_x * menu_properties['width'], choose_control_mode_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_control_mode_button_x = 2/6; self.choose_control_mode_button = gbl.menu_button(menu_frame, self.robotic_manipulator_control_mode, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_control_mode_button_x * menu_properties['width'], choose_control_mode_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_robotic_manipulator_control_mode).button
        send_command_joint_motors_button_x = 3/6; self.send_command_joint_motors_button = gbl.menu_button(menu_frame, "GO\njoint", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_command_joint_motors_button_x * menu_properties['width'], send_command_joint_motors_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_command_to_chosen_joint_motors).button
        send_command_all_motors_button_x = 4/6; self.send_command_all_motors_button = gbl.menu_button(menu_frame, "GO ALL\njoints", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_command_all_motors_button_x * menu_properties['width'], send_command_all_motors_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_command_to_all_motors).button
        send_command_end_effector_button_x = 5/6; self.send_command_end_effector_button = gbl.menu_button(menu_frame, "GO\nend-effector", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_command_end_effector_button_x * menu_properties['width'], send_command_end_effector_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_command_to_end_effector).button
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def build_camera_control_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus of the main menu that controls the camera
        self.clear_menus_background()
        # create the camera control sub menu
        camera_menu_title = submenus_titles[0]
        camera_menu_info = f"--- {camera_menu_title} ---\n" + submenus_descriptions[0]
        camera_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = camera_menu_title, info = camera_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 1, \
                                        rows = 17, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_camera_control_menu(self.create_static_menu_frame(camera_menu_properties), camera_menu_properties)
    def generate_camera_control_menu(self, menu_frame, menu_properties, event = None):  # build the camera control menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        define_camera_label_ord = 3
        open_close_camera_button_ord = define_camera_label_ord
        determine_camera_label_ord = define_camera_label_ord-1.0
        determine_camera_button_ord = define_camera_label_ord
        choose_aspect_ratio_label_ord = define_camera_label_ord-1.0
        choose_aspect_ratios_combobox_ord = define_camera_label_ord
        choose_resolution_label_ord = define_camera_label_ord-1.0
        choose_resolution_combobox_ord = define_camera_label_ord
        choose_windows_size_factor_label_ord = define_camera_label_ord-1.0
        choose_windows_size_factor_slider_ord = define_camera_label_ord
        choose_dFoV_label_ord = define_camera_label_ord+1.3
        choose_dFoV_button_ord = choose_dFoV_label_ord
        find_min_dist_2d_plane_label_ord = define_camera_label_ord+1.3
        find_min_dist_2d_plane_indicator_ord = find_min_dist_2d_plane_label_ord
        convert_to_grayscale_label_ord = 6
        choose_luminance_threshold_label_ord = convert_to_grayscale_label_ord-0.5
        choose_luminance_threshold_slider_ord = convert_to_grayscale_label_ord+0.5
        show_grayscale_image_button_ord = convert_to_grayscale_label_ord
        define_camera_pose_label_ord = 8
        choose_ArUco_marker_label_ord = define_camera_pose_label_ord+1
        choose_ArUco_marker_combobox_ord = choose_ArUco_marker_label_ord+1
        estimate_camera_pose_button_ord = choose_ArUco_marker_label_ord-0.1
        estimate_obst_plane_pose_button_ord = estimate_camera_pose_button_ord
        apply_moving_average_filter_label_ord = estimate_camera_pose_button_ord+1.3
        apply_moving_average_filter_button_ord = apply_moving_average_filter_label_ord
        camera_transformation_label_ord = define_camera_pose_label_ord+1.5
        camera_transformation_indicator_ord = camera_transformation_label_ord
        camera_optical_axis_label_ord = define_camera_pose_label_ord+3.3
        camera_optical_axis_button_ord = camera_optical_axis_label_ord
        camera_translation_label_ord = camera_optical_axis_button_ord+1
        camera_translation_button_ord = camera_translation_label_ord
        camera_orientation_label_ord = camera_translation_button_ord+1
        camera_orientation_button_ord = camera_orientation_label_ord
        z_rotate_camera_label_ord = camera_optical_axis_label_ord+0.3
        z_rotate_camera_slider_ord = z_rotate_camera_label_ord+1.3
        normal_translate_camera_label_ord = z_rotate_camera_label_ord
        normal_translate_camera_slider_ord = normal_translate_camera_label_ord+1.3
        camera_capture_workspacelabel_ord = 14.5
        recalibrate_camera_label_ord = camera_capture_workspacelabel_ord+1
        recalibrate_camera_button_ord = recalibrate_camera_label_ord+0.8
        show_camera_parameters_button_ord = recalibrate_camera_button_ord+0.8
        show_workspace_image_label_ord = recalibrate_camera_label_ord+0.4
        show_workspace_image_combobox_ord = show_workspace_image_label_ord+0.8
        show_workspace_image_button_ord = show_workspace_image_label_ord-0.4
        rename_workspace_image_button_ord = show_workspace_image_button_ord+0.8
        delete_workspace_image_button_ord = rename_workspace_image_button_ord+0.8
        draw_2d_plane_button_ord = show_workspace_image_label_ord
        capture_workspace_image_button_ord = show_workspace_image_label_ord+1.2
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        define_camera_label_x = 1/8; gbl.menu_label(menu_frame, "Define the\ncamera object:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], define_camera_label_x * menu_properties['width'], define_camera_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        open_close_camera_button_x = 7/8; self.open_close_camera_button = gbl.menu_button(menu_frame, ["open\ncamera", "close\ncamera"][[False, True].index(self.camera_thread_flag)], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], open_close_camera_button_x * menu_properties['width'], open_close_camera_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.open_close_camera).button
        determine_camera_label_x = 2/7; gbl.menu_label(menu_frame, "Camera device:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], determine_camera_label_x * menu_properties['width'], determine_camera_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        determine_camera_button_x = determine_camera_label_x; self.determine_camera_button = gbl.menu_button(menu_frame, self.camera_choice, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], determine_camera_button_x * menu_properties['width'], determine_camera_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.determine_camera).button
        choose_aspect_ratio_label_x = 3/7; gbl.menu_label(menu_frame, "Aspect ratio:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_aspect_ratio_label_x * menu_properties['width'], choose_aspect_ratio_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_aspect_ratio_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 5, values = self.camera_aspect_ratios_list, justify = "center")
        choose_aspect_ratio_combobox_x = choose_aspect_ratio_label_x; self.choose_aspect_ratio_combobox.place(x = choose_aspect_ratio_combobox_x * menu_properties['width'], y = choose_aspect_ratios_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_aspect_ratio_combobox.bind("<<ComboboxSelected>>", self.change_camera_aspect_ratio)
        choose_resolution_label_x = 4/7; gbl.menu_label(menu_frame, "Resolution:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_resolution_label_x * menu_properties['width'], choose_resolution_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_resolution_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 5, values = self.camera_resolutions_list[self.camera_aspect_ratios_values.index(self.camera_aspect_ratio)], justify = "center")
        choose_resolution_combobox_x = choose_resolution_label_x; self.choose_resolution_combobox.place(x = choose_resolution_combobox_x * menu_properties['width'], y = choose_resolution_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_resolution_combobox.bind("<<ComboboxSelected>>", self.change_camera_resolution)
        choose_windows_size_factor_label_x = 5/7; gbl.menu_label(menu_frame, "Image size (%):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_windows_size_factor_label_x * menu_properties['width'], choose_windows_size_factor_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_windows_size_factor_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']-3}", orient = tk.HORIZONTAL, from_ = 0, to = 100, resolution = 1, length = menu_properties['width'] / 10.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_windows_size_factor_slider)
        choose_windows_size_factor_slider_x = choose_windows_size_factor_label_x; self.choose_windows_size_factor_slider.place(x = choose_windows_size_factor_slider_x * menu_properties['width'], y = choose_windows_size_factor_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        camera_dFoV_label_x = 2/7; gbl.menu_label(menu_frame, "Diagonal FoV (°):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], camera_dFoV_label_x * menu_properties['width'], choose_dFoV_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        camera_dFoV_button_x = 3/7; self.choose_dFoV_button = gbl.menu_button(menu_frame, f"{self.camera_dFoV:.1f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], camera_dFoV_button_x * menu_properties['width'], choose_dFoV_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_camera_dFoV).button
        find_min_dist_label_x = 4/7; gbl.menu_label(menu_frame, "Min distance (m)\nfrom 2D plane:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], find_min_dist_label_x * menu_properties['width'], find_min_dist_2d_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        find_min_dist_indicator_x = 5/7; self.find_min_dist_2d_plane_indicator = gbl.menu_label(menu_frame, "min\ndistance", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], find_min_dist_indicator_x * menu_properties['width'], find_min_dist_2d_plane_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        convert_frame_black_white_label_x = 1/6; gbl.menu_label(menu_frame, "Convert frames to\nblack & white:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], convert_frame_black_white_label_x * menu_properties['width'], convert_to_grayscale_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_luminance_threshold_label_x = 1/2; gbl.menu_label(menu_frame, "Luminance threshold:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_luminance_threshold_label_x * menu_properties['width'], choose_luminance_threshold_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_luminance_threshold_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']-3}", orient = tk.HORIZONTAL, from_ = self.luminance_threshold_limits[0], to = self.luminance_threshold_limits[1], length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_luminance_threshold_slider)
        choose_luminance_threshold_slider_x = 1/2; self.choose_luminance_threshold_slider.place(x = choose_luminance_threshold_slider_x * menu_properties['width'], y = choose_luminance_threshold_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        show_grayscale_image_button_x = 4/5; self.show_grayscale_image_button = gbl.menu_button(menu_frame, "show\ngrayscale image", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_grayscale_image_button_x * menu_properties['width'], show_grayscale_image_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_grayscale_image).button
        define_camera_pose_label_x = 1/2; gbl.menu_label(menu_frame, "Define the camera pose:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], define_camera_pose_label_x * menu_properties['width'], define_camera_pose_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_ArUco_marker_label_x = 7/16; gbl.menu_label(menu_frame, "ArUco marker:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_ArUco_marker_label_x * menu_properties['width'], choose_ArUco_marker_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_ArUco_marker_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", values = self.saved_ArUco_markers_list, state = "normal", width = 15, justify = "center")
        choose_ArUco_marker_combobox_x = choose_ArUco_marker_label_x; self.choose_ArUco_marker_combobox.place(x = choose_ArUco_marker_combobox_x * menu_properties['width'], y = choose_ArUco_marker_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_ArUco_marker_combobox.bind("<<ComboboxSelected>>", self.change_ArUco_marker)
        self.choose_ArUco_marker_combobox.bind("<Return>", self.change_ArUco_marker)
        estimate_camera_pose_button_x = 1/9; self.camera_estimate_pose_button = gbl.menu_button(menu_frame, "estimate\ncamera pose", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], estimate_camera_pose_button_x * menu_properties['width'], estimate_camera_pose_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.estimate_camera_pose).button
        estimate_obst_plane_pose_button_x = 1/4; self.estimate_obst_plane_pose_button = gbl.menu_button(menu_frame, "estimate\nplane pose", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], estimate_obst_plane_pose_button_x * menu_properties['width'], estimate_obst_plane_pose_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.estimate_obstacles_plane_pose).button
        apply_moving_average_filter_label_x = 1/9; gbl.menu_label(menu_frame, "Apply moving\naverage filter:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], apply_moving_average_filter_label_x * menu_properties['width'], apply_moving_average_filter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        apply_moving_average_filter_button_x = 1/4; self.apply_moving_average_filter_button = gbl.menu_button(menu_frame, ["no", "yes"][[False, True].index(self.apply_moving_average_poses_estimation)], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_moving_average_filter_button_x * menu_properties['width'], apply_moving_average_filter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_moving_average_filter).button
        camera_transformation_label_x = 5/8; gbl.menu_label(menu_frame, "Camera\ntransformation\n(w.r.t. world):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], camera_transformation_label_x * menu_properties['width'], camera_transformation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        camera_transformation_indicator_x = 5/6; self.camera_transformation_indicator = gbl.menu_label(menu_frame, "transformation", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], camera_transformation_indicator_x * menu_properties['width'], camera_transformation_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        camera_translation_label_x = 1/8; gbl.menu_label(menu_frame, "Translation (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], camera_translation_label_x * menu_properties['width'], camera_translation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        camera_translation_button_x = 2/6; self.choose_camera_translation_button = gbl.menu_button(menu_frame, str([np.round(self.camera_translation[k], 3) for k in range(len(self.camera_translation))]), f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], camera_translation_button_x * menu_properties['width'], camera_translation_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_camera_translation).button
        camera_optical_axis_label_x = 1/8; gbl.menu_label(menu_frame, "Optical axis:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], camera_optical_axis_label_x * menu_properties['width'], camera_optical_axis_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        camera_optical_axis_button_x = 2/6; self.choose_camera_optical_axis_button = gbl.menu_button(menu_frame, str([np.round(self.camera_optical_axis[k], 3) for k in range(len(self.camera_optical_axis))]), f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], camera_optical_axis_button_x * menu_properties['width'], camera_optical_axis_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_camera_optical_axis).button
        camera_orientation_label_x = 1/8; gbl.menu_label(menu_frame, "Orientation (°):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], camera_orientation_label_x * menu_properties['width'], camera_orientation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        camera_orientation_button_x = 2/6; self.choose_camera_orientation_button = gbl.menu_button(menu_frame, f"{self.camera_orientation:.1f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], camera_orientation_button_x * menu_properties['width'], camera_orientation_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_camera_orientation).button
        z_rotate_camera_label_x = 7/12; gbl.menu_label(menu_frame, "Rotation around\nz-axis (°):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], z_rotate_camera_label_x * menu_properties['width'], z_rotate_camera_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.z_rotate_camera_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']-3}", orient = tk.HORIZONTAL, from_ = 0, to = 360, resolution = 10**(-self.angles_precision), length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_camera_move_sliders)
        z_rotate_camera_slider_x = z_rotate_camera_label_x; self.z_rotate_camera_slider.place(x = z_rotate_camera_slider_x * menu_properties['width'], y = z_rotate_camera_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.z_rotate_camera_slider.bind("<ButtonRelease-3>", self.reset_camera_sliders)
        normal_translate_camera_label_x = 5/6; gbl.menu_label(menu_frame, "Translation along\noptical axis (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], normal_translate_camera_label_x * menu_properties['width'], normal_translate_camera_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.normal_translate_camera_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']-3}", orient = tk.HORIZONTAL, from_ = -1, to = 1, resolution = 0.001, length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_camera_move_sliders)
        normal_translate_camera_slider_x = normal_translate_camera_label_x; self.normal_translate_camera_slider.place(x = normal_translate_camera_slider_x * menu_properties['width'], y = normal_translate_camera_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.normal_translate_camera_slider.bind("<ButtonRelease-3>", self.reset_camera_sliders)
        camera_capture_workspacelabel_x = 1/2; gbl.menu_label(menu_frame, "Operations for capturing the workspace image using the camera:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], camera_capture_workspacelabel_x * menu_properties['width'], camera_capture_workspacelabel_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        recalibrate_camera_label_x = 1/6; gbl.menu_label(menu_frame, "Camera calibration:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], recalibrate_camera_label_x * menu_properties['width'], recalibrate_camera_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        recalibrate_camera_button_x = recalibrate_camera_label_x; self.recalibrate_camera_button = gbl.menu_button(menu_frame, "recalibrate camera", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], recalibrate_camera_button_x * menu_properties['width'], recalibrate_camera_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.recalibrate_camera).button
        show_camera_parameters_button_x = recalibrate_camera_label_x; self.show_camera_parameters_button = gbl.menu_button(menu_frame, "show camera parameters", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_camera_parameters_button_x * menu_properties['width'], show_camera_parameters_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_camera_parameters).button
        draw_2d_plane_button_x = 5/11; self.draw_2d_plane_button = gbl.menu_button(menu_frame, "draw 2D plane\non frame", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], draw_2d_plane_button_x * menu_properties['width'], draw_2d_plane_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.draw_2d_plane_on_image).button
        capture_workspace_image_button_x = draw_2d_plane_button_x; self.capture_workspace_image_button = gbl.menu_button(menu_frame, "capture image", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], capture_workspace_image_button_x * menu_properties['width'], capture_workspace_image_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.capture_workspace_image).button
        show_workspace_image_label_x = 7/10; gbl.menu_label(menu_frame, "Workspace images:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], show_workspace_image_label_x * menu_properties['width'], show_workspace_image_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.show_workspace_image_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 13, values = self.saved_workspace_images_list, justify = "center")
        show_workspace_image_combobox_x = show_workspace_image_label_x; self.show_workspace_image_combobox.place(x = show_workspace_image_combobox_x * menu_properties['width'], y = show_workspace_image_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.show_workspace_image_combobox.bind("<<ComboboxSelected>>", self.change_shown_workspace_image)
        self.show_workspace_image_combobox.bind("<Return>", self.change_shown_workspace_image)
        show_workspace_image_button_x = 8/9; self.show_workspace_image_button = gbl.menu_button(menu_frame, "show image", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_workspace_image_button_x * menu_properties['width'], show_workspace_image_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_workspace_image).button
        rename_workspace_image_button_x = show_workspace_image_button_x; self.rename_workspace_image_button = gbl.menu_button(menu_frame, "rename image", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], rename_workspace_image_button_x * menu_properties['width'], rename_workspace_image_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.rename_workspace_image).button
        delete_workspace_image_button_x = show_workspace_image_button_x; self.delete_workspace_image_button = gbl.menu_button(menu_frame, "delete image", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], delete_workspace_image_button_x * menu_properties['width'], delete_workspace_image_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.delete_workspace_image).button
        self.update_camera_control_indicators()  # update the indicators of the camera control
        self.change_shown_workspace_image()  # change the shown workspace image
    def build_workspace_obstacles_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus of the main menu that creates the workspace obstacles
        self.clear_menus_background()
        # create the workspace obstacles sub menu
        obstacles_menu_title = submenus_titles[0]
        obstacles_menu_info = f"--- {obstacles_menu_title} ---\n" + submenus_descriptions[0]
        obstacles_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = obstacles_menu_title, info = obstacles_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 1, \
                                            rows = 17, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_workspace_obstacles_menu(self.create_static_menu_frame(obstacles_menu_properties), obstacles_menu_properties)
    def generate_workspace_obstacles_menu(self, menu_frame, menu_properties, event = None):  # build the make workspace obstacles menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        create_obstacles_2d_plane_label_ord = 2
        choose_2d_plane_x_length_label_ord = create_obstacles_2d_plane_label_ord+0.8
        choose_2d_plane_x_length_button_ord = choose_2d_plane_x_length_label_ord
        choose_2d_plane_y_length_label_ord = choose_2d_plane_x_length_label_ord+1
        choose_2d_plane_y_length_button_ord = choose_2d_plane_y_length_label_ord
        choose_2d_plane_normal_vector_label_ord = choose_2d_plane_y_length_label_ord+1
        choose_2d_plane_normal_vector_button_ord = choose_2d_plane_normal_vector_label_ord
        choose_2d_plane_translation_label_ord = choose_2d_plane_normal_vector_label_ord+1
        choose_2d_plane_translation_button_ord = choose_2d_plane_translation_label_ord
        choose_2d_plane_orientation_label_ord = choose_2d_plane_translation_label_ord+1
        choose_2d_plane_orientation_button_ord = choose_2d_plane_orientation_label_ord
        camera_plane_z_axis_alignment_label_ord = choose_2d_plane_orientation_label_ord+1
        camera_plane_z_axis_alignment_indicator_ord = camera_plane_z_axis_alignment_label_ord
        z_rotate_2d_plane_label_ord = create_obstacles_2d_plane_label_ord+1
        z_rotate_2d_plane_slider_ord = z_rotate_2d_plane_label_ord+1.3
        normal_translate_2d_plane_label_ord = z_rotate_2d_plane_label_ord
        normal_translate_2d_plane_silder_ord = normal_translate_2d_plane_label_ord+1.3
        obstacles_transformation_label_ord = z_rotate_2d_plane_label_ord+3.7
        ArUco_marker_pose_on_plane_label_ord = camera_plane_z_axis_alignment_label_ord+1
        define_ArUco_marker_position_label_ord = ArUco_marker_pose_on_plane_label_ord
        define_ArUco_marker_position_button_ord = ArUco_marker_pose_on_plane_label_ord
        define_ArUco_marker_orientation_label_ord = ArUco_marker_pose_on_plane_label_ord
        define_ArUco_marker_orientation_button_ord = ArUco_marker_pose_on_plane_label_ord
        obstacles_transformation_indicator_ord = obstacles_transformation_label_ord
        plane_singularities_label_ord = create_obstacles_2d_plane_label_ord+8.5
        plane_singularities_tolerance_label_ord = plane_singularities_label_ord-0.4
        plane_singularities_tolerance_button_ord = plane_singularities_tolerance_label_ord+0.8
        plane_singularities_samples_label_ord = plane_singularities_label_ord-0.4
        plane_singularities_samples_button_ord = plane_singularities_label_ord+0.4
        save_obstacles_transformation_button_ord = plane_singularities_label_ord
        load_obstacles_transformation_label_ord = plane_singularities_label_ord-0.4
        load_obstacles_transformation_combobox_ord = plane_singularities_label_ord+0.4
        find_plane_singularities_button_ord = plane_singularities_label_ord
        build_obstacles_objects_label_ord = create_obstacles_2d_plane_label_ord+10
        detect_obstacles_boundaries_label_ord = build_obstacles_objects_label_ord+1.8
        load_workspace_image_label_ord = detect_obstacles_boundaries_label_ord-1
        load_workspace_image_combobox_ord = load_workspace_image_label_ord+0.8
        obstacles_height_saved_label_ord = load_workspace_image_combobox_ord+1.2
        obstacles_height_saved_button_ord = obstacles_height_saved_label_ord
        boundaries_precision_label_ord = detect_obstacles_boundaries_label_ord-1
        boundaries_precision_slider_ord = boundaries_precision_label_ord+0.8
        boundaries_vertices_lower_limit_label_ord = boundaries_precision_label_ord
        boundaries_vertices_lower_limit_slider_ord = boundaries_vertices_lower_limit_label_ord+0.8
        obstacles_height_detection_label_ord = boundaries_precision_slider_ord+1.4
        obstacles_height_detection_button_ord = obstacles_height_detection_label_ord
        detect_compute_boundaries_button_ord = detect_obstacles_boundaries_label_ord
        create_obstacles_meshes_3dprint_label_ord = build_obstacles_objects_label_ord+4.7
        load_obstacles_data_label_ord = create_obstacles_meshes_3dprint_label_ord-0.4
        load_obstacles_data_combobox_ord = load_obstacles_data_label_ord+0.8
        xy_axis_res_mesh_label_ord = load_obstacles_data_label_ord+0.1
        xy_axis_res_mesh_button_ord = xy_axis_res_mesh_label_ord
        z_axis_res_mesh_label_ord = xy_axis_res_mesh_label_ord+0.7
        z_axis_res_mesh_button_ord = z_axis_res_mesh_label_ord
        create_obstacle_mesh_stl_button_ord = create_obstacles_meshes_3dprint_label_ord-0.4
        create_all_obstacles_stl_button_ord = create_obstacles_meshes_3dprint_label_ord+0.4
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        create_obstacles_2d_plane_label_x = 1/2; gbl.menu_label(menu_frame, "Create the obstacles transformation:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], create_obstacles_2d_plane_label_x * menu_properties['width'], create_obstacles_2d_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_2d_plane_x_length_label_x = 1/7; gbl.menu_label(menu_frame, "2D plane x length (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_2d_plane_x_length_label_x * menu_properties['width'], choose_2d_plane_x_length_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_2d_plane_x_length_button_x = 2/6; self.choose_2d_plane_x_length_button = gbl.menu_button(menu_frame, f"{self.obstacles_2d_plane_x_length:.3f} m", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_2d_plane_x_length_button_x * menu_properties['width'], choose_2d_plane_x_length_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_2d_plane_x_length).button
        choose_2d_plane_y_length_label_x = 1/7; gbl.menu_label(menu_frame, "2D plane y length (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_2d_plane_y_length_label_x * menu_properties['width'], choose_2d_plane_y_length_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_2d_plane_y_length_button_x = 2/6; self.choose_2d_plane_y_length_button = gbl.menu_button(menu_frame, f"{self.obstacles_2d_plane_y_length:.3f} m", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_2d_plane_y_length_button_x * menu_properties['width'], choose_2d_plane_y_length_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_2d_plane_y_length).button
        choose_2d_plane_normal_vector_label_x = 1/8; gbl.menu_label(menu_frame, "Normal vector:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_2d_plane_normal_vector_label_x * menu_properties['width'], choose_2d_plane_normal_vector_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_2d_plane_normal_vector_button_x = 2/6; self.choose_2d_plane_normal_vector_button = gbl.menu_button(menu_frame, str([np.round(self.obstacles_2d_plane_normal_vector[k], 3) for k in range(len(self.obstacles_2d_plane_normal_vector))]), f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_2d_plane_normal_vector_button_x * menu_properties['width'], choose_2d_plane_normal_vector_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_2d_plane_normal_vector).button
        choose_2d_plane_translation_label_x = 1/8; gbl.menu_label(menu_frame, "Translation (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_2d_plane_translation_label_x * menu_properties['width'], choose_2d_plane_translation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_2d_plane_translation_button_x = 2/6; self.choose_2d_plane_translation_button = gbl.menu_button(menu_frame, str([np.round(self.obstacles_2d_plane_translation[k], 3) for k in range(len(self.obstacles_2d_plane_translation))]) + " m", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_2d_plane_translation_button_x * menu_properties['width'], choose_2d_plane_translation_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_2d_plane_translation).button
        choose_2d_plane_orientation_label_x = 1/8; gbl.menu_label(menu_frame, "Orientation (°):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_2d_plane_orientation_label_x * menu_properties['width'], choose_2d_plane_orientation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_2d_plane_orientation_button_x = 2/6; self.choose_2d_plane_orientation_button = gbl.menu_button(menu_frame, f"{self.obstacles_2d_plane_orientation:.1f} °", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_2d_plane_orientation_button_x * menu_properties['width'], choose_2d_plane_orientation_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_2d_plane_orientation).button
        z_rotate_2d_plane_label_x = 7/12; gbl.menu_label(menu_frame, "Rotation around\nz-axis (°):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], z_rotate_2d_plane_label_x * menu_properties['width'], z_rotate_2d_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.z_rotate_2d_plane_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']-3}", orient = tk.HORIZONTAL, from_ = 0, to = 360, resolution = 10**(-self.angles_precision), length = menu_properties['width'] / 6.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_2d_plane_move_sliders)
        z_rotate_2d_plane_slider_x = z_rotate_2d_plane_label_x; self.z_rotate_2d_plane_slider.place(x = z_rotate_2d_plane_slider_x * menu_properties['width'], y = z_rotate_2d_plane_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.z_rotate_2d_plane_slider.bind("<ButtonRelease-3>", self.reset_2d_plane_sliders)
        normal_translate_2d_plane_label_x = 5/6; gbl.menu_label(menu_frame, "Translation along\nnormal vector (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], normal_translate_2d_plane_label_x * menu_properties['width'], normal_translate_2d_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.normal_translate_2d_plane_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']-3}", orient = tk.HORIZONTAL, from_ = -1, to = 1, resolution = 0.001, length = menu_properties['width'] / 6.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_2d_plane_move_sliders)
        normal_translate_2d_plane_slider_x = normal_translate_2d_plane_label_x; self.normal_translate_2d_plane_slider.place(x = normal_translate_2d_plane_slider_x * menu_properties['width'], y = normal_translate_2d_plane_silder_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.normal_translate_2d_plane_slider.bind("<ButtonRelease-3>", self.reset_2d_plane_sliders)
        camera_plane_z_axis_alignment_label_x = 1/7; gbl.menu_label(menu_frame, "Camera-plane z-axis\nalignment:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], camera_plane_z_axis_alignment_label_x * menu_properties['width'], camera_plane_z_axis_alignment_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        camera_plane_z_axis_alignment_indicator_x = 2/6; self.camera_plane_z_axis_alignment_indicator = gbl.menu_label(menu_frame, "alignment", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], camera_plane_z_axis_alignment_indicator_x * menu_properties['width'], camera_plane_z_axis_alignment_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        obstacles_transformation_label_x = 7/12; gbl.menu_label(menu_frame, "Obstacles\ntransformation\n(w.r.t. world):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], obstacles_transformation_label_x * menu_properties['width'], obstacles_transformation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        obstacles_transformation_indicator_x = 5/6; self.obstacles_transformation_indicator = gbl.menu_label(menu_frame, "transformation", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], obstacles_transformation_indicator_x * menu_properties['width'], obstacles_transformation_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        ArUco_marker_pose_on_plane_label_x = 1/7; gbl.menu_label(menu_frame, "ArUco pose on plane:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], ArUco_marker_pose_on_plane_label_x * menu_properties['width'], ArUco_marker_pose_on_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        define_ArUco_marker_position_label_x = 9/24; gbl.menu_label(menu_frame, "Position:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], define_ArUco_marker_position_label_x * menu_properties['width'], define_ArUco_marker_position_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        define_ArUco_marker_position_button_x = 13/24; self.define_ArUco_marker_position_button = gbl.menu_button(menu_frame, self.ArUco_marker_position, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], define_ArUco_marker_position_button_x * menu_properties['width'], define_ArUco_marker_position_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.define_ArUco_marker_position).button
        define_ArUco_marker_orientation_label_x = 18/24; gbl.menu_label(menu_frame, "Orientation (°):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], define_ArUco_marker_orientation_label_x * menu_properties['width'], define_ArUco_marker_orientation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        define_ArUco_marker_orientation_button_x = 21/24; self.define_ArUco_marker_orientation_button = gbl.menu_button(menu_frame, f"{self.ArUco_marker_orientation:.1f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], define_ArUco_marker_orientation_button_x * menu_properties['width'], define_ArUco_marker_orientation_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.define_ArUco_marker_orientation).button
        plane_singularities_label_x = 1/8; gbl.menu_label(menu_frame, "Find singularities\non obstacles plane\n(the end-effector is\nperpendicular to it):", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], plane_singularities_label_x * menu_properties['width'], plane_singularities_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        plane_singularities_tolerance_label_x = 2/7; gbl.menu_label(menu_frame, "Tolerance:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], plane_singularities_tolerance_label_x * menu_properties['width'], plane_singularities_tolerance_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        plane_singularities_tolerance_button_x = 2/7; self.plane_singularities_tolerance_button = gbl.menu_button(menu_frame, f"{self.plane_singularities_tolerance:.3f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], plane_singularities_tolerance_button_x * menu_properties['width'], plane_singularities_tolerance_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_plane_singularities_tolerance).button
        plane_singularities_samples_label_x = 3/7; gbl.menu_label(menu_frame, "Samples:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], plane_singularities_samples_label_x * menu_properties['width'], plane_singularities_samples_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        plane_singularities_samples_button_x = 3/7; self.plane_singularities_samples_button = gbl.menu_button(menu_frame, f"{self.plane_singularities_samples}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], plane_singularities_samples_button_x * menu_properties['width'], plane_singularities_samples_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_plane_singularities_samples).button
        find_plane_singularities_button_x = 4/7; self.find_plane_singularities_button = gbl.menu_button(menu_frame, "find\nkinematic\nsingularities", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], find_plane_singularities_button_x * menu_properties['width'], find_plane_singularities_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.find_plane_singularities).button
        save_obstacles_transformation_button_x = 5/7; self.save_obstacles_transformation_button = gbl.menu_button(menu_frame, "save\nplane", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], save_obstacles_transformation_button_x * menu_properties['width'], save_obstacles_transformation_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.save_obstacles_transformation).button
        load_obstacles_transformation_label_x = 7/8; gbl.menu_label(menu_frame, "Load plane:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], load_obstacles_transformation_label_x * menu_properties['width'], load_obstacles_transformation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.load_obstacles_transformation_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']} bold", state = "normal", width = 13, values = self.saved_obstacles_transformations_list, justify = "center")
        load_obstacles_transformation_combobox_x = load_obstacles_transformation_label_x; self.load_obstacles_transformation_combobox.place(x = load_obstacles_transformation_combobox_x * menu_properties['width'], y = load_obstacles_transformation_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.load_obstacles_transformation_combobox.bind("<<ComboboxSelected>>", self.load_obstacles_transformation)
        self.load_obstacles_transformation_combobox.bind("<Return>", self.load_obstacles_transformation)
        build_obstacles_objects_label_x = 1/2; gbl.menu_label(menu_frame, "Build workspace obstacles:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], build_obstacles_objects_label_x * menu_properties['width'], build_obstacles_objects_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        detect_obstacles_boundaries_label_x = 1/8; gbl.menu_label(menu_frame, "Detect obstacles\nboundaries:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], detect_obstacles_boundaries_label_x * menu_properties['width'], detect_obstacles_boundaries_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        load_workspace_image_label_x = 11/30; gbl.menu_label(menu_frame, "Load workspace image:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], load_workspace_image_label_x * menu_properties['width'], load_workspace_image_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.load_workspace_image_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']} bold", state = "normal", width = 13, values = self.saved_workspace_images_list, justify = "center")
        load_workspace_image_combobox_x = load_workspace_image_label_x; self.load_workspace_image_combobox.place(x = load_workspace_image_combobox_x * menu_properties['width'], y = load_workspace_image_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.load_workspace_image_combobox.bind("<<ComboboxSelected>>", self.load_workspace_image_for_detection)
        self.load_workspace_image_combobox.bind("<Return>", self.load_workspace_image_for_detection)
        obstacles_height_saved_label_x = load_workspace_image_label_x-1/30; gbl.menu_label(menu_frame, "Obstacles saved\nheight (mm):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], obstacles_height_saved_label_x * menu_properties['width'], obstacles_height_saved_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        obstacles_height_saved_button_x = load_workspace_image_label_x+2/30; self.obstacles_height_saved_button = gbl.menu_button(menu_frame, f"{1000.0 * self.workspace_obstacles_height_saved:.1f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], obstacles_height_saved_button_x * menu_properties['width'], obstacles_height_saved_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_obstacles_height_saved).button
        boundaries_precision_label_x = 4/7; gbl.menu_label(menu_frame, "Precision:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], boundaries_precision_label_x * menu_properties['width'], boundaries_precision_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.boundaries_precision_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']-3}", orient = tk.HORIZONTAL, from_ = self.boundaries_precision_limits[0], to = self.boundaries_precision_limits[1], resolution = 1, length = menu_properties['width'] / 9.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_boundaries_precision)
        boundaries_precision_slider_x = boundaries_precision_label_x; self.boundaries_precision_slider.place(x = boundaries_precision_slider_x * menu_properties['width'], y = boundaries_precision_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        boundaries_vertices_lower_limit_label_x = 5/7; gbl.menu_label(menu_frame, "Vertices limit:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], boundaries_vertices_lower_limit_label_x * menu_properties['width'], boundaries_vertices_lower_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.boundaries_vertices_lower_limit_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']-3}", orient = tk.HORIZONTAL, from_ = self.boundaries_minimum_vertices_limits[0], to = self.boundaries_minimum_vertices_limits[1], resolution = 1, length = menu_properties['width'] / 9.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_boundaries_vertices_lower_limit)
        boundaries_vertices_lower_limit_slider_x = boundaries_vertices_lower_limit_label_x; self.boundaries_vertices_lower_limit_slider.place(x = boundaries_vertices_lower_limit_slider_x * menu_properties['width'], y = boundaries_vertices_lower_limit_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        obstacles_height_detection_label_x = 17/28; gbl.menu_label(menu_frame, "Obstacles height (mm)\nfor detection:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], obstacles_height_detection_label_x * menu_properties['width'], obstacles_height_detection_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        obstacles_height_detection_button_x = 21/28; self.obstacles_height_detection_button = gbl.menu_button(menu_frame, f"{1000.0 * self.workspace_obstacles_height_detection:.1f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], obstacles_height_detection_button_x * menu_properties['width'], obstacles_height_detection_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_obstacles_height_detection).button
        detect_compute_boundaries_button_x = 7/8; self.detect_compute_boundaries_button = gbl.menu_button(menu_frame, "detect\nboundaries", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], detect_compute_boundaries_button_x * menu_properties['width'], detect_compute_boundaries_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.detect_compute_boundaries_workspace).button
        create_obstacles_meshes_3dprint_label_x = detect_obstacles_boundaries_label_x; gbl.menu_label(menu_frame, "Create STL files\nfor the obstacles:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], create_obstacles_meshes_3dprint_label_x * menu_properties['width'], create_obstacles_meshes_3dprint_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        load_obstacles_data_label_x = 11/30; gbl.menu_label(menu_frame, "Load obstacle data:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], load_obstacles_data_label_x * menu_properties['width'], load_obstacles_data_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.load_obstacles_data_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']} bold", state = "normal", width = 13, values = self.chosen_workspace_saved_obstacles_objects_list, justify = "center")
        load_obstacles_data_combobox_x = load_obstacles_data_label_x; self.load_obstacles_data_combobox.place(x = load_obstacles_data_combobox_x * menu_properties['width'], y = load_obstacles_data_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.load_obstacles_data_combobox.bind("<<ComboboxSelected>>", self.load_chosen_obstacle_boundary_data)
        self.load_obstacles_data_combobox.bind("<Return>", self.load_chosen_obstacle_boundary_data)
        xy_axis_res_mesh_label_x = 4/7; gbl.menu_label(menu_frame, "xy-axis resolution:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], xy_axis_res_mesh_label_x * menu_properties['width'], xy_axis_res_mesh_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        xy_axis_res_mesh_button_x = 5/7; self.xy_axis_res_mesh_button = gbl.menu_button(menu_frame, self.xy_res_obstacle_mesh, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], xy_axis_res_mesh_button_x * menu_properties['width'], xy_axis_res_mesh_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_xy_axis_res_mesh).button
        z_axis_res_mesh_label_x = xy_axis_res_mesh_label_x; gbl.menu_label(menu_frame, "z-axis resolution:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], z_axis_res_mesh_label_x * menu_properties['width'], z_axis_res_mesh_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        z_axis_res_mesh_button_x = xy_axis_res_mesh_button_x; self.z_axis_res_mesh_button = gbl.menu_button(menu_frame, self.z_res_obstacle_mesh, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], z_axis_res_mesh_button_x * menu_properties['width'], z_axis_res_mesh_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_z_axis_resolution_mesh).button
        create_obstacle_mesh_stl_button_x = 6/7; self.create_obstacle_mesh_stl_button = gbl.menu_button(menu_frame, "create stl file", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], create_obstacle_mesh_stl_button_x * menu_properties['width'], create_obstacle_mesh_stl_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.create_obstacle_mesh_stl).button
        create_all_obstacles_stl_button_x = create_obstacle_mesh_stl_button_x; self.create_all_obstacles_stl_button = gbl.menu_button(menu_frame, "create all stl files", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], create_all_obstacles_stl_button_x * menu_properties['width'], create_all_obstacles_stl_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.create_all_obstacles_stl).button
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
        self.load_workspace_image_for_detection()  # change the chosen workspace image
    def build_solve_obstacles_avoidance_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus of the main menu that solves the obstacles avoidance
        self.clear_menus_background()
        # create the solve obstacles avoidance sub menu
        solve_obstacles_avoidance_menu_title = submenus_titles[0]
        solve_obstacles_avoidance_menu_info = f"--- {solve_obstacles_avoidance_menu_title} ---\n" + submenus_descriptions[0]
        solve_obstacles_avoidance_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = solve_obstacles_avoidance_menu_title, info = solve_obstacles_avoidance_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 1, \
                                                            rows = 17, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_obstacles_avoidance_solver_menu(self.create_static_menu_frame(solve_obstacles_avoidance_menu_properties), solve_obstacles_avoidance_menu_properties)
    def generate_obstacles_avoidance_solver_menu(self, menu_frame, menu_properties, event = None):  # build the solve obstacles avoidance menu
        # options orders
        menu_title_ord = 1
        menu_info_button_ord = 1
        load_workspace_image_obstacles_label_ord = 2
        load_workspace_image_label_ord = load_workspace_image_obstacles_label_ord+0.8
        load_workspace_obstacles_combobox_ord = load_workspace_image_label_ord+0.8
        plot_obstacles_objects_button_ord = load_workspace_image_label_ord+0.4
        define_workspace_plane_label_ord = plot_obstacles_objects_button_ord
        define_workspace_plane_button_ord = define_workspace_plane_label_ord
        show_obstacles_infos_label_ord = load_workspace_image_label_ord+2
        show_obstacles_infos_indicator_ord = show_obstacles_infos_label_ord
        define_start_target_pos_plane_label_ord = 6
        start_pos_plane_label_ord = define_start_target_pos_plane_label_ord+1
        start_pos_plane_button_ord = start_pos_plane_label_ord
        target_pos_plane_label_ord = start_pos_plane_label_ord+0.8
        target_pos_plane_button_ord = target_pos_plane_label_ord
        check_start_target_pos_label_ord = start_pos_plane_label_ord-0.2
        check_start_target_pos_indicator_ord = check_start_target_pos_label_ord+1
        wtoR2_total_transformation_label_ord = 9
        build_wtd_transformation_label_ord = wtoR2_total_transformation_label_ord+1.2
        build_wtd_transformation_button_ord = build_wtd_transformation_label_ord-0.3
        show_wtd_details_button_ord = build_wtd_transformation_label_ord+0.3
        build_dtR2_transformation_label_ord = build_wtd_transformation_label_ord
        build_dtR2_transformation_button_ord = build_dtR2_transformation_label_ord-0.3
        show_dtR2_details_button_ord = build_dtR2_transformation_label_ord+0.3
        start_transforming_button_ord = build_wtd_transformation_label_ord-0.4
        show_transformations_built_indicator_ord = start_transforming_button_ord+0.8
        compute_control_law_for_robot_label_ord = 11.5
        define_control_parameters_label_ord = compute_control_law_for_robot_label_ord+1.7
        k_d_parameter_label_ord = define_control_parameters_label_ord-0.8
        k_d_parameter_button_ord = k_d_parameter_label_ord
        k_i_parameter_label_ord = define_control_parameters_label_ord
        k_i_parameter_combobox_ord = k_i_parameter_label_ord
        w_phi_parameter_label_ord = define_control_parameters_label_ord-0.8
        w_phi_parameter_button_ord = w_phi_parameter_label_ord
        dp_min_parameter_label_ord = define_control_parameters_label_ord
        dp_min_parameter_button_ord = dp_min_parameter_label_ord
        vp_max_parameter_label_ord = define_control_parameters_label_ord-0.8
        vp_max_parameter_button_ord = vp_max_parameter_label_ord
        choose_solver_dt_label_ord = define_control_parameters_label_ord+0.8
        choose_solver_dt_button_ord = choose_solver_dt_label_ord
        choose_solver_max_iter_label_ord = define_control_parameters_label_ord+1.6
        choose_solver_max_iter_button_ord = choose_solver_max_iter_label_ord
        show_max_path_time_label_ord = define_control_parameters_label_ord+0.8
        show_max_path_time_indicator_ord = show_max_path_time_label_ord
        show_max_path_length_label_ord = define_control_parameters_label_ord+1.6
        show_max_path_length_indicator_ord = show_max_path_length_label_ord
        choose_solver_error_tol_label_ord = define_control_parameters_label_ord+0.8
        choose_solver_error_tol_button_ord = choose_solver_error_tol_label_ord
        save_control_parameters_button_ord = define_control_parameters_label_ord
        load_control_parameters_label_ord = define_control_parameters_label_ord-0.8
        load_control_parameters_combobox_ord = load_control_parameters_label_ord+0.7
        enable_error_correction_label_ord = define_control_parameters_label_ord+1.6
        enable_error_correction_button_ord = enable_error_correction_label_ord
        navigation_field_label_ord = compute_control_law_for_robot_label_ord+4.4
        show_field_values_button_ord = navigation_field_label_ord-0.3
        show_field_gradients_button_ord = navigation_field_label_ord+0.3
        field_plot_points_divs_label_ord = navigation_field_label_ord+1.3
        field_plot_points_divs_button_ord = field_plot_points_divs_label_ord
        apply_control_law_button_ord = navigation_field_label_ord
        compute_robot_trajectory_button_ord = apply_control_law_button_ord
        trajectory_speed_label_ord = compute_robot_trajectory_button_ord-0.3
        trajectory_speed_button_ord = trajectory_speed_label_ord+0.6
        move_simulated_robot_button_ord = apply_control_law_button_ord+1.3
        move_real_robot_button_ord = move_simulated_robot_button_ord
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        load_workspace_image_obstacles_label_x = 1/2; gbl.menu_label(menu_frame, "Load the desired workspace image with all the built obstacles:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], load_workspace_image_obstacles_label_x * menu_properties['width'], load_workspace_image_obstacles_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        load_workspace_image_label_x = 1/5; gbl.menu_label(menu_frame, "Workspace image:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], load_workspace_image_label_x * menu_properties['width'], load_workspace_image_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.load_workspace_obstacles_objects_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 12, values = self.saved_workspace_images_list, justify = "center")
        load_workspace_obstacles_combobox_x = load_workspace_image_label_x; self.load_workspace_obstacles_objects_combobox.place(x = load_workspace_obstacles_combobox_x * menu_properties['width'], y = load_workspace_obstacles_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.load_workspace_obstacles_objects_combobox.bind("<<ComboboxSelected>>", self.load_workspace_obstacles_infos_for_solver)
        self.load_workspace_obstacles_objects_combobox.bind("<Return>", self.load_workspace_obstacles_infos_for_solver)
        plot_obstacles_objects_button_x = 2/5; self.plot_obstacles_objects_button = gbl.menu_button(menu_frame, "plot obstacles\nboundaries", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], plot_obstacles_objects_button_x * menu_properties['width'], plot_obstacles_objects_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.plot_obstacles_objects).button
        define_workspace_plane_label_x = 4/6; gbl.menu_label(menu_frame, "Workspace plane\ntransformation:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], define_workspace_plane_label_x * menu_properties['width'], define_workspace_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        define_workspace_plane_button_x = 5/6; self.define_workspace_plane_button = gbl.menu_button(menu_frame, self.workspace_plane_creation_parameter, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], define_workspace_plane_button_x * menu_properties['width'], define_workspace_plane_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.define_workspace_plane).button
        show_obstacles_infos_label_x = 1/3; gbl.menu_label(menu_frame, "Obstacles infos:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], show_obstacles_infos_label_x * menu_properties['width'], show_obstacles_infos_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_obstacles_infos_indicator_x = 5/8; self.show_obstacles_infos_indicator = gbl.menu_label(menu_frame, self.obstacles_infos_text, f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], show_obstacles_infos_indicator_x * menu_properties['width'], show_obstacles_infos_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        define_start_target_pos_plane_label_x = 1/2; gbl.menu_label(menu_frame, "Define the start and target positions of the robot's end-effector on the workspace plane:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], define_start_target_pos_plane_label_x * menu_properties['width'], define_start_target_pos_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        start_pos_plane_label_x = 4/30; gbl.menu_label(menu_frame, "Start position (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], start_pos_plane_label_x * menu_properties['width'], start_pos_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        start_pos_plane_button_x = 10/30; self.start_pos_plane_button = gbl.menu_button(menu_frame, "start position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], start_pos_plane_button_x * menu_properties['width'], start_pos_plane_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.set_start_pos_on_plane).button
        target_pos_plane_label_x = start_pos_plane_label_x; gbl.menu_label(menu_frame, "Target position (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], target_pos_plane_label_x * menu_properties['width'], target_pos_plane_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        target_pos_plane_button_x = start_pos_plane_button_x; self.target_pos_plane_button = gbl.menu_button(menu_frame, "final position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], target_pos_plane_button_x * menu_properties['width'], target_pos_plane_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.set_target_pos_on_plane).button
        check_start_target_pos_label_x = 21/30; gbl.menu_label(menu_frame, "Check the given positions:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], check_start_target_pos_label_x * menu_properties['width'], check_start_target_pos_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        check_start_target_pos_indicator_x = check_start_target_pos_label_x; self.check_start_target_pos_indicator = gbl.menu_label(menu_frame, "positions", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], check_start_target_pos_indicator_x * menu_properties['width'], check_start_target_pos_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        wtoR2_total_transformation_label_x = 1/2; gbl.menu_label(menu_frame, "Transform the real workspace to the unit disk (radius = 1) and then back to R2 plane:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], wtoR2_total_transformation_label_x * menu_properties['width'], wtoR2_total_transformation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        start_transforming_button_x = 4/30; self.start_transforming_button = gbl.menu_button(menu_frame, "start mapping", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], start_transforming_button_x * menu_properties['width'], start_transforming_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.start_building_workspace_transformations).button
        show_transformations_built_indicator_x = start_transforming_button_x; self.show_transformations_built_indicator = gbl.menu_label(menu_frame, "transformations built", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], show_transformations_built_indicator_x * menu_properties['width'], show_transformations_built_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        build_wtd_transformation_label_x = 11/30; gbl.menu_label(menu_frame, "1. Real workspace ⭢\n⭢ Unit disk:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], build_wtd_transformation_label_x * menu_properties['width'], build_wtd_transformation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        build_wtd_transformation_button_x = 16/30; self.build_wtd_transformation_button = gbl.menu_button(menu_frame, "build", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], build_wtd_transformation_button_x * menu_properties['width'], build_wtd_transformation_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.build_realws_to_unit_disk_transformation).button
        show_wtd_details_button_x = build_wtd_transformation_button_x; self.show_wtd_details_button = gbl.menu_button(menu_frame, "interact", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_wtd_details_button_x * menu_properties['width'], show_wtd_details_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.realws_to_unit_disk_plot_interact).button
        build_dtR2_transformation_label_x = 21/30; gbl.menu_label(menu_frame, "2. Unit disk ⭢\n⭢ R2 plane:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], build_dtR2_transformation_label_x * menu_properties['width'], build_dtR2_transformation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        build_dtR2_transformation_button_x = 25/30; self.build_dtR2_transformation_button = gbl.menu_button(menu_frame, "build", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], build_dtR2_transformation_button_x * menu_properties['width'], build_dtR2_transformation_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.build_unit_disk_to_R2_transformation).button
        show_dtR2_details_button_x = build_dtR2_transformation_button_x; self.show_dtR2_details_button = gbl.menu_button(menu_frame, "interact", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_dtR2_details_button_x * menu_properties['width'], show_dtR2_details_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.unit_disk_to_R2_plot_interact).button
        compute_control_law_for_robot_label_x = 1/2; gbl.menu_label(menu_frame, "Apply the control law to compute the trajectory of the robotic manipulator:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], compute_control_law_for_robot_label_x * menu_properties['width'], compute_control_law_for_robot_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        define_control_parameters_label_x = 5/60; gbl.menu_label(menu_frame, "Control law\nparameters:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], define_control_parameters_label_x * menu_properties['width'], define_control_parameters_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        k_d_parameter_label_x = 6/30; gbl.menu_label(menu_frame, "k_d:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], k_d_parameter_label_x * menu_properties['width'], k_d_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        k_d_parameter_button_x = k_d_parameter_label_x+3/30; self.choose_k_d_parameter_button = gbl.menu_button(menu_frame, f"{self.k_d:.2f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], k_d_parameter_button_x * menu_properties['width'], k_d_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_k_d_parameter).button
        k_i_parameters_label_x = k_d_parameter_label_x; gbl.menu_label(menu_frame, "k_i:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], k_i_parameters_label_x * menu_properties['width'], k_i_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_k_i_parameters_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 6, values = [f"{self.k_i[k]:.2f}" for k in range(len(self.k_i))], justify = "center")
        k_i_parameters_combobox_x = k_d_parameter_button_x; self.choose_k_i_parameters_combobox.place(x = k_i_parameters_combobox_x * menu_properties['width'], y = k_i_parameter_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_k_i_parameters_combobox.bind("<<ComboboxSelected>>", self.change_chosen_k_i_number)
        self.choose_k_i_parameters_combobox.bind("<Return>", self.change_chosen_k_i_value)
        w_phi_parameter_label_x = 13/30; gbl.menu_label(menu_frame, "w_phi:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], w_phi_parameter_label_x * menu_properties['width'], w_phi_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        w_phi_parameter_button_x = w_phi_parameter_label_x+3/30; self.choose_w_phi_parameter_button = gbl.menu_button(menu_frame, f"{self.w_phi:.2f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], w_phi_parameter_button_x * menu_properties['width'], w_phi_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_w_phi_parameter).button
        dp_min_parameter_label_x = w_phi_parameter_label_x; gbl.menu_label(menu_frame, "dp_min (cm):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], dp_min_parameter_label_x * menu_properties['width'], dp_min_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        dp_min_parameter_button_x = w_phi_parameter_button_x; self.choose_dp_min_parameter_button = gbl.menu_button(menu_frame, f"{self.dp_min:.1f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], dp_min_parameter_button_x * menu_properties['width'], dp_min_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_dp_min_parameter).button
        vp_max_parameter_label_x = 20/30; gbl.menu_label(menu_frame, "vp_max (cm/sec):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], vp_max_parameter_label_x * menu_properties['width'], vp_max_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        vp_max_parameter_button_x = vp_max_parameter_label_x+4/30; self.choose_vp_max_parameter_button = gbl.menu_button(menu_frame, f"{self.vp_max:.1f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], vp_max_parameter_button_x * menu_properties['width'], vp_max_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_vp_max_parameter).button
        save_control_parameters_button_x = 21/30; self.save_control_parameters_button = gbl.menu_button(menu_frame, "save parameters", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], save_control_parameters_button_x * menu_properties['width'], save_control_parameters_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.save_control_law_parameters).button
        load_control_parameters_label_x = 55/60; gbl.menu_label(menu_frame, "Load param.:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], load_control_parameters_label_x * menu_properties['width'], load_control_parameters_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.load_control_parameters_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 11, values = self.saved_control_law_parameters_list, justify = "center")
        load_control_parameters_combobox_x = load_control_parameters_label_x; self.load_control_parameters_combobox.place(x = load_control_parameters_combobox_x * menu_properties['width'], y = load_control_parameters_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.load_control_parameters_combobox.bind("<<ComboboxSelected>>", self.load_control_law_parameters)
        self.load_control_parameters_combobox.bind("<Return>", self.load_control_law_parameters)
        choose_solver_dt_label_x = 6/30; gbl.menu_label(menu_frame, "dt (sec):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_solver_dt_label_x * menu_properties['width'], choose_solver_dt_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_solver_dt_button_x = choose_solver_dt_label_x+3/30; self.choose_solver_dt_button = gbl.menu_button(menu_frame, self.solver_time_step_dt, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_solver_dt_button_x * menu_properties['width'], choose_solver_dt_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_solver_dt).button 
        choose_solver_max_iter_label_x = choose_solver_dt_label_x; gbl.menu_label(menu_frame, "Max iter.:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_solver_max_iter_label_x * menu_properties['width'], choose_solver_max_iter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_solver_max_iter_button_x = choose_solver_dt_button_x; self.choose_solver_max_iter_button = gbl.menu_button(menu_frame, self.solver_maximum_iterations, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_solver_max_iter_button_x * menu_properties['width'], choose_solver_max_iter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_solver_max_iter).button
        show_max_path_time_label_x = 14/30; gbl.menu_label(menu_frame, "Max path time (sec):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], show_max_path_time_label_x * menu_properties['width'], show_max_path_time_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_max_path_time_indicator_x = show_max_path_time_label_x+5/30; self.show_max_path_time_indicator = gbl.menu_label(menu_frame, "max time", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], show_max_path_time_indicator_x * menu_properties['width'], show_max_path_time_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        show_max_path_length_label_x = show_max_path_time_label_x; gbl.menu_label(menu_frame, "Max path length (cm):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], show_max_path_length_label_x * menu_properties['width'], show_max_path_length_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_max_path_length_indicator_x = show_max_path_time_indicator_x; self.show_max_path_length_indicator = gbl.menu_label(menu_frame, "max length", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], show_max_path_length_indicator_x * menu_properties['width'], show_max_path_length_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        choose_solver_error_tol_label_x = 24/30; gbl.menu_label(menu_frame, "Error tol. (cm):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_solver_error_tol_label_x * menu_properties['width'], choose_solver_error_tol_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_solver_error_tol_button_x = choose_solver_error_tol_label_x+4/30; self.choose_solver_error_tol_button = gbl.menu_button(menu_frame, f"{self.solver_error_tolerance:.1f}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_solver_error_tol_button_x * menu_properties['width'], choose_solver_error_tol_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_solver_error_tol).button
        enable_error_correction_label_x = choose_solver_error_tol_label_x; gbl.menu_label(menu_frame, "Error correction:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], enable_error_correction_label_x * menu_properties['width'], enable_error_correction_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        enable_error_correction_button_x = choose_solver_error_tol_button_x; self.enable_error_correction_button = gbl.menu_button(menu_frame, "yes", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], enable_error_correction_button_x * menu_properties['width'], enable_error_correction_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_solver_error_correction).button
        navigation_field_label_x = 3/30; gbl.menu_label(menu_frame, "Navigation\npotential field:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], navigation_field_label_x * menu_properties['width'], navigation_field_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_field_values_button_x = 8/30; self.show_field_values_button = gbl.menu_button(menu_frame, "plot function", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_field_values_button_x * menu_properties['width'], show_field_values_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.plot_navigation_potential_field_values).button
        show_field_gradients_button_x = show_field_values_button_x; self.show_field_gradients_button = gbl.menu_button(menu_frame, "plot gradients", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_field_gradients_button_x * menu_properties['width'], show_field_gradients_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.plot_navigation_potential_field_gradients).button
        field_plot_points_divs_label_x = navigation_field_label_x; gbl.menu_label(menu_frame, "Divisions to plot:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], field_plot_points_divs_label_x * menu_properties['width'], field_plot_points_divs_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        field_plot_points_divs_button_x = show_field_values_button_x; self.navigation_field_plot_points_divs_button = gbl.menu_button(menu_frame, self.navigation_field_plot_points_divs, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], field_plot_points_divs_button_x * menu_properties['width'], field_plot_points_divs_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_field_plot_points_divs).button
        apply_control_law_button_x = 1/2; self.apply_control_law_button = gbl.menu_button(menu_frame, "apply\ncontrol law", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_control_law_button_x * menu_properties['width'], apply_control_law_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_control_law_obstacles_avoidance).button
        compute_robot_trajectory_button_x = 7/10; self.compute_robot_trajectory_button = gbl.menu_button(menu_frame, "compute robot\ntrajectory", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], compute_robot_trajectory_button_x * menu_properties['width'], compute_robot_trajectory_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.compute_robot_trajectory_obstacles_avoidance).button
        trajectory_speed_label_x = 9/10; gbl.menu_label(menu_frame, "Path speed:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], trajectory_speed_label_x * menu_properties['width'], trajectory_speed_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        trajectory_speed_button_x = trajectory_speed_label_x; self.trajectory_speed_button = gbl.menu_button(menu_frame, f"{self.trajectory_relative_speed}", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], trajectory_speed_button_x * menu_properties['width'], trajectory_speed_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_trajectory_speed).button
        move_simulated_robot_button_x = 1/2; self.move_simulated_robot_button = gbl.menu_button(menu_frame, "move simulated robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], move_simulated_robot_button_x * menu_properties['width'], move_simulated_robot_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.move_simulated_robot_obstacles_avoidance).button
        move_real_robot_button_x = 8/10; self.move_real_robot_button = gbl.menu_button(menu_frame, "move real robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], move_real_robot_button_x * menu_properties['width'], move_real_robot_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.move_real_robot_obstacles_avoidance).button
        self.update_obstacles_avoidance_solver_indicators()  # update the indicators of the obstacles avoidance solver
        self.load_workspace_obstacles_infos_for_solver()  # load the workspace obstacles infos of the chosen workspace image
        
    # functions for all the menus options
    # functions for the main menu where the robotic manipulator model is built
    def change_robotic_manipulator_model_name(self, event = None):  # change the name of the robotic manipulator model
        self.robotic_manipulator_model_name = self.model_name_entrybox.get()  # change the name of the robotic manipulator model
    def set_frames_links_number(self, joints_number, event = None):  # set the total frames and links number of the robotic manipulator
        frames_number = joints_number + 3  # set the frames number of the robotic manipulator
        links_number = joints_number + 1  # set the links number of the robotic manipulator
        return frames_number, links_number  # return the frames and links number
    def change_joints_number(self, event = None):  # change the joints number of the robotic manipulator
        if not self.robotic_manipulator_is_built:  # if there is no robotic manipulator model built
            ask_joints_number = sd.askinteger("Joints number", f"Enter the number of the robotic manipulator joints ({self.max_joints_number} maximum):", initialvalue = self.joints_number, minvalue = 1, maxvalue = self.max_joints_number, parent = self.menus_area)  # ask the user to enter the number of the robotic manipulator joints
            if ask_joints_number != None:  # if the user enters a number
                previous_joints_number = self.joints_number  # remember the previous joints number of the robotic manipulator
                self.joints_number = ask_joints_number  # set the total number of joints of the robotic manipulator
                self.frames_number, self.links_number = self.set_frames_links_number(joints_number = self.joints_number)  # set the total number of frames and links of the robotic manipulator
                if self.joints_number < previous_joints_number:  # if the new joints number is less than the previous joints number
                    self.chosen_joint_number_model = 1  # set the chosen joint number for the model to the last joint
                    self.chosen_frame_visualization = "frame 0"  # set the chosen frame for the visualization to the "frame 0" ("joint 1") frame
                    self.chosen_link_visualization = "link 1"  # set the chosen link for the visualization to the "link 1" link
                    self.chosen_joint_number_control = 1  # set the chosen joint number for the control to the first joint
                self.joints_number_update_lists_values(previous_joints_number, self.joints_number)  # update some lists values according to the new joints number of the robotic manipulator
                self.update_model_visualization_indicators()  # update the model and visualization indicators
        else:  # if a robotic manipulator model is built
            ms.showerror("Error", "It is not allowed to change the joints number of the built robotic manipulator! You have to destroy it first!", parent = self.menus_area)  # show an error message
    def joints_number_update_lists_values(self, previous_joints_number, current_joints_number):  # update some lists values according to the new joints number of the robotic manipulator
        if current_joints_number >= previous_joints_number:  # if the new joints number is greater than or equal to the previous joints number
            self.joints_types += [self.joints_types_list[0] for _ in range(current_joints_number - previous_joints_number)]  # add the new joints types to the list of the joints types of the robotic manipulator
            self.a_den_har_parameters += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new a parameters to the list of the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator
            self.d_den_har_parameters += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new d parameters to the list of the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator
            self.alpha_den_har_parameters += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new alpha parameters to the list of the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator
            self.theta_den_har_parameters += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new theta parameters to the list of the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator
            self.control_joints_variables += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new control variables to the list of the control joints variables of the robotic manipulator
            self.control_joints_variables_limits += [copy.deepcopy(self.control_joints_variables_extreme_limits) for _ in range(current_joints_number - previous_joints_number)]  # add the new control variables limits to the list of the control joints variables limits of the robotic manipulator
            self.joints_z_axis_positions += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new joints z axis positions to the list of the joints z axis positions of the robotic manipulator
            self.frames_origins_colors = list(self.frames_origins_colors[:-1]) + ["black" for _ in range(current_joints_number - previous_joints_number)] + [self.frames_origins_colors[-1]]  # add the new frames origins colors to the list of the frames origins colors of the robotic manipulator
            self.frames_origins_sizes = list(self.frames_origins_sizes[:-1]) + [self.frames_origins_sizes[-1] for _ in range(current_joints_number - previous_joints_number)] + [self.frames_origins_sizes[-1]]  # add the new frames origins sizes to the list of the frames origins sizes of the robotic manipulator
            self.initial_links_lengths = list(self.initial_links_lengths[:-1]) + [self.initial_links_lengths[-1] for _ in range(current_joints_number - previous_joints_number)] + [self.initial_links_lengths[-1]]  # add the new initial links lengths to the list of the initial links lengths of the robotic manipulator
            self.links_colors = list(self.links_colors[:-1]) + ["red" for _ in range(current_joints_number - previous_joints_number)] + [self.links_colors[-1]]  # add the new links colors to the list of the links colors of the robotic manipulator
            self.links_sizes = list(self.links_sizes[:-1]) + [self.links_sizes[-1] for _ in range(current_joints_number - previous_joints_number)] + [self.links_sizes[-1]]  # add the new links sizes to the list of the links sizes of the robotic manipulator
            self.joints_motors_list += [["X"] for _ in range(current_joints_number - previous_joints_number)]  # add the new joints motors to the list of the joints motors of the robotic manipulator
            self.joints_motors_mult_factors += [[1.0] for _ in range(current_joints_number - previous_joints_number)]  # add the new motors multiplication factors to the list of the motors multiplication factors of the robotic manipulator
            self.forward_kinematics_variables += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new forward kinematics variables to the list of the forward kinematics variables of the robotic manipulator
            self.differential_kinematics_velocities += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new differential kinematics velocities to the list of the differential kinematics velocities of the robotic manipulator
            self.diffkine_velocities_limits += [copy.deepcopy(self.diffkine_velocities_limits[0]) for _ in range(current_joints_number - previous_joints_number)]  # add the new differential kinematics velocities limits of the robotic manipulator
        else:  # if the new joints number is less than the previous joints number
            self.joints_types = copy.deepcopy(self.joints_types[:self.joints_number])  # remove the joints types that are not needed anymore
            self.a_den_har_parameters = copy.deepcopy(self.a_den_har_parameters[:self.joints_number])  # remove the a parameters that are not needed anymore
            self.d_den_har_parameters = copy.deepcopy(self.d_den_har_parameters[:self.joints_number])  # remove the d parameters that are not needed anymore
            self.alpha_den_har_parameters = copy.deepcopy(self.alpha_den_har_parameters[:self.joints_number])  # remove the alpha parameters that are not needed anymore
            self.theta_den_har_parameters = copy.deepcopy(self.theta_den_har_parameters[:self.joints_number])  # remove the theta parameters that are not needed anymore
            self.control_joints_variables = copy.deepcopy(self.control_joints_variables[:self.joints_number])  # remove the control variables that are not needed anymore
            self.control_joints_variables_limits = copy.deepcopy(self.control_joints_variables_limits[:self.joints_number])  # remove the control limits that are not needed anymore
            self.joints_z_axis_positions = copy.deepcopy(self.joints_z_axis_positions[:self.joints_number])  # remove the joints z axis positions that are not needed anymore
            self.frames_origins_colors = copy.deepcopy(self.frames_origins_colors[:(self.frames_number - 1)] + [self.frames_origins_colors[-1]])  # remove the frames origins colors that are not needed anymore
            self.frames_origins_sizes = copy.deepcopy(self.frames_origins_sizes[:(self.frames_number - 1)] + [self.frames_origins_sizes[-1]])  # remove the frames origins sizes that are not needed anymore
            self.initial_links_lengths = copy.deepcopy(self.initial_links_lengths[:(self.links_number - 1)] + [self.initial_links_lengths[-1]])  # remove the initial links lengths that are not needed anymore
            self.links_colors = copy.deepcopy(self.links_colors[:(self.links_number - 1)] + [self.links_colors[-1]])  # remove the links colors that are not needed anymore
            self.links_sizes = copy.deepcopy(self.links_sizes[:(self.links_number - 1)] + [self.links_sizes[-1]])  # remove the links sizes that are not needed anymore
            self.joints_motors_list = copy.deepcopy(self.joints_motors_list[:self.joints_number])  # remove the joints motors that are not needed anymore
            self.joints_motors_mult_factors = copy.deepcopy(self.joints_motors_mult_factors[:self.joints_number])  # remove the motors multiplication factors that are not needed anymore
            self.forward_kinematics_variables = copy.deepcopy(self.forward_kinematics_variables[:self.joints_number])  # remove the forward kinematics variables that are not needed anymore
            self.differential_kinematics_velocities = copy.deepcopy(self.differential_kinematics_velocities[:self.joints_number])  # remove the differential kinematics velocities that are not needed anymore
            self.diffkine_velocities_limits = copy.deepcopy(self.diffkine_velocities_limits[:self.joints_number])  # remove the differential kinematics velocities limits that are not needed anymore
    def change_chosen_joint_number_model(self, event = None):  # change the chosen joint number of the robotic manipulator
        self.chosen_joint_number_model = int(self.choose_joint_number_model_combobox.get().split(" ")[-1])  # change the chosen joint number of the robotic manipulator
        self.chosen_frame_visualization = f"frame {self.chosen_joint_number_model - 1}"  # change the chosen frame of the robotic manipulator for visualization
        self.chosen_link_visualization = f"link {self.chosen_joint_number_model}"  # change the chosen link of the robotic manipulator for visualization
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_chosen_joint_type(self, event = None):  # change the chosen joint type of the robotic manipulator
        self.joints_types[self.chosen_joint_number_model - 1] = self.joints_types_combobox.get()  # change the chosen joint type of the robotic manipulator
        if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1]:
            self.control_joints_variables[self.chosen_joint_number_model - 1] = self.d_den_har_parameters[self.chosen_joint_number_model - 1]  # set the current control variable to the d parameter of the chosen joint
        else:  # if the chosen joint is revolute
            self.control_joints_variables[self.chosen_joint_number_model - 1] = self.theta_den_har_parameters[self.chosen_joint_number_model - 1]  # set the current control variable to the theta parameter of the chosen joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_a_den_har_parameter(self, event = None):  # change the a parameter of the Denavit - Hartenberg parameters of the robotic manipulator
        ask_a_den_har_parameter = sd.askfloat("Define parameter a", f"Enter the parameter a for the joint {self.chosen_joint_number_model} (in meters):", initialvalue = self.a_den_har_parameters[self.chosen_joint_number_model - 1], minvalue = self.a_parameters_limits[0], maxvalue = self.a_parameters_limits[1], parent = self.menus_area)
        if ask_a_den_har_parameter != None:  # if the user enters a number
            self.a_den_har_parameters[self.chosen_joint_number_model - 1] = ask_a_den_har_parameter  # change the a parameter for the chosen model joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_d_den_har_parameter(self, event = None):  # change the d parameter of the Denavit - Hartenberg parameters of the robotic manipulator
        if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0]:  # if the chosen joint is revolute
            ask_d_den_har_parameter = sd.askfloat("Define parameter d", f"Enter the parameter d for the \nrevolute joint {self.chosen_joint_number_model} (in meters):", initialvalue = self.d_den_har_parameters[self.chosen_joint_number_model - 1], minvalue = self.d_parameters_limits[0], maxvalue = self.d_parameters_limits[1], parent = self.menus_area)
        elif self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1]:  # if the chosen joint is prismatic
            ask_d_den_har_parameter = sd.askfloat("Define variable d", f"Enter the parameter d for the \nprismatic joint {self.chosen_joint_number_model} (in meters):", initialvalue = self.d_den_har_parameters[self.chosen_joint_number_model - 1], minvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][1][0], maxvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][1][1], parent = self.menus_area)
        if ask_d_den_har_parameter != None:  # if the user enters a number
            self.d_den_har_parameters[self.chosen_joint_number_model - 1] = ask_d_den_har_parameter  # change the d parameter for the chosen model joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_alpha_den_har_parameter(self, event = None):  # change the alpha parameter of the Denavit - Hartenberg parameters of the robotic manipulator
        ask_alpha_den_har_parameter = sd.askfloat("Define parameter alpha", f"Enter the parameter alpha for the joint {self.chosen_joint_number_model} (in degrees):", initialvalue = np.rad2deg(self.alpha_den_har_parameters[self.chosen_joint_number_model - 1]), minvalue = np.rad2deg(self.alpha_parameters_limits[0]), maxvalue = np.rad2deg(self.alpha_parameters_limits[1]), parent = self.menus_area)
        if ask_alpha_den_har_parameter != None:  # if the user enters a number
            self.alpha_den_har_parameters[self.chosen_joint_number_model - 1] = np.deg2rad(ask_alpha_den_har_parameter)  # change the alpha parameter for the chosen model joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_theta_den_har_parameter(self, event = None):  # change the theta parameter of the Denavit - Hartenberg parameters of the robotic manipulator
        if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1]:  # if the chosen joint is prismatic
            ask_theta_den_har_parameter = sd.askfloat("Define parameter theta", f"Enter the parameter theta for the \nprismatic joint {self.chosen_joint_number_model} (in degrees):", initialvalue = np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]), minvalue = np.rad2deg(self.theta_parameters_limits[0]), maxvalue = np.rad2deg(self.theta_parameters_limits[1]), parent = self.menus_area)
        elif self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0]:  # if the chosen joint is revolute
            ask_theta_den_har_parameter = sd.askfloat("Define variable theta", f"Enter the parameter theta for the \nrevolute joint {self.chosen_joint_number_model} (in degrees):", initialvalue = np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]), minvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][0][0], maxvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][0][1], parent = self.menus_area)
        if ask_theta_den_har_parameter != None:  # if the user enters a number
            self.theta_den_har_parameters[self.chosen_joint_number_model - 1] = np.deg2rad(ask_theta_den_har_parameter)  # change the theta parameter for the chosen model joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_joint_variable_min_limit(self, event = None):  # change the minimum limit of the variable of the chosen joint
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_model - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the model
        ask_joint_variable_min_limit = sd.askfloat("Define variable's minimum limit", f"Enter the minimum limit of the joint {self.chosen_joint_number_model} \nvariable (in {['degrees', 'meters'][joint_type_index]}):", initialvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][0], minvalue = self.control_joints_variables_extreme_limits[joint_type_index][0], maxvalue = self.control_joints_variables_extreme_limits[joint_type_index][1], parent = self.menus_area)  # ask the user to enter the minimum limit of the joint variable
        if ask_joint_variable_min_limit != None:  # if the user enters a number
            if ask_joint_variable_min_limit <= self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][1]:  # if the minimum limit is less than or equal to the maximum limit
                self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][0] = ask_joint_variable_min_limit
                if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1] and self.d_den_har_parameters[self.chosen_joint_number_model - 1] < ask_joint_variable_min_limit:  # if the chosen joint is prismatic and the d parameter is less than the minimum limit
                    self.d_den_har_parameters[self.chosen_joint_number_model - 1] = ask_joint_variable_min_limit  # change the d parameter for the chosen model joint
                elif self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0] and np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]) < ask_joint_variable_min_limit:  # if the chosen joint is revolute and the theta parameter is less than the minimum limit
                    self.theta_den_har_parameters[self.chosen_joint_number_model - 1] = np.deg2rad(ask_joint_variable_min_limit)  # change the theta parameter for the chosen model joint
            else:  # if the minimum limit is greater than the maximum limit
                ms.showerror("Error", "The minimum limit must be less than or equal to the maximum limit.", parent = self.menus_area)  # show an error message
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_joint_variable_max_limit(self, event = None):  # change the maximum limit of the variable of the chosen joint
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_model - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the model
        ask_joint_variable_max_limit = sd.askfloat("Define variable's maximum limit", f"Enter the maximum limit of the joint {self.chosen_joint_number_model} \nvariable (in {['degrees', 'meters'][joint_type_index]}):", initialvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][1], minvalue = self.control_joints_variables_extreme_limits[joint_type_index][0], maxvalue = self.control_joints_variables_extreme_limits[joint_type_index][1], parent = self.menus_area)  # ask the user to enter the maximum limit of the joint variable
        if ask_joint_variable_max_limit != None:  # if the user enters a number
            if ask_joint_variable_max_limit >= self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][0]:  # if the maximum limit is greater than or equal to the minimum limit
                self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][1] = ask_joint_variable_max_limit
                if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1] and self.d_den_har_parameters[self.chosen_joint_number_model - 1] > ask_joint_variable_max_limit:  # if the chosen joint is prismatic and the d parameter is greater than the maximum limit
                    self.d_den_har_parameters[self.chosen_joint_number_model - 1] = ask_joint_variable_max_limit  # change the d parameter for the chosen model joint
                elif self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0] and np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]) > ask_joint_variable_max_limit:  # if the chosen joint is revolute and the theta parameter is greater than the maximum limit
                    self.theta_den_har_parameters[self.chosen_joint_number_model - 1] = np.deg2rad(ask_joint_variable_max_limit)  # change the theta parameter for the chosen model joint
            else:  # if the maximum limit is less than the minimum limit
                ms.showerror("Error", "The maximum limit must be greater than or equal to the minimum limit.", parent = self.menus_area)  # show an error message
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_base_zero_frame_positions(self, event = None):  # change the positions of the robotic manipulator base system (wrt the world frame) and zero frame (wrt the base system)
        base_position_x = sd.askfloat("Base position", "Enter the x position of the robotic manipulator base \nwrt the world frame (in meters):", initialvalue = self.base_position_wrt_world[0], minvalue = self.base_position_limits[0][0], maxvalue = self.base_position_limits[0][1], parent = self.menus_area)  # ask the user to enter the x position of the robotic manipulator base wrt the world frame
        if base_position_x != None:  # if the user enters a number
            self.base_position_wrt_world[0] = base_position_x  # change the x position of the robotic manipulator base
        base_position_y = sd.askfloat("Base position", "Enter the y position of the robotic manipulator base \nwrt the world frame (in meters):", initialvalue = self.base_position_wrt_world[1], minvalue = self.base_position_limits[1][0], maxvalue = self.base_position_limits[1][1], parent = self.menus_area)  # ask the user to enter the y position of the robotic manipulator base wrt the world frame
        if base_position_y != None:  # if the user enters a number
            self.base_position_wrt_world[1] = base_position_y  # change the y position of the robotic manipulator base
        base_position_z = sd.askfloat("Base position", "Enter the z position of the robotic manipulator base \nwrt the world frame (in meters):", initialvalue = self.base_position_wrt_world[2], minvalue = self.base_position_limits[2][0], maxvalue = self.base_position_limits[2][1], parent = self.menus_area)  # ask the user to enter the z position of the robotic manipulator base wrt the world frame
        if base_position_z != None:  # if the user enters a number
            self.base_position_wrt_world[2] = base_position_z  # change the z position of the robotic manipulator base
        zero_frame_position_x = sd.askfloat("Frame \"0\" position", "Enter the x position of frame \"0\" \nwrt the robotic manipulator base system (in meters):", initialvalue = self.zero_frame_position_wrt_base[0], minvalue = self.zero_frame_position_limits[0][0], maxvalue = self.zero_frame_position_limits[0][1], parent = self.menus_area)  # ask the user to enter the x position of the zero frame wrt the base system
        if zero_frame_position_x != None:  # if the user enters a number
            self.zero_frame_position_wrt_base[0] = zero_frame_position_x  # change the x position of the zero frame
        zero_frame_position_y = sd.askfloat("Frame \"0\" position", "Enter the y position of frame \"0\" \nwrt the robotic manipulator base system (in meters):", initialvalue = self.zero_frame_position_wrt_base[1], minvalue = self.zero_frame_position_limits[1][0], maxvalue = self.zero_frame_position_limits[1][1], parent = self.menus_area)  # ask the user to enter the y position of the zero frame wrt the base system
        if zero_frame_position_y != None:  # if the user enters a number
            self.zero_frame_position_wrt_base[1] = zero_frame_position_y  # change the y position of the zero frame
        zero_frame_position_z = sd.askfloat("Frame \"0\" position", "Enter the z position of frame \"0\" \nwrt the robotic manipulator base system (in meters):", initialvalue = self.zero_frame_position_wrt_base[2], minvalue = self.zero_frame_position_limits[2][0], maxvalue = self.zero_frame_position_limits[2][1], parent = self.menus_area)  # ask the user to enter the z position of the zero frame wrt the base system
        if zero_frame_position_z != None:  # if the user enters a number
            self.zero_frame_position_wrt_base[2] = zero_frame_position_z  # change the z position of the zero frame
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_base_zero_frame_orientations(self, event = None):  # change the orientations of the robotic manipulator base system (wrt the world frame) and zero frame (wrt the base system)
        base_x_rotation = sd.askfloat("Base orientation", "Enter the rotation of the base around world frame's x-axis (in degrees), \nthe first rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.base_orientation_wrt_world[0]), minvalue = self.base_orientation_limits[0][0], maxvalue = self.base_orientation_limits[0][1], parent = self.menus_area)  # ask the user to enter the rotation of the base around x-axis
        if base_x_rotation != None:  # if the user enters a number
            self.base_orientation_wrt_world[0] = np.deg2rad(base_x_rotation)  # change the rotation of the base around world frame's x-axis
        base_y_rotation = sd.askfloat("Base orientation", "Enter the rotation of the base around world frame's y-axis (in degrees), \nthe second rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.base_orientation_wrt_world[1]), minvalue = self.base_orientation_limits[1][0], maxvalue = self.base_orientation_limits[1][1], parent = self.menus_area)  # ask the user to enter the rotation of the base around y-axis
        if base_y_rotation != None:  # if the user enters a number
            self.base_orientation_wrt_world[1] = np.deg2rad(base_y_rotation)  # change the rotation of the base around world frame's y-axis
        base_z_rotation = sd.askfloat("Base orientation", "Enter the rotation of the base around world frame's z-axis (in degrees), \nthe third rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.base_orientation_wrt_world[2]), minvalue = self.base_orientation_limits[2][0], maxvalue = self.base_orientation_limits[2][1], parent = self.menus_area)  # ask the user to enter the rotation of the base around z-axis of the 
        if base_z_rotation != None:  # if the user enters a number
            self.base_orientation_wrt_world[2] = np.deg2rad(base_z_rotation)  # change the rotation of the base around world frame's z-axis
        zero_frame_x_rotation = sd.askfloat("Frame \"0\" orientation", "Enter the rotation of frame \"0\" around the robotic manipulator base system's x-axis (in degrees), \nthe first rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.zero_frame_orientation_wrt_base[0]), minvalue = self.zero_frame_orientation_limits[0][0], maxvalue = self.zero_frame_orientation_limits[0][1], parent = self.menus_area)  # ask the user to enter the rotation of the zero frame around x-axis
        if zero_frame_x_rotation != None:  # if the user enters a number
            self.zero_frame_orientation_wrt_base[0] = np.deg2rad(zero_frame_x_rotation)  # change the rotation of the zero frame around the robotic manipulator base system's x-axis
        zero_frame_y_rotation = sd.askfloat("Frame \"0\" orientation", "Enter the rotation of frame \"0\" around the robotic manipulator base system's y-axis (in degrees), \nthe second rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.zero_frame_orientation_wrt_base[1]), minvalue = self.zero_frame_orientation_limits[1][0], maxvalue = self.zero_frame_orientation_limits[1][1], parent = self.menus_area)  # ask the user to enter the rotation of the zero frame around y-axis
        if zero_frame_y_rotation != None:  # if the user enters a number
            self.zero_frame_orientation_wrt_base[1] = np.deg2rad(zero_frame_y_rotation)  # change the rotation of the zero frame around the robotic manipulator base system's y-axis
        zero_frame_z_rotation = sd.askfloat("Frame \"0\" orientation", "Enter the rotation of frame \"0\" around the robotic manipulator base system's z-axis (in degrees), \nthe third rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.zero_frame_orientation_wrt_base[2]), minvalue = self.zero_frame_orientation_limits[2][0], maxvalue = self.zero_frame_orientation_limits[2][1], parent = self.menus_area)  # ask the user to enter the rotation of the zero frame around z-axis
        if zero_frame_z_rotation != None:  # if the user enters a number
            self.zero_frame_orientation_wrt_base[2] = np.deg2rad(zero_frame_z_rotation)  # change the rotation of the zero frame around the robotic manipulator base system's z-axis
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_end_effector_position(self, event = None):  # change the position of the robotic manipulator end-effector system wrt the last joint frame
        end_effector_position_x = sd.askfloat("End-effector position", "Enter the x position of the robotic manipulator end-effector \nwrt the frame \"n\" (in meters):", initialvalue = self.end_effector_position_wrt_last_frame[0], minvalue = self.end_effector_position_limits[0][0], maxvalue = self.end_effector_position_limits[0][1], parent = self.menus_area)  # ask the user to enter the x position of the robotic manipulator end-effector wrt the last joint frame
        if end_effector_position_x != None:  # if the user enters a number
            self.end_effector_position_wrt_last_frame[0] = end_effector_position_x  # change the x position of the robotic manipulator end-effector wrt the last joint frame
        end_effector_position_y = sd.askfloat("End-effector position", "Enter the y position of the robotic manipulator end-effector \nwrt the frame \"n\" (in meters):", initialvalue = self.end_effector_position_wrt_last_frame[1], minvalue = self.end_effector_position_limits[1][0], maxvalue = self.end_effector_position_limits[1][1], parent = self.menus_area)  # ask the user to enter the y position of the robotic manipulator end-effector wrt the last joint frame
        if end_effector_position_y != None:  # if the user enters a number
            self.end_effector_position_wrt_last_frame[1] = end_effector_position_y  # change the y position of the robotic manipulator end-effector wrt the last joint frame
        end_effector_position_z = sd.askfloat("End-effector position", "Enter the z position of the robotic manipulator end-effector \nwrt the frame \"n\" (in meters):", initialvalue = self.end_effector_position_wrt_last_frame[2], minvalue = self.end_effector_position_limits[2][0], maxvalue = self.end_effector_position_limits[2][1], parent = self.menus_area)  # ask the user to enter the z position of the robotic manipulator end-effector wrt the last joint frame
        if end_effector_position_z != None:  # if the user enters a number
            self.end_effector_position_wrt_last_frame[2] = end_effector_position_z  # change the z position of the robotic manipulator end-effector wrt the last joint frame
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_end_effector_orientation(self, event = None):  # change the orientation of the robotic manipulator end-effector system wrt the last joint frame
        end_effector_x_rotation = sd.askfloat("end-effector orientation", "Enter the rotation of the end-effector around \"n\" frame's x-axis (in degrees), \nthe first rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.end_effector_orientation_wrt_last_frame[0]), minvalue = self.end_effector_orientation_limits[0][0], maxvalue = self.end_effector_orientation_limits[0][1], parent = self.menus_area)  # ask the user to enter the rotation of the end-effector around x-axis
        if end_effector_x_rotation != None:  # if the user enters a number
            self.end_effector_orientation_wrt_last_frame[0] = np.deg2rad(end_effector_x_rotation)  # change the rotation of the end-effector around last joint frame's x-axis
        end_effector_y_rotation = sd.askfloat("end-effector orientation", "Enter the rotation of the end-effector around \"n\" frame's y-axis (in degrees), \nthe second rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.end_effector_orientation_wrt_last_frame[1]), minvalue = self.end_effector_orientation_limits[1][0], maxvalue = self.end_effector_orientation_limits[1][1], parent = self.menus_area)  # ask the user to enter the rotation of the end-effector around y-axis
        if end_effector_y_rotation != None:  # if the user enters a number
            self.end_effector_orientation_wrt_last_frame[1] = np.deg2rad(end_effector_y_rotation)  # change the rotation of the end-effector around last joint frame's y-axis
        end_effector_z_rotation = sd.askfloat("end-effector orientation", "Enter the rotation of the end-effector around \"n\" frame's z-axis (in degrees), \nthe third rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.end_effector_orientation_wrt_last_frame[2]), minvalue = self.end_effector_orientation_limits[2][0], maxvalue = self.end_effector_orientation_limits[2][1], parent = self.menus_area)  # ask the user to enter the rotation of the end-effector around z-axis
        if end_effector_z_rotation != None:  # if the user enters a number
            self.end_effector_orientation_wrt_last_frame[2] = np.deg2rad(end_effector_z_rotation)  # change the rotation of the end-effector around last joint frame's z-axis
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def build_robotic_manipulator_model(self, event = None):  # apply the changes to the current robotic manipulator model, shown also in the visualization
        if self.robotic_manipulator_model_name == "":  # if the robotic manipulator model name is empty
            ms.showerror("Error", "You should first give a name to the robotic manipulator model, in order to build it!", parent = self.menus_area)  # show an error message
        else:  # if the robotic manipulator model name is not empty
            robotic_manipulator_joints_parameters = []  # the parameters of the robotic manipulator joints
            for joint in range(self.joints_number):  # iterate through all joints of the robotic manipulator
                if self.joints_types[joint] == self.joints_types_list[1]:  # if the current joint is prismatic
                    robotic_manipulator_joints_parameters.append(rtb.PrismaticDH(a = self.a_den_har_parameters[joint], alpha = self.alpha_den_har_parameters[joint], theta = self.theta_den_har_parameters[joint], offset = self.d_den_har_parameters[joint], qlim = self.control_joints_variables_limits[joint][1]))  # add the prismatic joint to the list of the robotic manipulator joints parameters
                else:  # if the current joint is revolute
                    robotic_manipulator_joints_parameters.append(rtb.RevoluteDH(a = self.a_den_har_parameters[joint], alpha = self.alpha_den_har_parameters[joint], d = self.d_den_har_parameters[joint], offset = self.theta_den_har_parameters[joint], qlim = np.deg2rad(self.control_joints_variables_limits[joint][0])))  # add the revolute joint to the list of the robotic manipulator joints parameters
            robot_base_T = self.get_transformation_matrix(self.base_position_wrt_world, self.base_orientation_wrt_world) * \
                            self.get_transformation_matrix(self.zero_frame_position_wrt_base, self.zero_frame_orientation_wrt_base)  # set the base of the robotic manipulator model
            robot_end_effector_T = self.get_transformation_matrix(self.end_effector_position_wrt_last_frame, self.end_effector_orientation_wrt_last_frame)  # set the end-effector of the robotic manipulator model
            self.built_robotic_manipulator = rtb.DHRobot(links = robotic_manipulator_joints_parameters, base = robot_base_T, tool = robot_end_effector_T, name = self.robotic_manipulator_model_name)  # build the robotic manipulator model
            self.built_robotic_manipulator.q = self.get_robot_joints_variables(self.control_or_kinematics_variables_visualization)  # set the current joints variables (to be visualized) to the robotic manipulator model
            self.built_robotic_manipulator_info["name"] = self.robotic_manipulator_model_name  # set the name of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_number"] = self.joints_number  # set the number of the joints of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_types"] = copy.deepcopy(self.joints_types)  # set the types of the joints of the robotic manipulator model
            self.built_robotic_manipulator_info["base_position_wrt_world"] = np.copy(self.base_position_wrt_world)  # set the position of the robotic manipulator base wrt the world frame
            self.built_robotic_manipulator_info["base_orientation_wrt_world"] = np.copy(self.base_orientation_wrt_world)  # set the orientation of the robotic manipulator base wrt the world frame
            self.built_robotic_manipulator_info["zero_frame_position_wrt_base"] = np.copy(self.zero_frame_position_wrt_base)  # set the position of the robotic manipulator zero frame wrt the base system
            self.built_robotic_manipulator_info["zero_frame_orientation_wrt_base"] = np.copy(self.zero_frame_orientation_wrt_base)  # set the orientation of the robotic manipulator zero frame wrt the base system
            self.built_robotic_manipulator_info["end_effector_position_wrt_last_frame"] = np.copy(self.end_effector_position_wrt_last_frame)  # set the position of the robotic manipulator end-effector wrt the last joint frame
            self.built_robotic_manipulator_info["end_effector_orientation_wrt_last_frame"] = np.copy(self.end_effector_orientation_wrt_last_frame)  # set the orientation of the robotic manipulator end-effector wrt the last joint frame
            self.built_robotic_manipulator_info["a_den_har_parameters"] = copy.deepcopy(self.a_den_har_parameters)  # set the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.built_robotic_manipulator_info["alpha_den_har_parameters"] = copy.deepcopy(self.alpha_den_har_parameters)  # set the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.built_robotic_manipulator_info["d_den_har_parameters"] = copy.deepcopy(self.d_den_har_parameters)  # set the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.built_robotic_manipulator_info["theta_den_har_parameters"] = copy.deepcopy(self.theta_den_har_parameters)  # set the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.built_robotic_manipulator_info["frames_origins_colors"] = copy.deepcopy(self.frames_origins_colors)  # set the colors of the frames of the robotic manipulator model
            self.built_robotic_manipulator_info["frames_origins_sizes"] = copy.deepcopy(self.frames_origins_sizes)  # set the sizes of the frames of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_z_axis_positions"] = copy.deepcopy(self.joints_z_axis_positions)  # set the z-axis positions of the joints of the robotic manipulator model
            self.built_robotic_manipulator_info["links_colors"] = copy.deepcopy(self.links_colors)  # set the colors of the links of the robotic manipulator model
            self.built_robotic_manipulator_info["links_sizes"] = copy.deepcopy(self.links_sizes)  # set the sizes of the links of the robotic manipulator model
            self.built_robotic_manipulator_info["initial_links_lengths"] = copy.deepcopy(self.initial_links_lengths)  # set the initial lengths of the links of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_motors"] = copy.deepcopy(self.joints_motors_list[:])  # set the motors of the joints of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_motors_mult_factors"] = copy.deepcopy(self.joints_motors_mult_factors[:])  # set the motors multiplication factors of the joints of the robotic manipulator model
            control_joints_variables_limits = []  # the limits of the control variables of the joints of the robotic manipulator model
            for k in range(self.joints_number):  # iterate through all joints of the robotic manipulator
                if self.joints_types[k] == self.joints_types_list[0]: control_joints_variables_limits.append(self.control_joints_variables_limits[k][0])  # if the current joint is revolute
                elif self.joints_types[k] == self.joints_types_list[1]: control_joints_variables_limits.append(self.control_joints_variables_limits[k][1])  # if the current joint is prismatic
            self.built_robotic_manipulator_info["control_joints_variables_limits"] = control_joints_variables_limits  # set the limits of the control variables of the joints of the robotic manipulator model
            self.robotic_manipulator_is_built = True  # the robotic manipulator model has been built
            self.robot_joints_control_law_output = []  # reset the robot joints control law output
            print(f"\nRobotic manipulator name: {self.robotic_manipulator_model_name}\n", self.built_robotic_manipulator)  # print the robotic manipulator model
            ms.showinfo("Robotic manipulator model built", f"The \"{self.robotic_manipulator_model_name}\" robotic manipulator model has been built successfully!", parent = self.menus_area)  # show an information message
            self.update_model_visualization_indicators()  # update the model and visualization indicators
    def reload_built_robotic_manipulator_info_data(self, event = None):  # reload the data of the built robotic manipulator model, if any
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            self.robotic_manipulator_model_name = self.built_robotic_manipulator_info["name"]  # set the name of the robotic manipulator model
            self.joints_number = self.built_robotic_manipulator_info["joints_number"]  # set the total number of joints of the robotic manipulator
            self.frames_number, self.links_number = self.set_frames_links_number(joints_number = self.joints_number)  # set the total number of frames and links of the robotic manipulator
            self.joints_types = copy.deepcopy(self.built_robotic_manipulator_info["joints_types"])  # set the types of the joints of the robotic manipulator model
            self.base_position_wrt_world = np.copy(self.built_robotic_manipulator_info["base_position_wrt_world"])  # set the position of the robotic manipulator base wrt the world frame
            self.base_orientation_wrt_world = np.copy(self.built_robotic_manipulator_info["base_orientation_wrt_world"])  # set the orientation of the robotic manipulator base wrt the world frame
            self.zero_frame_position_wrt_base = np.copy(self.built_robotic_manipulator_info["zero_frame_position_wrt_base"])  # set the position of the robotic manipulator zero frame wrt the base system
            self.zero_frame_orientation_wrt_base = np.copy(self.built_robotic_manipulator_info["zero_frame_orientation_wrt_base"])  # set the orientation of the robotic manipulator zero frame wrt the base system
            self.end_effector_position_wrt_last_frame = np.copy(self.built_robotic_manipulator_info["end_effector_position_wrt_last_frame"])  # set the position of the robotic manipulator end-effector wrt the last joint frame
            self.end_effector_orientation_wrt_last_frame = np.copy(self.built_robotic_manipulator_info["end_effector_orientation_wrt_last_frame"])  # set the orientation of the robotic manipulator end-effector wrt the last joint frame
            self.a_den_har_parameters = copy.deepcopy(self.built_robotic_manipulator_info["a_den_har_parameters"])  # set the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.alpha_den_har_parameters = copy.deepcopy(self.built_robotic_manipulator_info["alpha_den_har_parameters"])  # set the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.d_den_har_parameters = copy.deepcopy(self.built_robotic_manipulator_info["d_den_har_parameters"])  # set the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.theta_den_har_parameters = copy.deepcopy(self.built_robotic_manipulator_info["theta_den_har_parameters"])  # set the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.frames_origins_colors = copy.deepcopy(self.built_robotic_manipulator_info["frames_origins_colors"])  # set the colors of the frames of the robotic manipulator model
            self.frames_origins_sizes = copy.deepcopy(self.built_robotic_manipulator_info["frames_origins_sizes"])  # set the sizes of the frames of the robotic manipulator model
            self.joints_z_axis_positions = copy.deepcopy(self.built_robotic_manipulator_info["joints_z_axis_positions"])  # set the z-axis positions of the joints of the robotic manipulator model
            self.initial_links_lengths = copy.deepcopy(self.built_robotic_manipulator_info["initial_links_lengths"])  # set the initial lengths of the links of the robotic manipulator model
            self.links_colors = copy.deepcopy(self.built_robotic_manipulator_info["links_colors"])  # set the colors of the links of the robotic manipulator model
            self.links_sizes = copy.deepcopy(self.built_robotic_manipulator_info["links_sizes"])  # set the sizes of the links of the robotic manipulator model
            self.joints_motors_list = copy.deepcopy(self.built_robotic_manipulator_info["joints_motors"])  # set the motors of the joints of the robotic manipulator model
            self.joints_motors_mult_factors = copy.deepcopy(self.built_robotic_manipulator_info["joints_motors_mult_factors"])  # set the motors multiplication factors of the joints of the robotic manipulator model
            control_joints_variables_limits = copy.deepcopy(self.control_joints_variables_limits)  # initialize the limits of the control variables of the joints of the robotic manipulator model
            for k in range(self.joints_number):  # iterate through all joints of the robotic manipulator
                if self.joints_types[k] == self.joints_types_list[0]: control_joints_variables_limits[k][0] = copy.deepcopy(self.built_robotic_manipulator_info["control_joints_variables_limits"][k])  # if the current joint is revolute
                elif self.joints_types[k] == self.joints_types_list[1]: control_joints_variables_limits[k][1] = copy.deepcopy(self.built_robotic_manipulator_info["control_joints_variables_limits"][k])  # if the current joint is prismatic
            self.control_joints_variables_limits = control_joints_variables_limits  # set the limits of the control variables of the joints of the robotic manipulator model
    def destroy_robotic_manipulator_model(self, event = None):  # destroy the current loaded and built robotic manipulator model
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            previous_robotic_manipulator_name = self.built_robotic_manipulator_info['name']
            self.built_robotic_manipulator = None  # destroy the robotic manipulator model
            self.built_robotic_manipulator_info = {}  # clear the robotic manipulator model info
            self.robotic_manipulator_is_built = False  # no robotic manipulator model has been built
            ms.showinfo("Robotic manipulator model destroyed", f"The \"{previous_robotic_manipulator_name}\" robotic manipulator model has been destroyed successfully!", parent = self.menus_area)  # show an information message
    def show_robotic_manipulator_model_info(self, event = None):  # show to the user the current robotic manipulator model
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            ms.showinfo("Robotic manipulator model info (in SI units where is needed)", \
                f"Name: {self.built_robotic_manipulator_info['name']}\n\
--- Cordinates systems ---\n\
• Base position wrt world (in m): {self.built_robotic_manipulator_info['base_position_wrt_world']}\nBase orientation (in degrees) wrt world (roll-pitch-yaw): {np.rad2deg(self.built_robotic_manipulator_info['base_orientation_wrt_world'])}\n\
• Frame \"0\" position wrt base (in m): {self.built_robotic_manipulator_info['zero_frame_position_wrt_base']}\nFrame \"0\" orientation (in degrees) wrt base (roll-pitch-yaw): {np.rad2deg(self.built_robotic_manipulator_info['zero_frame_orientation_wrt_base'])}\n\
• End-effector position wrt frame \"n\" (in m): {self.built_robotic_manipulator_info['end_effector_position_wrt_last_frame']}\nEnd-effector orientation (in degrees) wrt frame \"n\" (roll-pitch-yaw): {np.rad2deg(self.built_robotic_manipulator_info['end_effector_orientation_wrt_last_frame'])}\n\
--- Joints and frames parameters ---\n\
• Joints number: {self.built_robotic_manipulator_info['joints_number']}\n\
• Joints types: {self.built_robotic_manipulator_info['joints_types']}\n\
• Joints a parameters (in m): {self.built_robotic_manipulator_info['a_den_har_parameters']}\n\
• Joints alpha parameters (in degrees): {np.rad2deg(self.built_robotic_manipulator_info['alpha_den_har_parameters'])}\n\
• Joints d parameters (in m): {self.built_robotic_manipulator_info['d_den_har_parameters']}\n\
• Joints theta parameters (in degrees): {np.rad2deg(self.built_robotic_manipulator_info['theta_den_har_parameters'])}\n\
• Joints frames z-axis positions (in m): {self.built_robotic_manipulator_info['joints_z_axis_positions']}\n\
• Frames origins colors: {self.built_robotic_manipulator_info['frames_origins_colors']}\n\
• Frames origins sizes: {self.built_robotic_manipulator_info['frames_origins_sizes']}\n\
--- Links parameters ---\n\
• Links colors: {self.built_robotic_manipulator_info['links_colors']}\n\
• Links sizes: {self.built_robotic_manipulator_info['links_sizes']}\n\
• Links lengths initial (in m): {self.built_robotic_manipulator_info['initial_links_lengths']}\n\
--- Control parameters ---\n\
• Joints control variables limits (in degrees): {self.built_robotic_manipulator_info['control_joints_variables_limits']}\n\
• Joints motors: {self.built_robotic_manipulator_info['joints_motors']}\n\
• Joints motors multiplication factors: {self.built_robotic_manipulator_info['joints_motors_mult_factors']}", parent = self.menus_area)  # show the robotic manipulator model info
        else:  # if the robotic manipulator model has not been built
            ms.showerror("Error", "There is no robotic manipulator model built yet!", parent = self.menus_area)  # show an error message
    def save_robotic_manipulator_model_file(self, event = None):  # save the current robotic manipulator model
        if self.robotic_manipulator_model_name == "":  # if the robotic manipulator model name is empty
            ms.showerror("Error", "You should first give a name to the robotic manipulator model, in order to save it!", parent = self.menus_area)  # show an error message
        else:  # if the robotic manipulator model name is not empty
            ask_file_name = sd.askstring("Save robotic manipulator model", f"Enter the name of the file to save the robotic manipulator model (the model name -\"{self.robotic_manipulator_model_name}\"- \nyou specified will remain as it is right now, regardless of the file's name):", initialvalue = self.robotic_manipulator_model_name, parent = self.menus_area)  # ask the user to enter the name of the file to save the robotic manipulator model
            if ask_file_name != None and ask_file_name != "":  # if the user enters a name for the file
                overwrite_file_accept = False  # the user's choice to overwrite the file
                if (ask_file_name + ".txt") in os.listdir(self.saved_robotic_manipulators_folder_path):  # if there is already a file named exactly like this
                    overwrite_file_accept = ms.askyesno("Asking permission to overwrite file", "There is already a file named exactly like this. Do you want to overwrite it?")  # ask the user if they want to overwrite the file
                if (ask_file_name + ".txt") not in os.listdir(self.saved_robotic_manipulators_folder_path) or overwrite_file_accept:  # if the file does not exist or the user wants to overwrite it
                    model_file = open(self.saved_robotic_manipulators_folder_path + fr"/{ask_file_name}.txt", "w", encoding = "utf-8")  # the path of the file where the robotic manipulator model will be saved
                    model_file.write(f"Name: {self.robotic_manipulator_model_name}\n")  # write the name of the robotic manipulator model
                    model_file.write(f"Joints number: {self.joints_number}\n")  # write the number of the joints of the robotic manipulator model
                    model_file.write(f"Base position wrt world: {self.base_position_wrt_world[0]:.3f} {self.base_position_wrt_world[1]:.3f} {self.base_position_wrt_world[2]:.3f}\n")  # write the position of the robotic manipulator base wrt the world frame
                    model_file.write(f"Base orientation wrt world: {np.rad2deg(self.base_orientation_wrt_world[0]):.3f} {np.rad2deg(self.base_orientation_wrt_world[1]):.3f} {np.rad2deg(self.base_orientation_wrt_world[2]):.3f}\n")  # write the orientation of the robotic manipulator base wrt the world frame
                    model_file.write(f"Frame \"0\" position wrt base: {self.zero_frame_position_wrt_base[0]:.3f} {self.zero_frame_position_wrt_base[1]:.3f} {self.zero_frame_position_wrt_base[2]:.3f}\n")  # write the position of the robotic manipulator zero frame wrt the base system
                    model_file.write(f"Frame \"0\" orientation wrt base: {np.rad2deg(self.zero_frame_orientation_wrt_base[0]):.3f} {np.rad2deg(self.zero_frame_orientation_wrt_base[1]):.3f} {np.rad2deg(self.zero_frame_orientation_wrt_base[2]):.3f}\n")  # write the orientation of the robotic manipulator zero frame wrt the base system
                    model_file.write(f"End-effector position wrt frame \"n\": {self.end_effector_position_wrt_last_frame[0]:.3f} {self.end_effector_position_wrt_last_frame[1]:.3f} {self.end_effector_position_wrt_last_frame[2]:.3f}\n")  # write the position of the robotic manipulator end-effector wrt the last joint frame
                    model_file.write(f"End-effector orientation wrt frame \"n\": {np.rad2deg(self.end_effector_orientation_wrt_last_frame[0]):.3f} {np.rad2deg(self.end_effector_orientation_wrt_last_frame[1]):.3f} {np.rad2deg(self.end_effector_orientation_wrt_last_frame[2]):.3f}\n")  # write the orientation of the robotic manipulator end-effector wrt the last joint frame
                    for k in range(self.joints_number):  # iterate through all the joints of the robotic manipulator
                        joint_type_index = self.joints_types_list.index(self.joints_types[k])  # the index of the current joint type (0 for revolute and 1 for prismatic) for the model
                        joint_motors = ", ".join(self.joints_motors_list[k])  # the motors of the current joint
                        joint_motors_mult_factors = ", ".join([str(float(num)) for num in self.joints_motors_mult_factors[k]])  # the multiplication factors of the motors of the current joint
                        model_file.write(f"Joint {k + 1} type: {self.joints_types[k]}\n")  # write the type of the current joint
                        model_file.write(f"Joint {k + 1} Denavit - Hartenberg parameters: {self.a_den_har_parameters[k]:.3f} {np.rad2deg(self.alpha_den_har_parameters[k]):.3f} {self.d_den_har_parameters[k]:.3f} {np.rad2deg(self.theta_den_har_parameters[k]):.3f}\n")  # write the Denavit - Hartenberg parameters of the current joint
                        model_file.write(f"Joint {k + 1} variable limits: {self.control_joints_variables_limits[k][joint_type_index][0]:.3f} {self.control_joints_variables_limits[k][joint_type_index][1]:.3f}\n")  # write the variable limits of the current joint
                        model_file.write(f"Joint {k + 1} motors: {joint_motors}\n")  # write the motors of the current joint
                        model_file.write(f"Joint {k + 1} motors mult. factors: {joint_motors_mult_factors}\n")  # write the multiplication factors of the motors of the current joint
                        model_file.write(f"Joint {k + 1} z-axis position: {self.joints_z_axis_positions[k]:.3f}\n")  # write the z-axis position of the current joint (along the z-axis of its frame)
                    for k in range(self.frames_number):  # itearate through all the frames of the robotic manipulator
                        frame = (["base"] + [f"{i}" for i in range(self.links_number)] + ["end-effector"])[k]  # the name of the current frame
                        model_file.write(f"Frame \"{frame}\" origin color: {self.frames_origins_colors[k]}\n")  # write the color of the current frame origin
                        model_file.write(f"Frame \"{frame}\" origin size: {self.frames_origins_sizes[k]}\n")  # write the size of the current frame origin
                        if frame != f"{self.joints_number}" and frame != "end-effector":  # if the current frame is not the "n" frame nor the end-effector
                            model_file.write(f"Frame \"{frame}\" joint's link color: {self.links_colors[k]}\n")  # write the color of the current link
                            model_file.write(f"Frame \"{frame}\" joint's link size: {self.links_sizes[k]}\n")  # write the size of the current link
                            model_file.write(f"Frame \"{frame}\" initial joint's link length: {self.initial_links_lengths[k]:.3f}\n")  # write the initial length of the current link
                    model_file.close()  # close the file where the robotic manipulator model is saved
                    if ask_file_name not in self.saved_robots_models_files_list:  # if the name of the saved file is not in the values of the combobox that contains the names of the saved files
                        self.saved_robots_models_files_list.append(ask_file_name)  # add the name of the saved file to the list of the saved files
                    ms.showinfo("Robot model saved!", f"The robotic manipulator model has been saved successfully in the file \"{ask_file_name}.txt\"!", parent = self.menus_area)  # show an information message
                    self.update_model_visualization_indicators()  # update the model and visualization indicators
                else:  # if the file exists and the user does not want to overwrite it
                    ms.showinfo("File not saved", "The file has not been saved!", parent = self.menus_area)  # show an information message
            else:  # if the user does not enter a name for the file
                ms.showerror("Error", "You have not entered a name for the file!", parent = self.menus_area)  # show an error message
    def delete_robotic_manipulator_model_file(self, event = None):  # delete a saved robotic manipulator model
        file_to_delete = sd.askstring("Delete robotic manipulator model", "Enter the name of the file to delete the robotic manipulator model:", parent = self.menus_area)  # ask the user to enter the name of the file to delete the robotic manipulator model
        if file_to_delete != None and file_to_delete != "":  # if the user enters a name for the file
            if file_to_delete in self.saved_robots_models_files_list:  # if the chosen file exists
                delete_file_accept = ms.askyesno("Confirm deletion", f"Are you sure you want to delete the file \"{file_to_delete}.txt\"?")  # ask the user if they want to delete the file
                if delete_file_accept:  # if the user wants to delete the file
                    os.remove(self.saved_robotic_manipulators_folder_path + fr"/{file_to_delete}.txt")  # delete the file
                    self.saved_robots_models_files_list.remove(file_to_delete)  # remove the name of the deleted file from the list of the saved files
                    ms.showinfo("File deleted!", f"The file \"{file_to_delete}.txt\" has been deleted successfully!", parent = self.menus_area)  # show an information message
                    self.update_model_visualization_indicators()  # update the model and visualization indicators
            else:  # if the file does not exist
                ms.showerror("Error", "The file does not exist!", parent = self.menus_area)  # show an error message
        else:  # if the user does not enter a name for the file
            ms.showerror("Error", "You have not entered a name for the file!", parent = self.menus_area)  # show an error message
    def load_robotic_manipulator_model(self, robot_model_name, event = None):  # load a saved robotic manipulator model
        load_file_accept = False  # the user's choice to load the file
        if event != None:  # if the function is called by an event
            loaded_robot_file = self.load_model_combobox.get()  # the name of the chosen loaded file that contains the robotic manipulator model
        else:
            loaded_robot_file = robot_model_name  # the name of the chosen loaded file that contains the robotic manipulator model
        if loaded_robot_file in self.saved_robots_models_files_list:  # if the chosen file exists
            if event != None:  # if the function is called by an event
                load_file_accept = ms.askyesno("Confirm loading", f"Are you sure you want to load the file \"{loaded_robot_file}.txt\"?")  # ask the user if they want to load the chosen file
            else:
                load_file_accept = True  # the file is loaded automatically
            if load_file_accept:  # if the user wants to load the chosen file
                self.destroy_robotic_manipulator_model()  # destroy the current built robotic manipulator model
                model_file = open(self.saved_robotic_manipulators_folder_path + fr"/{loaded_robot_file}.txt", "r", encoding = "utf-8")  # open the file that contains the robotic manipulator model
                model_file_lines = model_file.readlines()  # read all the lines of the file
                model_file.close()  # close the file
                self.robotic_manipulator_model_name = model_file_lines[0].split(": ")[1].strip()  # the name of the robotic manipulator model
                self.joints_number = int(model_file_lines[1].split(": ")[1].strip())  # set the total number of joints
                systems_first_line_index = 2  # the index of the first line of the base and end-effector systems in the file
                self.frames_number, self.links_number = self.set_frames_links_number(joints_number = self.joints_number)  # set the total number of frames and links of the robotic manipulator
                self.base_position_wrt_world = np.array([float(value) for value in model_file_lines[systems_first_line_index].split(": ")[1].strip().split(" ")])  # the position of the robotic manipulator base
                self.base_orientation_wrt_world = np.array([np.deg2rad(float(value)) for value in model_file_lines[systems_first_line_index + 1].split(": ")[1].strip().split(" ")])  # the orientation of the robotic manipulator base
                self.zero_frame_position_wrt_base = np.array([float(value) for value in model_file_lines[systems_first_line_index + 2].split(": ")[1].strip().split(" ")])  # the position of the robotic manipulator zero frame
                self.zero_frame_orientation_wrt_base = np.array([np.deg2rad(float(value)) for value in model_file_lines[systems_first_line_index + 3].split(": ")[1].strip().split(" ")])  # the orientation of the robotic manipulator zero frame
                self.end_effector_position_wrt_last_frame = np.array([float(value) for value in model_file_lines[systems_first_line_index + 4].split(": ")[1].strip().split(" ")])  # the position of the robotic manipulator end-effector
                self.end_effector_orientation_wrt_last_frame = np.array([np.deg2rad(float(value)) for value in model_file_lines[systems_first_line_index + 5].split(": ")[1].strip().split(" ")])  # the orientation of the robotic manipulator end-effector
                self.joints_types = []  # the types of the joints of the robotic manipulator model
                self.a_den_har_parameters = []  # the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                self.alpha_den_har_parameters = []  # the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                self.d_den_har_parameters = []  # the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                self.theta_den_har_parameters = []  # the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                self.joints_z_axis_positions = []  # the z-axis positions of the joints of the robotic manipulator model
                self.joints_motors_list = []  # the motors of the joints of the robotic manipulator model
                self.joints_motors_mult_factors = []  # the multiplication factors of the motors of the joints of the robotic manipulator model
                self.control_joints_variables_limits = [copy.deepcopy(self.control_joints_variables_extreme_limits) for _ in range(self.joints_number)]  # the limits of the control joints variables in degrees or meters, depending on the joint's type
                joints_first_line_index = 8  # the index of the first line of the joints in the file
                lines_number_for_each_joint = 6  # the number of lines for each joint separately in the file
                for k in range(self.joints_number):  # iterate through all joints of the robotic manipulator model
                    self.joints_types.append(model_file_lines[joints_first_line_index + lines_number_for_each_joint * k].split(": ")[1].strip())  # the type of the current joint
                    self.a_den_har_parameters.append(float(model_file_lines[(joints_first_line_index + 1) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")[0]))  # the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                    self.alpha_den_har_parameters.append(np.deg2rad(float(model_file_lines[(joints_first_line_index + 1) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")[1])))  # the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                    self.d_den_har_parameters.append(float(model_file_lines[(joints_first_line_index + 1) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")[2]))  # the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                    self.theta_den_har_parameters.append(np.deg2rad(float(model_file_lines[(joints_first_line_index + 1) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")[3])))  # the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                    self.control_joints_variables_limits[k][self.joints_types_list.index(self.joints_types[k])] = [float(value) for value in model_file_lines[(joints_first_line_index + 2) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")]  # the limits of the variables of the current joint
                    self.joints_motors_list.append(model_file_lines[(joints_first_line_index + 3) + lines_number_for_each_joint * k].split(": ")[1].strip().split(", "))  # the motors of the current joint
                    self.joints_motors_mult_factors.append([float(num) for num in model_file_lines[(joints_first_line_index + 4) + lines_number_for_each_joint * k].split(": ")[1].strip().split(", ")])  # the multiplication factors of the motors of the current joint
                    self.joints_z_axis_positions.append(float(model_file_lines[(joints_first_line_index + 5) + lines_number_for_each_joint * k].split(": ")[1].strip()))  # the z-axis position of the current joint (along the z-axis of its frame)
                self.frames_origins_colors = []  # the colors of the frames origins of the robotic manipulator model
                self.frames_origins_sizes = []  # the sizes of the frames origins of the robotic manipulator model
                self.links_colors = []  # the colors of the links of the robotic manipulator model
                self.links_sizes = []  # the sizes of the links of the robotic manipulator model
                self.initial_links_lengths = []  # the initial lengths of the links of the robotic manipulator model
                frames_line_index = joints_first_line_index + self.joints_number * lines_number_for_each_joint  # the index of the first line of the frames in the file
                lines_number_for_each_frame = 5  # the number of lines for each frame separately in the file
                for k in range(self.links_number):  # iterate through the first frames (base and joints frames) of the robotic manipulator
                    self.frames_origins_colors.append(model_file_lines[frames_line_index + lines_number_for_each_frame * k].split(": ")[1].strip())  # the color of the current frame origin
                    self.frames_origins_sizes.append(int(model_file_lines[frames_line_index + lines_number_for_each_frame * k + 1].split(": ")[1].strip()))  # the size of the current frame origin
                    self.links_colors.append(model_file_lines[frames_line_index + lines_number_for_each_frame * k + 2].split(": ")[1].strip())  # the color of the current link
                    self.links_sizes.append(int(model_file_lines[frames_line_index + lines_number_for_each_frame * k + 3].split(": ")[1].strip()))  # the size of the current link
                    self.initial_links_lengths.append(float(model_file_lines[frames_line_index + lines_number_for_each_frame * k + 4].split(": ")[1].strip()))  # the initial length of the current link
                for k in range(2):  # iterate through the last frames ("n" frame end-effector frame) of the robotic manipulator
                    self.frames_origins_colors.append(model_file_lines[frames_line_index + lines_number_for_each_frame * self.links_number + 2*k].split(": ")[1].strip())  # the color of the current frame origin
                    self.frames_origins_sizes.append(int(model_file_lines[frames_line_index + lines_number_for_each_frame * self.links_number + 2*k+1].split(": ")[1].strip()))  # the size of the current frame origin
                self.control_joints_variables = [0.0 for _ in range(self.joints_number)]  # the variables of the joints of the robotic manipulator model
                self.forward_kinematics_variables = [0.0 for _ in range(self.joints_number)]  # the variables of the forward kinematics of the robotic manipulator model
                self.differential_kinematics_velocities = [0.0 for _ in range(self.joints_number)]  # the velocities of the differential kinematics of the robotic manipulator model
                self.chosen_joint_number_model = 1  # the chosen joint number of the robotic manipulator model
                self.chosen_frame_visualization = "frame 0"  # the chosen frame of the robotic manipulator for visualization
                self.chosen_link_visualization = "link 1"  # the chosen link of the robotic manipulator for visualization
                self.chosen_joint_number_control = 1  # the chosen joint number of the robotic manipulator control
                self.model_name_entrybox.delete(0, "end")  # delete the current name of the robotic manipulator model
                self.model_name_entrybox.insert(0, self.robotic_manipulator_model_name)  # insert the name of the robotic manipulator model
                self.build_robotic_manipulator_model()  # build the robotic manipulator model
        else:  # if the chosen file does not exist
            ms.showerror("Error", f"The chosen robot model \"{loaded_robot_file}.txt\" does not exist!", parent = self.menus_area)  # show an error message
    # functions for the main menu where the robotic manipulator is visualized
    def get_number_from_frame_visualization(self, frame_visualization):  # get the number from the frame visualization
        if frame_visualization == "base": return 0  # if the chosen frame is the base frame
        elif frame_visualization == "end-effector": return self.joints_number + 2  # if the chosen frame is the end-effector frame
        else: return int(frame_visualization.split(" ")[-1]) + 1  # if the chosen frame is a joint frame
    def get_number_from_link_visualization(self, link_visualization):  # get the number from the link visualization
        if link_visualization == "base": return 0  # if the chosen link is the base link
        else: return int(link_visualization.split(" ")[-1])  # if the chosen link is not the base link
    def change_chosen_frame_visualization(self, event = None):  # change the chosen joint number of the robotic manipulator visualization
        self.chosen_frame_visualization = self.choose_frame_visualization_combobox.get()  # change the chosen frame of the robotic manipulator for visualization
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_joint_position(self, event = None):  # change the position of the chosen joint of the robotic manipulator
        if self.chosen_frame_visualization != "base" and self.chosen_frame_visualization != "end-effector" and self.chosen_frame_visualization != f"frame {self.joints_number}":  # if the chosen frame is a joint frame
            chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
            joint_z_axis_position = sd.askfloat("Choose the joint position offset from its frame origin", f"Enter the z-axis position of the joint {chosen_frame_number_visualization} \n(in meters) wrt its frame (\"{self.chosen_frame_visualization}\" frame):", initialvalue = self.joints_z_axis_positions[chosen_frame_number_visualization - 1], minvalue = -self.d_parameters_limits[1], maxvalue = self.d_parameters_limits[1], parent = self.menus_area)
            if joint_z_axis_position != None:  # if the user enters a number
                self.joints_z_axis_positions[chosen_frame_number_visualization - 1] = joint_z_axis_position  # change the position offset of the chosen joint of the robotic manipulator
        else:
            ms.showerror("Error", f"This frame is not related to any joint! Choose one of the frames 0 to {self.joints_number-1}!", parent = self.menus_area)  # show an error message
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_frame_color(self, event = None):  # change the color of the chosen frame origin of the robotic manipulator
        chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
        self.frames_origins_colors[chosen_frame_number_visualization] = cc.askcolor(title = f"Choose the \"{self.chosen_frame_visualization}\" frame origin color", color = self.frames_origins_colors[chosen_frame_number_visualization], parent = self.menus_area)[1]  # change the color of the chosen frame origin of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def apply_color_to_all_frames(self, event = None):  # apply the chosen color to all the frames origins of the robotic manipulator
        chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
        for k in range(self.frames_number):  # iterate through all the frames origins of the robotic manipulator
            self.frames_origins_colors[k] = self.frames_origins_colors[chosen_frame_number_visualization]  # apply the chosen color to all the frames origins
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_frame_size(self, event = None):  # change the size of the chosen frame origin of the robotic manipulator
        chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
        frame_origin_size = sd.askinteger("Choose the frame origin width", f"Enter the relative size of the \"{self.chosen_frame_visualization}\" frame origin\n({self.min_visualization_size} for minimum and {self.max_visualization_size} for maximum):", initialvalue = self.frames_origins_sizes[chosen_frame_number_visualization], minvalue = self.min_visualization_size, maxvalue = self.max_visualization_size, parent = self.menus_area)
        if frame_origin_size != None:  # if the user enters a number
            self.frames_origins_sizes[chosen_frame_number_visualization] = frame_origin_size  # change the size of the chosen frame origin of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def apply_size_to_all_frames(self, event = None):  # apply the chosen size to all the frames origins of the robotic manipulator
        chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
        for k in range(self.frames_number):  # iterate through all the frames origins of the robotic manipulator
            self.frames_origins_sizes[k] = self.frames_origins_sizes[chosen_frame_number_visualization]  # apply the chosen size to all the frames origins
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_chosen_link_visualization(self, event = None):  # change the chosen link of the robotic manipulator for visualization
        self.chosen_link_visualization = self.choose_link_visualization_combobox.get()  # change the chosen link of the robotic manipulator for visualization
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_link_color(self, event = None):  # change the color of the chosen link of the robotic manipulator
        chosen_link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen frame number
        self.links_colors[chosen_link_number_visualization] = cc.askcolor(title = f"Choose the \"{self.chosen_link_visualization}\" link color", color = self.links_colors[chosen_link_number_visualization], parent = self.menus_area)[1]  # change the color of the chosen link of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def apply_color_to_all_links(self, event = None):  # apply the chosen color to all links of the robotic manipulator
        chosen_link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen frame number
        for k in range(self.links_number):  # iterate through all links of the robotic manipulator
            self.links_colors[k] = self.links_colors[chosen_link_number_visualization]  # apply the chosen color to all the links
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_link_size(self, event = None):  # change the size of the chosen link of the robotic manipulator
        chosen_link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen frame number
        link_size = sd.askinteger("Choose the link width", f"Enter the relative size of the \"{self.chosen_link_visualization}\" link \n({self.min_visualization_size} for minimum and {self.max_visualization_size} for maximum):", initialvalue = self.links_sizes[chosen_link_number_visualization], minvalue = self.min_visualization_size, maxvalue = self.max_visualization_size, parent = self.menus_area)
        if link_size != None:  # if the user enters a number
            self.links_sizes[chosen_link_number_visualization] = link_size  # change the size of the chosen link of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def apply_size_to_all_links(self, event = None):  # apply the chosen size to all links of the robotic manipulator
        chosen_link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen frame number
        for k in range(self.links_number):  # iterate through all links of the robotic manipulator
            self.links_sizes[k] = self.links_sizes[chosen_link_number_visualization]  # apply the chosen size to all the links
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_canvas_color(self, event = None):  # change the color of the canvas where the robotic manipulator is visualized
        self.workspace_canvas_color = cc.askcolor(title = "Choose the canvas color", color = self.workspace_canvas_color, parent = self.menus_area)[1]
        self.workspace_canvas.configure(bg = self.workspace_canvas_color)  # change the color of the canvas where the robotic manipulator is visualized
        self.choose_visualized_variables_button.configure(bg = self.workspace_canvas_color)  # change the bg color of the button that allows the user to choose the visualized variables
        self.visualized_variables_values_indicator.configure(bg = self.workspace_canvas_color)  # change the bg color of the indicator that shows the values of the visualized variables
    def change_terrain_color(self, event = None):  # change the color of the terrain of the workspace
        self.up_side_terrain_color = cc.askcolor(title = "Choose the terrain up side color", color = self.up_side_terrain_color, parent = self.menus_area)[1]
        self.down_side_terrain_color = cc.askcolor(title = "Choose the terrain down side color", color = self.down_side_terrain_color, parent = self.menus_area)[1]
    def change_axis_colors(self, event = None):  # change the color of the axis of the workspace
        self.workspace_axis_colors[0] = cc.askcolor(title = "Choose the x-axis color", color = self.workspace_axis_colors[0], parent = self.menus_area)[1]
        self.workspace_axis_colors[1] = cc.askcolor(title = "Choose the y-axis color", color = self.workspace_axis_colors[1], parent = self.menus_area)[1]
        self.workspace_axis_colors[2] = cc.askcolor(title = "Choose the z-axis color", color = self.workspace_axis_colors[2], parent = self.menus_area)[1]
    def change_axis_size(self, event = None):  # change the size of the axis of the workspace
        axis_sizes = sd.askinteger("Choose the axis size", f"Enter the relative size of the axis \n({self.min_visualization_size} for minimum and {self.max_visualization_size} for maximum):", initialvalue = self.workspace_axis_sizes, minvalue = self.min_visualization_size, maxvalue = self.max_visualization_size, parent = self.menus_area)  # ask the user to enter the size of the axis
        if axis_sizes != None:  # if the user enters a number
            self.workspace_axis_sizes = axis_sizes  # change the size of the axis of the workspace
    def change_obstacles_2d_plane_color(self, event = None):  # change the color of the 2D plane of the workspace, where the workspace obstacles are located
        self.front_side_2d_plane_color = cc.askcolor(title = "Choose the 2D plane front side color", color = self.front_side_2d_plane_color, parent = self.menus_area)[1]
        self.back_side_2d_plane_color = cc.askcolor(title = "Choose the 2D plane back side color", color = self.back_side_2d_plane_color, parent = self.menus_area)[1]
    def change_obstacles_2d_plane_size(self, event = None):  # change the size of the 2D plane of the workspace, where the workspace obstacles are located
        obstacles_2d_plane_size = sd.askinteger("Choose the 2D plane size", f"Enter the relative size of the 2D plane \n({self.min_visualization_size} for minimum and {self.max_visualization_size} for maximum):", initialvalue = self.workspace_2d_plane_size, minvalue = self.min_visualization_size, maxvalue = self.max_visualization_size, parent = self.menus_area)  # ask the user to enter the size of the 2D plane
        if obstacles_2d_plane_size != None:  # if the user enters a number
            self.workspace_2d_plane_size = obstacles_2d_plane_size
    def change_obstacles_objects_color(self, event = None):  # change the color of the obstacles of the workspace
        self.workspace_obstacles_color = cc.askcolor(title = "Choose the obstacles color", color = self.workspace_obstacles_color, parent = self.menus_area)[1]
    def change_obstacles_objects_size(self, event = None):  # change the size of the obstacles of the workspace
        obstacles_objects_size = sd.askinteger("Choose the obstacles size", f"Enter the relative size of the obstacles \n({self.min_visualization_size} for minimum and {self.max_visualization_size} for maximum):", initialvalue = self.workspace_obstacles_size, minvalue = self.min_visualization_size, maxvalue = self.max_visualization_size, parent = self.menus_area)  # ask the user to enter the size of the obstacles
        if obstacles_objects_size != None:  # if the user enters a number
            self.workspace_obstacles_size = obstacles_objects_size
    def open_matplotlib_simulator(self, event = None):  # open the matplotlib simulator of the robotic manipulator
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            self.built_robotic_manipulator.teach(q = self.built_robotic_manipulator.q, backend = "pyplot", vellipse = True, block = True)  # open the pyplot simulator and allow the user to control the robotic manipulator
        else:
            ms.showerror("Error", "You have not built the robotic manipulator model yet!", parent = self.menus_area)  # show an error message
    def open_close_swift_simulator(self, event = None):  # open the Swift simulator of the robotic manipulator
        if not self.swift_sim_thread_flag:  # if the thread that runs the online Swift simulation is not running
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                if self.built_robotic_manipulator_info["name"] in self.swift_simulated_robots_list:  # if the robotic manipulator model can be simulated in the Swift simulator
                    self.swift_sim_env = swift.Swift()  # create the Swift environment
                    self.swift_sim_env.launch(realtime = True, headless = False, rate = 60)  # launch the Swift simulator environment
                    self.swift_sim_env.sim_time = 0.0
                    self.swift_env_load_robot()  # load the robotic manipulator model inside the Swift simulator environment
                    self.swift_env_load_workspace_obstacles()  # load the workspace obstacles inside the Swift simulator environment
                    self.swift_info_label = swift.Label(f"Robotic manipulator name: {self.built_robotic_manipulator_info['name']}. If you want to exit the simulation, do not close the browser tab. Just press again the GUI's button that started the simulation.")
                    self.swift_robot_configuration_label = swift.Label(f"Robot configuration ({self.control_or_kinematics_variables_visualization}): {self.built_robotic_manipulator.q}")
                    self.swift_simulated_robot_move_button = swift.Button(self.move_swift_robot_obstacles_avoidance, "Move simulated robot")
                    self.swift_real_robot_move_button = swift.Button(self.move_real_robot_obstacles_avoidance, "Move real robot")
                    self.swift_sim_env.add(self.swift_info_label)  # add a label with some instructions inside the Swift simulator environment
                    self.swift_sim_env.add(self.swift_robot_configuration_label)  # add a label with the robotic manipulator configuration inside the Swift simulator environment
                    self.swift_sim_env.add(self.swift_simulated_robot_move_button)  # add a button that allows the user to move the simulated robotic manipulator inside the Swift simulator environment
                    self.swift_sim_env.add(self.swift_real_robot_move_button)  # add a button that allows the user to move the real robotic manipulator
                    self.start_swift_thread()  # start the thread that runs the online Swift simulation
                else:
                    ms.showerror("Error", f"The Swift simulator is available only for the following robots models! \n{self.swift_simulated_robots_list}", parent = self.menus_area)
            else:
                ms.showerror("Error", "You have not built the robotic manipulator model yet!", parent = self.menus_area)  # show an error message
        else:
            self.kill_swift_thread()  # kill the thread that runs the online Swift simulation
    def swift_env_load_robot(self, event = None):  # load the robotic manipulator model in the Swift simulator environment
        robot_name = self.built_robotic_manipulator_info["name"].lower()  # the name of the chosen robotic manipulator model
        robot_model_urdf_file_path = self.saved_robots_descriptions_folder_path + fr"/{robot_name}_description/urdf/{robot_name}.urdf"  # the path of the chosen robotic manipulator urdf/xacro file
        self.swift_robotic_manipulator = rtb.Robot.URDF(robot_model_urdf_file_path)  # load the chosen robotic manipulator model reading the URDF file
        self.swift_sim_env.add(self.swift_robotic_manipulator, robot_alpha = 1.0, collision_alpha = 0.0)  # add the chosen robotic manipulator model inside the Swift simulator environment
    def swift_env_load_workspace_obstacles(self, event = None):  # load the workspace obstacles in the Swift simulator environment
        # for the 2D plane where the obstacles are located
        plane_stl_file = self.saved_obstacles_objects_infos_folder_path + r"/obstacles_2d_plane.stl"  # the stl file of the 2D obstacles plane for the Swift simulation
        camera_stl_file = self.saved_obstacles_objects_infos_folder_path + r"/camera_object.stl"  # the stl file of the camera object for the Swift simulation
        plane_color = list(np.array(ImageColor.getcolor(self.front_side_2d_plane_color, "RGB")) / 255.0) + [1.0]  # the color of the 2D obstacles plane for the Swift simulation
        self.swift_sim_2d_plane = sg.Mesh(plane_stl_file, scale = np.array([self.obstacles_2d_plane_x_length * 0.001, self.obstacles_2d_plane_y_length * 0.001, 0.001]), color = plane_color)  # the 2D obstacles plane for the Swift simulation
        self.swift_sim_2d_plane._T = np.eye(4)  # the transformation of the 2D obstacles plane
        self.swift_sim_camera = sg.Mesh(camera_stl_file, scale = np.array([0.001, 0.001, 0.001]), color = [0.3, 0.0, 0.3, 1.0])  # the camera object for the Swift simulation
        self.swift_sim_camera._T = np.eye(4)  # the transformation of the camera object
        self.swift_sim_env.add(self.swift_sim_2d_plane)  # add the 2D obstacles plane inside the Swift simulator environment
        self.swift_sim_env.add(self.swift_sim_camera)  # add the camera object inside the Swift simulator environment
        # for the obstacles objects
        if len(self.chosen_workspace_saved_obstacles_objects_list) > 0:  # if there are saved workspaces with obstacles objects
            workspace_image_folder = self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_solver_workspace_image_name}"  # the path of the folder that contains the obstacles objects for the chosen workspace image
            obstacles_meshes_files = [workspace_image_folder + fr"/{obstacle_file}" for obstacle_file in os.listdir(workspace_image_folder) if ".stl" in obstacle_file]  # the stl files of the obstacles objects for the Swift simulation
            obstacles_color = list(np.array(ImageColor.getcolor(self.workspace_obstacles_color, "RGB")) / 255.0) + [1.0]  # the color of the obstacles objects for the Swift simulation
            for k in range(len(obstacles_meshes_files)):  # iterate through all the obstacles objects for the chosen workspace
                self.swift_sim_obstacles_objects.append(sg.Mesh(obstacles_meshes_files[k], scale = np.array([0.001, 0.001, 0.001]), color = obstacles_color))  # the obstacles objects for the Swift simulation
                self.swift_sim_obstacles_objects[-1]._T = np.eye(4)  # the transformation of the obstacles objects
                self.swift_sim_env.add(self.swift_sim_obstacles_objects[k])  # add the obstacles objects inside the Swift simulator environment
    def update_model_visualization_indicators(self, event = None):  # update the model and visualization indicators
        try:
            # model indicators
            self.joints_number_button.configure(text = self.joints_number)  # change the text of the button that shows the joints number of the robotic manipulator
            self.choose_joint_number_model_combobox.set(f"joint {self.chosen_joint_number_model}")  # set the chosen joint number to the chosen joint
            self.choose_joint_number_model_combobox["values"] = [f"joint {joint}" for joint in range(1, self.links_number)]  # change the values of the combobox that shows the joints number of the robotic manipulator model
            self.choose_frame_visualization_combobox.set(f"{self.chosen_frame_visualization}")  # set the chosen joint number to the chosen joint
            self.choose_frame_visualization_combobox["values"] = ["base"] + [f"frame {frame}" for frame in range(self.links_number)] + ["end-effector"]  # change the values of the combobox that shows the joints number of the robotic manipulator visualization
            self.choose_link_visualization_combobox.set(f"{self.chosen_link_visualization}")  # set the chosen link number to the chosen link
            self.choose_link_visualization_combobox["values"] = ["base"] + [f"link {link}" for link in range(1, self.links_number)]  # change the values of the combobox that shows the links number of the robotic manipulator visualization
            self.joints_types_combobox.set(self.joints_types[self.chosen_joint_number_model - 1])  # set the corresponding combobox to the current joint type
            self.a_den_har_parameter_button.configure(text = f"{self.a_den_har_parameters[self.chosen_joint_number_model - 1]:.3f}")  # change the text of the button that shows the a parameter of the Denavit - Hartenberg parameters of the robotic manipulator
            self.alpha_den_har_parameter_button.configure(text = f"{np.rad2deg(self.alpha_den_har_parameters[self.chosen_joint_number_model - 1]):.3f}")  # change the text of the button that shows the alpha parameter of the Denavit - Hartenberg parameters of the robotic manipulator
            if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0]:  # if the chosen joint is revolute
                self.d_den_har_parameter_button.configure(text = f"{self.d_den_har_parameters[self.chosen_joint_number_model - 1]:.3f}")  # change the text of the button that shows the d parameter of the Denavit - Hartenberg parameters of the robotic manipulator
                self.theta_den_har_parameter_button.configure(text = f"{np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]):.3f} " + "(var)")  # change the text of the button that shows the theta parameter of the Denavit - Hartenberg parameters of the robotic manipulator
            else:  # if the chosen joint is prismatic
                self.d_den_har_parameter_button.configure(text = f"{self.d_den_har_parameters[self.chosen_joint_number_model - 1]:.3f} " + "(var)")  # change the text of the button that shows the d parameter of the Denavit - Hartenberg parameters of the robotic manipulator
                self.theta_den_har_parameter_button.configure(text = f"{np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]):.3f}")  # change the text of the button that shows the theta parameter of the Denavit - Hartenberg parameters of the robotic manipulator
            joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_model - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the model
            self.var_limits_label.configure(text = f"Variable limits\n({['degrees', 'meters'][joint_type_index]}):")  # change the text of the label that shows the variable limits of the chosen joint
            self.var_min_limit_button.configure(text = f"{np.round(self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][0], [self.angles_precision, self.distances_precision][joint_type_index]):.3f}")  # change the text of the button that shows the minimum limit of the variable of the chosen joint
            self.var_max_limit_button.configure(text = f"{np.round(self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][1], [self.angles_precision, self.distances_precision][joint_type_index]):.3f}")  # change the text of the button that shows the maximum limit of the variable of the chosen joint
            self.load_model_combobox["values"] = self.saved_robots_models_files_list  # update the values of the combobox that contains the names of the saved files
            # visualization indicators
            frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
            link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen link number
            if frame_number_visualization in list(range(1, self.links_number)):  # if the chosen frame is a joint frame
                joint_number_visualization = frame_number_visualization - 1  # get the chosen joint number
                self.change_joint_position_button.configure(text = f"{self.joints_z_axis_positions[joint_number_visualization]:.3f}")  # change the text of the button that shows the position of the chosen joint
            else:
                self.change_joint_position_button.configure(text = "none")  # change the text of the button that shows the position of the chosen joint
            self.change_frame_color_button.configure(text = self.frames_origins_colors[frame_number_visualization])  # change the text of the button that shows the color of the chosen frame origin
            self.change_frame_size_button.configure(text = self.frames_origins_sizes[frame_number_visualization])  # change the text of the button that shows the size of the chosen frame origin
            self.change_link_color_button.configure(text = self.links_colors[link_number_visualization])  # change the text of the button that shows the color of the chosen link
            self.change_link_size_button.configure(text = self.links_sizes[link_number_visualization])  # change the text of the button that shows the size of the chosen link
            frames_origins, frames_orientations = self.get_all_frames_positions_orientations(np.zeros(self.joints_number,))  # get the positions and the orientations of the frames of the robotic manipulator when all the joints are at zero
            robotic_manipulator_links_points = [np.array(frames_origins[0])] + [np.array(frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]) for k in range(self.joints_number)] + [np.array(frames_origins[-1])]  # the points of the links of the robotic manipulator
            self.initial_links_lengths = np.linalg.norm(np.diff(robotic_manipulator_links_points, axis = 0), axis = 1).tolist()  # the initial lengths of the links of the robotic manipulator
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                frames_origins, frames_orientations = self.get_all_frames_positions_orientations(self.built_robotic_manipulator.q)  # get the positions and the orientations of the current frames of the robotic manipulator
            robotic_manipulator_links_points = [np.array(frames_origins[0])] + [np.array(frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]) for k in range(self.joints_number)] + [np.array(frames_origins[-1])]  # the points of the links of the robotic manipulator
            links_lengths = np.linalg.norm(np.diff(robotic_manipulator_links_points, axis = 0), axis = 1)  # the sizes of the links of the robotic manipulator
            self.link_length_indicator.configure(text = f"{links_lengths[link_number_visualization]:.3f}")  # change the text of the button that shows the length of the chosen link
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_model_visualization_indicators: {e}")

    # the functions used for the thread that runs the online swift simulation
    def start_swift_thread(self, event = None):  # start the thread that runs the online Swift simulation
        self.swift_sim_thread_flag = True  # let the swift thread run
        self.swift_sim_thread = threading.Thread(target = self.swift_thread_run_function)  # create the thread for the online Swift simulation
        self.swift_sim_thread.start()  # start the online Swift simulation thread
    def kill_swift_thread(self, event = None):  # kill the thread that runs the online Swift simulation
        self.swift_sim_thread_flag = False  # stop the online Swift simulation thread
    def swift_thread_run_function(self):  # the run function, it continuously runs the online Swift simulation
        while True:  # while the thread is running
            if self.swift_sim_thread_flag:  # if the thread is running
                try:  # try to run the online Swift simulation
                    # for the robotic manipulator
                    self.swift_robotic_manipulator.base = self.get_transformation_matrix(self.built_robotic_manipulator_info["base_position_wrt_world"], self.built_robotic_manipulator_info["base_orientation_wrt_world"])  # set the base pose of the robotic manipulator model
                    self.swift_robotic_manipulator.tool = self.get_transformation_matrix(self.built_robotic_manipulator_info["end_effector_position_wrt_last_frame"], self.built_robotic_manipulator_info["end_effector_orientation_wrt_last_frame"])  # set the end-effector pose of the robotic manipulator model
                    if not self.simulated_robot_is_moving:  # if the simulated robot is not moving
                        self.swift_robotic_manipulator.q = np.copy(self.built_robotic_manipulator.q)  # set the joints angles of the robotic manipulator model
                    # for the camera object
                    self.swift_sim_camera._T = self.camera_wrt_world_transformation_matrix  # set the camera object transformation
                    # for the 2D plane where the obstacles are located
                    self.swift_sim_2d_plane._T = self.obst_plane_wrt_world_transformation_matrix  # set the 2D plane transformation
                    # for the obstacles objects
                    for obst in range(len(self.swift_sim_obstacles_objects)):
                        self.swift_sim_obstacles_objects[obst]._T = self.obst_plane_wrt_world_transformation_matrix  # set the obstacles objects transformation
                    # for the labels
                    joints_configuration_columns_indicator = 1  # the number of rows of the joints configuration indicator
                    joints_configuration = ""  # the configuration of the joints of the robotic manipulator
                    for k in range(self.joints_number):
                        joints_configuration += f"{k + 1}" + [f"(°): {np.rad2deg(self.swift_robotic_manipulator.q[k]):.1f}", f"(m): {self.swift_robotic_manipulator.q[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])]  # the joints configuration indicator of the robotic manipulator
                        if k < self.joints_number - 1: joints_configuration += ",   "
                        if (k + 1) % joints_configuration_columns_indicator == 0 and (k + 1) != self.joints_number: joints_configuration += " "
                    self.swift_robot_configuration_label.desc = f"Robot configuration ({self.control_or_kinematics_variables_visualization}): {joints_configuration}"  # update the label with the robotic manipulator configuration inside the Swift simulator environment
                    # make the Swift simulation dt step and render the scene
                    self.swift_sim_env.step(dt = self.swift_sim_dt, render = True)  # run the online Swift simulation
                except:  # if an error occurs
                    self.kill_swift_thread()  # kill the thread that runs the online Swift simulation
            else:  # if the thread is stopped
                try: self.swift_sim_env.close()  # close the online Swift simulation environment
                except: pass
                break  # break the while loop
        # self.swift_sim_env.hold()  # hold the online Swift simulation environment

    # functions for the main menu where the robotic manipulator kinematics are analyzed
    # forward kinematics
    def change_chosen_joint_number_fkine(self, event = None):  # change the chosen joint number for the forward kinematics analysis
        self.chosen_joint_number_fkine = int(self.choose_joint_number_fkine_combobox.get().split(" ")[-1])  # change the chosen control joint number of the robotic manipulator
        self.update_forward_kinematics_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_chosen_joint_number_fkine_2(self, event = None):  # change the chosen joint number for the forward kinematics analysis
        self.chosen_joint_number_fkine = self.joint_fkine_value_combobox.current() + 1  # change the chosen control joint number of the robotic manipulator
        self.update_forward_kinematics_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_chosen_fkine_variable(self, event = None):  # change the value of the chosen joint variable for the forward kinematics analysis
        try:  # try to get the entered value
            joint_fkine_value = float(self.joint_fkine_value_combobox.get())  # the fkine variable of the chosen joint
            joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_fkine - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
            self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1] = [np.deg2rad(joint_fkine_value), joint_fkine_value][joint_type_index]  # change the fkine variable of the chosen joint
            control_variable_limits = [np.deg2rad(self.control_joints_variables_limits[self.chosen_joint_number_fkine - 1][0]).tolist(), self.control_joints_variables_limits[self.chosen_joint_number_fkine - 1][1]][joint_type_index]  # the control variable limits of the chosen joint
            if joint_fkine_value < self.control_joints_variables_limits[self.chosen_joint_number_fkine - 1][joint_type_index][0]:  # if the entered value is less than the joint fkine variable minimum limit
                self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1] = control_variable_limits[0]  # change the fkine variable of the chosen joint to the minimum limit
            elif joint_fkine_value > self.control_joints_variables_limits[self.chosen_joint_number_fkine - 1][joint_type_index][1]:  # if the entered value is greater than the joint fkine variable maximum limit
                self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1] = control_variable_limits[1]  # change the fkine variable of the chosen joint to the maximum limit
            self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
        except:  # if the entered value is not a number
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
    def copy_control_to_fkine_values(self, event = None):  # copy the control variables to the forward kinematics variables
        self.forward_kinematics_variables = copy.deepcopy(self.control_joints_variables)  # copy the control variables to the forward kinematics variables
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def change_fkine_variable_slider(self, slider_num, event = None):  # change the value of the chosen joint variable (changing the corresponding slider) for the forward kinematics analysis
        joint_fkine_value = self.fkine_variables_sliders[slider_num].get()  # the fkine variable of the chosen joint
        joint_type_index = self.joints_types_list.index(self.joints_types[slider_num])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
        self.forward_kinematics_variables[slider_num] = [np.deg2rad(joint_fkine_value), joint_fkine_value][joint_type_index]  # change the fkine variable of the chosen joint
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def change_chosen_frame_fkine(self, event = None):  # change the chosen frame for the forward kinematics analysis
        self.chosen_frame_fkine = self.choose_frame_fkine_combobox.get()  # change the chosen frame for the forward kinematics analysis
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def change_orientation_representation_fkine(self, event = None):  # change the orientation representation for the forward kinematics analysis
        self.orientation_representation_fkine = self.frame_orientation_representation_combobox.get()  # change the orientation representation for the forward kinematics analysis
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def set_joints_range_divisions(self, event = None):  # set the divisions of the joints range for the calculation of the robot's reachable workspace
        self.joints_range_divisions = self.alternate_matrix_elements(self.joints_range_divisions_list, self.joints_range_divisions)  # change the divisions of the joints range for the calculation of the robot's reachable workspace
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def compute_plot_reachable_workspace(self, event = None):  # compute and plot the reachable workspace of the robotic manipulator
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            kin.find_reachable_workspace(self.built_robotic_manipulator, self.joints_range_divisions, True)  # compute and plot the reachable workspace of the robotic manipulator
        else:  # if no robotic manipulator is built yet
            ms.showerror("Error", "You have not built a robotic manipulator model yet!", parent = self.menus_area)  # show an error message
    def show_fkine_info(self, event = None):  # show the forward kinematics information
        pass
    def update_forward_kinematics_indicators(self, event = None):  # update the forward kinematics indicators
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_fkine - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
                joint_fkine_value = [f"{np.rad2deg(self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1]):.1f}", f"{self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1]:.3f}"][joint_type_index]  # the fkine value of the chosen joint
                self.choose_joint_number_fkine_combobox.set(f"joint {self.chosen_joint_number_fkine}")  # set the combobox to the chosen joint number for the forward kinematics analysis
                self.joint_fkine_value_combobox.set(joint_fkine_value)  # set the combobox to the chosen joint variable for the forward kinematics analysis
                self.joint_fkine_value_combobox["values"] = [f"{k + 1}: " + [f"{np.rad2deg(self.forward_kinematics_variables[k]):.1f}", f"{self.forward_kinematics_variables[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])] for k in range(self.joints_number)]  # update the values of the joint fkine value combobox
                self.fkine_value_unit_indicator.configure(text = self.joints_types[self.chosen_joint_number_fkine - 1] + " " + ["(°)", "(m)"][joint_type_index])  # change the unit of the forward kinematics value indicator
                self.choose_frame_fkine_combobox.set(self.chosen_frame_fkine)  # set the combobox to the chosen frame for the forward kinematics analysis
                self.frame_orientation_representation_combobox.set(self.orientation_representation_fkine)  # set the combobox to the chosen orientation representation for the forward kinematics analysis
                # solve the forward kinematics of the robotic manipulator for the chosen frame and the chosen joints configuration
                self.fkine_frame_position, fkine_frame_orientation = self.get_fkine_frame_position_orientation(self.forward_kinematics_variables, self.chosen_frame_fkine)  # calculate the position and the orientation of the chosen frame
                self.frame_position_indicator.configure(text = str([np.round(self.fkine_frame_position[k], self.distances_precision) for k in range(len(self.fkine_frame_position))]))
                r = sc.spatial.transform.Rotation.from_matrix(fkine_frame_orientation)  # create a rotation object from the rotation matrix
                if self.orientation_representation_fkine == self.orientation_representations_fkine_list[0]:  # if the orientation representation is in Euler xyz extrinsic angles
                    orient = r.as_euler("xyz")
                    self.fkine_frame_orientation = np.copy(orient)  # set the orientation of the chosen frame for the forward kinematics analysis
                    self.frame_orientation_indicator.configure(text = str([np.round(np.rad2deg(orient[k]), self.angles_precision) for k in range(3)]))
                elif self.orientation_representation_fkine == self.orientation_representations_fkine_list[1]:  # if the orientation representation is in quaternion form
                    orient = r.as_quat()
                    self.frame_orientation_indicator.configure(text = str([np.round(orient[k], 3) for k in range(4)]))
                elif self.orientation_representation_fkine == self.orientation_representations_fkine_list[2]:  # if the orientation representation is in rotation matrix form
                    self.frame_orientation_indicator.configure(text = np.round(fkine_frame_orientation, 3))
                for k in range(self.joints_number):
                    self.fkine_variables_sliders[k].set([np.rad2deg(self.forward_kinematics_variables[k]), self.forward_kinematics_variables[k]][self.joints_types_list.index(self.joints_types[k])])  # set the value of the chosen joint variable (changing the corresponding slider) for the forward kinematics analysis
                self.joints_range_divisions_button.configure(text = f"{self.joints_range_divisions}")  # change the text of the button that shows the divisions of the joints range for the calculation of the robot's reachable workspace
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_forward_kinematics_indicators: {e}")
    # inverse kinematics
    def get_fkine_pose_result(self, event = None):  # get the forward kinematics pose result to the inverse kinematics analysis
        self.chosen_frame_fkine = "end-effector"  # change the chosen frame for the forward kinematics analysis
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
        self.chosen_invkine_3d_position = np.copy(self.fkine_frame_position)  # change the end-effector position for the inverse kinematics analysis
        self.chosen_invkine_orientation = np.copy(self.fkine_frame_orientation)  # change the end-effector orientation for the inverse kinematics analysis
        self.update_inverse_kinematics_indicators()  # update the inverse kinematics indicators
    def send_invkine_joints_config_result(self, event = None):  # send the inverse kinematics joints configuration result to the forward kinematics analysis
        self.chosen_frame_fkine = "end-effector"  # change the chosen frame for the forward kinematics analysis
        self.forward_kinematics_variables = np.copy(self.invkine_joints_configuration).tolist()  # change the joints configuration for the forward kinematics analysis
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def choose_end_effector_position_invkine(self, event = None):  # choose the end-effector position for the inverse kinematics analysis
        end_effector_pos_x = sd.askfloat("Choose the end-effector x position", "Enter the x position of the end-effector (in meters):", initialvalue = self.chosen_invkine_3d_position[0], parent = self.menus_area)  # ask the user to enter the x position of the end-effector
        if end_effector_pos_x != None:  # if the user enters a number
            self.chosen_invkine_3d_position[0] = end_effector_pos_x  # change the x position of the end-effector
        end_effector_pos_y = sd.askfloat("Choose the end-effector y position", "Enter the y position of the end-effector (in meters):", initialvalue = self.chosen_invkine_3d_position[1], parent = self.menus_area)  # ask the user to enter the y position of the end-effector
        if end_effector_pos_y != None:  # if the user enters a number
            self.chosen_invkine_3d_position[1] = end_effector_pos_y  # change the y position of the end-effector
        end_effector_pos_z = sd.askfloat("Choose the end-effector z position", "Enter the z position of the end-effector (in meters):", initialvalue = self.chosen_invkine_3d_position[2], parent = self.menus_area)  # ask the user to enter the z position of the end-effector
        if end_effector_pos_z != None:  # if the user enters a number
            self.chosen_invkine_3d_position[2] = end_effector_pos_z  # change the z position of the end-effector
        self.update_inverse_kinematics_indicators()  # update the inverse kinematics indicators
    def choose_end_effector_orientation_invkine(self, event = None):  # choose the end-effector orientation for the inverse kinematics analysis
        end_effector_rot_x = sd.askfloat("Choose the end-effector x rotation", "Enter the rotation of the end-effector around world frame's x-axis (in degrees), \nthe first rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.chosen_invkine_orientation[0]), parent = self.menus_area)  # ask the user to enter the x rotation of the end-effector
        if end_effector_rot_x != None:  # if the user enters a number
            self.chosen_invkine_orientation[0] = np.deg2rad(end_effector_rot_x)  # change the x rotation of the end-effector
        end_effector_rot_y = sd.askfloat("Choose the end-effector y rotation", "Enter the rotation of the end-effector around world frame's y-axis (in degrees), \nthe second rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.chosen_invkine_orientation[1]), parent = self.menus_area)  # ask the user to enter the y rotation of the end-effector
        if end_effector_rot_y != None:  # if the user enters a number
            self.chosen_invkine_orientation[1] = np.deg2rad(end_effector_rot_y)  # change the y rotation of the end-effector
        end_effector_rot_z = sd.askfloat("Choose the end-effector z rotation", "Enter the rotation of the end-effector around world frame's z-axis (in degrees), \nthe third rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.chosen_invkine_orientation[2]), parent = self.menus_area)  # ask the user to enter the z rotation of the end-effector
        if end_effector_rot_z != None:  # if the user enters a number
            self.chosen_invkine_orientation[2] = np.deg2rad(end_effector_rot_z)  # change the z rotation of the end-effector
        self.update_inverse_kinematics_indicators()  # update the inverse kinematics indicators
    def change_invkine_tolerance(self, event = None):  # change the tolerance for the inverse kinematics analysis
        invkine_tolerance = sd.askfloat("Choose the inverse kinematics solver tolerance", "Enter the tolerance for the inverse\nkinematics numerical solver:", initialvalue = self.invkine_tolerance, parent = self.menus_area)  # ask the user to enter the tolerance for the inverse kinematics analysis
        if invkine_tolerance != None:  # if the user enters a number
            self.invkine_tolerance = invkine_tolerance  # change the tolerance for the inverse kinematics analysis
        self.update_inverse_kinematics_indicators()  # update the inverse kinematics indicators
    def show_invkine_info(self, event = None):  # show the inverse kinematics information
        pass
    def update_inverse_kinematics_indicators(self, event = None):  # update the inverse kinematics indicators
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                self.choose_end_effector_position_button.configure(text = str([np.round(self.chosen_invkine_3d_position[k], self.distances_precision) for k in range(len(self.chosen_invkine_3d_position))]))  # change the text of the button that allows the user to choose the end-effector position
                self.choose_end_effector_orientation_button.configure(text = str([np.round(np.rad2deg(self.chosen_invkine_orientation[k]), self.angles_precision) for k in range(len(self.chosen_invkine_orientation))]))  # change the text of the button that allows the user to choose the end-effector orientation
                self.choose_invkine_tolerance_button.configure(text = f"{np.round(self.invkine_tolerance, 10)}")  # change the text of the button that allows the user to choose the tolerance for the inverse kinematics solver
                # solve the inverse kinematics of the robotic manipulator for the chosen end-effector position and orientation
                self.invkine_joints_configuration, invkine_success = kin.compute_inverse_kinematics(self.built_robotic_manipulator, self.get_transformation_matrix(self.chosen_invkine_3d_position, self.chosen_invkine_orientation), self.invkine_tolerance)  # compute the inverse kinematics of the robotic manipulator
                joints_configuration = ""  # initialize the joints configuration indicator of the robotic manipulator
                joints_configuration_columns_indicator = 5  # the number of rows of the joints configuration indicator
                if invkine_success:  # if the inverse kinematics analysis is successful
                    for k in range(self.joints_number):
                        joints_configuration += f"{k + 1}" + [f"(°): {np.rad2deg(self.invkine_joints_configuration[k]):.1f}", f"(m): {self.invkine_joints_configuration[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])]  # the joints configuration indicator of the robotic manipulator
                        if k < self.joints_number - 1: joints_configuration += "   "
                        if (k + 1) % joints_configuration_columns_indicator == 0 and (k + 1) != self.joints_number: joints_configuration += "\n"
                else:
                    joints_configuration = "This end-effector configuration does not belong\nin the reachable workspace! No solution found!"
                self.joints_configuration_indicator.configure(text = joints_configuration)  # change the text of the joints configuration indicator
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_inverse_kinematics_indicators: {e}")
    # differential kinematics
    def change_chosen_joint_number_diffkine(self, event = None):  # change the chosen joint number for the differential kinematics analysis
        self.chosen_joint_number_diffkine = int(self.choose_joint_number_diffkine_combobox.get().split(" ")[-1])  # change the chosen joint number for the differential kinematics analysis
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def change_chosen_joint_number_diffkine_2(self, event = None):  # change the chosen joint number for the differential kinematics analysis
        self.chosen_joint_number_diffkine = self.joint_diffkine_velocity_combobox.current() + 1  # change the chosen joint number for the differential kinematics analysis
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def change_chosen_diffkine_velocity(self, event = None):  # change the value of the chosen joint velocity for the differential kinematics analysis
        try:  # try to get the entered value
            joint_diffkine_velocity = np.deg2rad(float(self.joint_diffkine_velocity_combobox.get()))  # the diffkine velocity of the chosen joint
            self.differential_kinematics_velocities[self.chosen_joint_number_diffkine - 1] = joint_diffkine_velocity  # change the diffkine velocity of the chosen joint
            self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
        except:  # if the entered value is not a number
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
    def change_diffkine_variable_slider(self, slider_num, event = None):  # change the value of the chosen joint velocity (changing the corresponding slider) for the differential kinematics analysis
        joint_diffkine_velocity = self.diffkine_variables_sliders[slider_num].get()  # the diffkine velocity of the chosen joint
        joint_type_index = self.joints_types_list.index(self.joints_types[slider_num])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the differential kinematics analysis
        self.differential_kinematics_velocities[slider_num] = [np.deg2rad(joint_diffkine_velocity), joint_diffkine_velocity][joint_type_index]  # change the diffkine velocity of the chosen joint
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def change_diffkine_wrt_frame(self, event = None):  # change the frame with respect to which the differential kinematics is computed
        self.diffkine_wrt_frame = self.alternate_matrix_elements(self.diffkine_wrt_frame_list, self.diffkine_wrt_frame)  # change the frame with respect to which the differential kinematics is computed
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def show_diffkine_info(self, event = None):  # show the differential kinematics information
        pass
    def update_differential_kinematics_indicators(self, event = None):  # update the differential kinematics indicators
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                self.joints_configuration_diffkine_indicator.configure(text = self.control_or_kinematics_variables_visualization)  # change the text of the differential kinematics robots joints indicator
                joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_diffkine - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
                joint_diffkine_velocity = [f"{np.rad2deg(self.differential_kinematics_velocities[self.chosen_joint_number_diffkine - 1]):.1f}", f"{self.differential_kinematics_velocities[self.chosen_joint_number_diffkine - 1]:.3f}"][joint_type_index]  # the diffkine velocity of the chosen joint
                self.choose_joint_number_diffkine_combobox.set(f"joint {self.chosen_joint_number_diffkine}")  # set the combobox to the chosen joint number for the differential kinematics analysis
                self.joint_diffkine_velocity_combobox.set(joint_diffkine_velocity)  # set the combobox to the chosen joint velocity for the differential kinematics analysis
                self.joint_diffkine_velocity_combobox["values"] = [f"{k + 1}: " + [f"{np.rad2deg(self.differential_kinematics_velocities[k]):.1f}", f"{self.differential_kinematics_velocities[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])] for k in range(self.joints_number)]  # update the values of the joint diffkine velocity combobox
                self.diffkine_value_unit_indicator.configure(text = ["(°/s)", "(m/s)"][joint_type_index])  # change the unit of the differential kinematics velocity indicator
                self.diffkine_wrt_frame_button.configure(text = self.diffkine_wrt_frame)  # change the text of the button that allows the user to change the frame with respect to which the differential kinematics is computed
                # solve the differential kinematics of the robotic manipulator for the chosen joints velocities
                end_effector_velocities = kin.compute_differential_kinematics(self.built_robotic_manipulator, self.built_robotic_manipulator.q, self.differential_kinematics_velocities, self.diffkine_wrt_frame)  # compute the differential kinematics of the robotic manipulator
                self.diffkine_linear_vel = np.array(end_effector_velocities[:3], dtype = float)  # the linear velocity of the end-effector
                self.diffkine_angular_vel = np.array(end_effector_velocities[3:], dtype = float)  # the angular velocity of the end-effector
                self.end_effector_linear_vel_indicator.configure(text = str([np.round(self.diffkine_linear_vel[k], self.distances_precision) for k in range(len(self.diffkine_linear_vel))]))  # change the text of the end-effector linear velocity indicator
                self.end_effector_angular_vel_indicator.configure(text = str([np.round(np.rad2deg(self.diffkine_angular_vel[k]), self.angles_precision) for k in range(len(self.diffkine_angular_vel))]))  # change the text of the end-effector angular velocity indicator
                for k in range(self.joints_number):
                    self.diffkine_variables_sliders[k].set([np.rad2deg(self.differential_kinematics_velocities[k]), self.differential_kinematics_velocities[k]][self.joints_types_list.index(self.joints_types[k])])  # set the value of the chosen joint velocity (changing the corresponding slider) for the differential kinematics analysis
        except: pass
        # except Exception as e: 
        #     if "invalid command name" not in str(e): print(f"Error in update_differential_kinematics_indicators: {e}")
    # inverse differential kinematics
    def get_diffkine_velocities_result(self, event = None):  # get the differential kinematics velocity result to the inverse differential kinematics analysis
        self.chosen_invdiffkine_linear_vel = np.copy(self.diffkine_linear_vel)  # change the end-effector linear velocity for the inverse differential kinematics analysis
        self.chosen_invdiffkine_angular_vel = np.copy(self.diffkine_angular_vel)  # change the end-effector angular velocity for the inverse differential kinematics analysis
        self.update_inverse_differential_kinematics_indicators()  # update the differential kinematics indicators
    def send_invdiffkine_joints_velocities_result(self, event = None):  # send the inverse differential kinematics joints velocities result to the differential kinematics analysis
        self.differential_kinematics_velocities = np.copy(self.invdiffkine_joints_velocities)  # change the joints velocities for the inverse differential kinematics analysis
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def choose_end_effector_linear_velocity(self, event = None):  # choose the end-effector linear velocity for the differential kinematics analysis
        end_effector_vel_x = sd.askfloat("Choose the end-effector x linear velocity", "Enter the x linear velocity of the end-effector (in m/s):", initialvalue = self.chosen_invdiffkine_linear_vel[0], parent = self.menus_area)
        if end_effector_vel_x != None:  # if the user enters a number
            self.chosen_invdiffkine_linear_vel[0] = end_effector_vel_x  # change the x linear velocity of the end-effector
        end_effector_vel_y = sd.askfloat("Choose the end-effector y linear velocity", "Enter the y linear velocity of the end-effector (in m/s):", initialvalue = self.chosen_invdiffkine_linear_vel[1], parent = self.menus_area)
        if end_effector_vel_y != None:  # if the user enters a number
            self.chosen_invdiffkine_linear_vel[1] = end_effector_vel_y  # change the y linear velocity of the end-effector
        end_effector_vel_z = sd.askfloat("Choose the end-effector z linear velocity", "Enter the z linear velocity of the end-effector (in m/s):", initialvalue = self.chosen_invdiffkine_linear_vel[2], parent = self.menus_area)
        if end_effector_vel_z != None:  # if the user enters a number
            self.chosen_invdiffkine_linear_vel[2] = end_effector_vel_z  # change the z linear velocity of the end-effector
        self.update_inverse_differential_kinematics_indicators()  # update the differential kinematics indicators
    def choose_end_effector_angular_velocity(self, event = None):  # choose the end-effector angular velocity for the differential kinematics analysis
        end_effector_ang_vel_x = sd.askfloat("Choose the end-effector x angular velocity", "Enter the x angular velocity of the end-effector (in °/s):", initialvalue = np.rad2deg(self.chosen_invdiffkine_angular_vel[0]), parent = self.menus_area)
        if end_effector_ang_vel_x != None:  # if the user enters a number
            self.chosen_invdiffkine_angular_vel[0] = np.deg2rad(end_effector_ang_vel_x)  # change the x angular velocity of the end-effector
        end_effector_ang_vel_y = sd.askfloat("Choose the end-effector y angular velocity", "Enter the y angular velocity of the end-effector (in °/s):", initialvalue = np.rad2deg(self.chosen_invdiffkine_angular_vel[1]), parent = self.menus_area)
        if end_effector_ang_vel_y != None:  # if the user enters a number
            self.chosen_invdiffkine_angular_vel[1] = np.deg2rad(end_effector_ang_vel_y)  # change the y angular velocity of the end-effector
        end_effector_ang_vel_z = sd.askfloat("Choose the end-effector z angular velocity", "Enter the z angular velocity of the end-effector (in °/s):", initialvalue = np.rad2deg(self.chosen_invdiffkine_angular_vel[2]), parent = self.menus_area)
        if end_effector_ang_vel_z != None:  # if the user enters a number
            self.chosen_invdiffkine_angular_vel[2] = np.deg2rad(end_effector_ang_vel_z)  # change the z angular velocity of the end-effector
        self.update_inverse_differential_kinematics_indicators()  # update the differential kinematics indicators
    def change_invdiffkine_wrt_frame(self, event = None):  # change the frame with respect to which the inverse differential kinematics is computed
        self.invdiffkine_wrt_frame = self.alternate_matrix_elements(self.invdiffkine_wrt_frame_list, self.invdiffkine_wrt_frame)  # change the frame with respect to which the inverse differential kinematics is computed
        self.update_inverse_differential_kinematics_indicators()  # update the inverse differential kinematics indicators
    def show_invdiffkine_info(self, event = None):  # show the inverse differential kinematics information
        pass
    def update_inverse_differential_kinematics_indicators(self, event = None):  # update the inverse differential kinematics indicators
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                self.joints_configuration_invkine_indicator.configure(text = self.control_or_kinematics_variables_visualization)  # change the text of the inverse differential kinematics robots joints indicator
                self.end_effector_linear_velocity_button.configure(text = str([np.round(self.chosen_invdiffkine_linear_vel[k], self.distances_precision) for k in range(len(self.chosen_invdiffkine_linear_vel))]))  # change the text of the button that allows the user to choose the end-effector linear velocity
                self.end_effector_angular_velocity_button.configure(text = str([np.round(np.rad2deg(self.chosen_invdiffkine_angular_vel[k]), self.angles_precision) for k in range(len(self.chosen_invdiffkine_angular_vel))]))  # change the text of the button that allows the user to choose the end-effector angular velocity
                self.invdiffkine_wrt_frame_button.configure(text = self.invdiffkine_wrt_frame)  # change the text of the button that allows the user to change the frame with respect to which the inverse differential kinematics is computed
                # solve the inverse differential kinematics of the robotic manipulator for the chosen end-effector linear and angular velocities
                end_effector_velocity = np.concatenate((self.chosen_invdiffkine_linear_vel, self.chosen_invdiffkine_angular_vel), axis = 0)  # the end-effector velocity
                self.invdiffkine_joints_velocities, diffkine_success = kin.compute_inverse_differential_kinematics(self.built_robotic_manipulator, self.built_robotic_manipulator.q, end_effector_velocity, self.invdiffkine_wrt_frame)  # compute the inverse differential kinematics of the robotic manipulator
                joints_velocities = ""  # initialize the joints velocities indicator of the robotic manipulator
                joints_velocities_columns_indicator = 5  # the number of rows of the joints velocities indicator
                if diffkine_success:  # if the inverse differential kinematics analysis is successful
                    for k in range(self.joints_number):
                        joints_velocities += f"{k + 1}" + [f"(°/s): {np.rad2deg(self.invdiffkine_joints_velocities[k]):.1f}", f"(m/s): {self.invdiffkine_joints_velocities[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])]  # the joints velocities indicator of the robotic manipulator
                        if k < self.joints_number - 1: joints_velocities += "   "
                        if (k + 1) % joints_velocities_columns_indicator == 0 and (k + 1) != self.joints_number: joints_velocities += "\n"
                else:
                    joints_velocities = "The resulting Jacobian is singular!\nNo solution found!"
                self.joints_velocities_indicator.configure(text = joints_velocities)  # change the text of the joints configuration indicator
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_inverse_differential_kinematics_indicators: {e}")
    # functions used for the robotic manipulator manipulation
    def get_transformation_matrix(self, position, orientation, event = None):  # get the transformation matrix from the position and orientation (given in the form of roll-pitch-yaw angles) of a frame
        return sm.SE3(np.array(position, dtype = float)) * sm.SE3.RPY(np.array(orientation, dtype = float))  # return the transformation matrix
    def get_robot_joints_variables(self, joints_variables_chosen, event = None):  # get the proper values for the joints variables (control or fkine)
        if joints_variables_chosen == self.control_or_kinematics_variables_visualization_list[0]:  # if the control values are visualized
            q_config = np.array(self.control_joints_variables)  # get the control values for the joints variables
        elif joints_variables_chosen == self.control_or_kinematics_variables_visualization_list[1]:  # if the forward kinematics values are visualized
            q_config = np.array(self.forward_kinematics_variables)  # get the forward kinematics values for the joints variables
        return q_config  # return the proper values for the joints variables
    def get_all_frames_positions_orientations(self, q, event = None):  # get the positions and the orientations (as 3x3 rotation matrices) of the frames of the robotic manipulator
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            DH_parameters = np.vstack([self.built_robotic_manipulator_info["a_den_har_parameters"], self.built_robotic_manipulator_info["alpha_den_har_parameters"], self.built_robotic_manipulator_info["d_den_har_parameters"], self.built_robotic_manipulator_info["theta_den_har_parameters"]])  # the DH parameters of the robotic manipulator
            world_base_T = self.get_transformation_matrix(self.base_position_wrt_world, self.base_orientation_wrt_world)  # base wrt world transformation matrix
            base_0_T = self.get_transformation_matrix(self.zero_frame_position_wrt_base, self.zero_frame_orientation_wrt_base)  # zero frame wrt base transformation matrix
            robot_end_effector_T = self.get_transformation_matrix(self.end_effector_position_wrt_last_frame, self.end_effector_orientation_wrt_last_frame)  # end effector wrt n frame transformation matrix
            fkine_all_frames = kin.compute_forward_kinematics_all_frames(world_base_T, base_0_T, robot_end_effector_T, DH_parameters, self.built_robotic_manipulator_info["joints_types"], q)  # compute the forward kinematics for all the frames of the robotic manipulator
            frames_positions = [np.array(fkine_all_frames[frame])[:3, 3] for frame in range(len(fkine_all_frames))]  # get the positions of the frames of the robotic manipulator
            frames_orientations = [np.array(fkine_all_frames[frame])[:3, :3] for frame in range(len(fkine_all_frames))]  # get the orientations of the frames of the robotic manipulator
        else:
            frames_positions = [np.zeros(3, dtype = float) for frame in range(self.frames_number)]
            frames_orientations = [np.eye(3, dtype = float) for frame in range(self.frames_number)]
        return frames_positions, frames_orientations  # return the positions and the orientations of the frames of the robotic manipulator
    def get_fkine_frame_position_orientation(self, q, chosen_frame, event = None):  # calculate the position and the orientation of the chosen frame
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            DH_parameters = np.vstack([self.built_robotic_manipulator_info["a_den_har_parameters"], self.built_robotic_manipulator_info["alpha_den_har_parameters"], self.built_robotic_manipulator_info["d_den_har_parameters"], self.built_robotic_manipulator_info["theta_den_har_parameters"]])  # the DH parameters of the robotic manipulator
            world_base_T = self.get_transformation_matrix(self.base_position_wrt_world, self.base_orientation_wrt_world)  # base wrt world transformation matrix
            base_0_T = self.get_transformation_matrix(self.zero_frame_position_wrt_base, self.zero_frame_orientation_wrt_base)  # zero frame wrt base transformation matrix
            robot_end_effector_T = self.get_transformation_matrix(self.end_effector_position_wrt_last_frame, self.end_effector_orientation_wrt_last_frame)  # end effector wrt n frame transformation matrix
            fkine_chosen_frame = kin.compute_forward_kinematics_until_frame(self.built_robotic_manipulator, world_base_T, base_0_T, robot_end_effector_T, DH_parameters, self.built_robotic_manipulator_info["joints_types"], q, chosen_frame)  # compute the forward kinematics for the chosen frame of the robotic manipulator
            pos = np.array(fkine_chosen_frame)[:3, 3]  # get the position of the chosen frame
            orient = np.array(fkine_chosen_frame)[:3, :3]  # get the orientation of the chosen frame
        else:  # if there is no robotic manipulator model built
            pos = np.zeros(3, dtype = float)  # the position of the chosen frame
            orient = np.eye(3, dtype = float)  # the orientation of the chosen frame
        return pos, orient  # return the position and the orientation of the chosen frame

    # for the main menu where the robotic manipulator is controlled
    def obtain_serial_ports(self, event = None):  # obtain the entire list of the available serial ports
        self.available_serial_ports = spf.serial_ports()  # search for the available serial ports of the computer
        if self.available_serial_ports:  # if there are available serial ports
            self.serial_port = self.available_serial_ports[0]  # the current serial port
        else:  # if there are no available serial ports
            ms.showerror("Error", "No available serial ports found!", parent = self.menus_area)  # show an error message
        self.update_serial_connection_indicators(self.serial_connection_state)  # update the serial connection indicators
    def change_serial_port(self, event = None):  # change the serial port used for the serial communication with the arduino microcontroller
        self.serial_port = self.serial_ports_combobox.get()  # change the serial port used for the serial communication with the arduino microcontroller
        self.update_serial_connection_indicators(self.serial_connection_state)  # update the serial connection indicators
    def change_baudrate(self, event = None):  # change the baudrate used for the serial communication with the arduino microcontroller
        self.baudrate = self.baudrates_combobox.get()  # change the baudrate used for the serial communication with the arduino microcontroller
        self.update_serial_connection_indicators(self.serial_connection_state)  # update the serial connection indicators
    def serial_connect_disconnect(self, event = None):  # connect to the chosen serial port to establish the serial communication with the arduino microcontroller or disconnect from the current serial port
        if self.serial_port != "":  # if the serial port is chosen
            if self.baudrate != "":  # if the baudrate is chosen
                if self.serial_connection.is_open:  # if the serial connection is open, then close it
                    current_serial_port = self.serial_connection.port  # the current serial port
                    self.close_serial_connection()  # close the serial connection and kill the serial communication thread
                    self.write_serial_monitor("Info:", "green", f"The serial connection to port \"{current_serial_port}\" has been closed successfully!")  # inform the user that the serial connection between the arduino microcontroller and the computer has been closed successfully
                    print("The serial connection has been closed successfully!")  # print a message to inform the user that the serial connection has been closed successfully
                else:  # if the serial connection is closed, then try to open it
                    try:  # try to establish the serial communication between the arduino microcontroller and the computer
                        self.serial_connection.port = self.serial_port  # change the serial port of the serial communication
                        self.serial_connection.baudrate = self.baudrate  # change the baudrate of the serial communication
                        self.serial_connection.timeout = 1  # set the timeout of the serial communication in seconds
                        self.serial_connection.close()  # close the serial connection/port
                        self.serial_connection.open()  # open the serial connection/port
                        self.start_serial_communication_thread()  # create and start a serial communication thread
                        self.write_serial_monitor("Info:", "green", f"The serial connection to port \"{self.serial_port}\" has been established successfully!")  # inform the user that the serial connection between the arduino microcontroller and the computer has been established successfully
                        self.update_serial_connection_indicators("Connected")  # update the serial connection indicator to the connected state
                        print("The serial connection has been established successfully!")  # print a message to inform the user that the serial connection has been established successfully
                    except Exception as e:  # if an error occurs
                        ms.showerror("Error", f"Error opening serial port \"{self.serial_port}\".", parent = self.menus_area)  # show an error message
                        print("Error opening serial port: " + str(e))  # print the error message
            else:  # if the baudrate is not chosen
                ms.showerror("Error", "Please choose the baudrate!", parent = self.menus_area)  # show an error message
        else:  # if the serial port is not chosen
            ms.showerror("Error", "Please choose the serial port!", parent = self.menus_area)  # show an error message
    def update_serial_connection_indicators(self, serial_connection_state):  # update the serial connection indicator
        try:
            self.serial_ports_combobox["values"] = self.available_serial_ports  # store the available serial ports as combobox values
            self.serial_ports_combobox.set(self.serial_port)  # set the value of the current serial port
            self.baudrates_combobox.set(self.baudrate)  # set the value of the current baudrate
            new_serial_connection_state = ""  # initialize the new serial connection state
            for index, string in enumerate(self.serial_connection_states_list):  # iterate through the serial connection states list
                if serial_connection_state in string and serial_connection_state != "" and serial_connection_state != " ":  # if the new serial connection state is in the current string
                    new_serial_connection_state = self.serial_connection_states_list[index]  # change the new serial connection state to the current string
            if new_serial_connection_state == "":  # if the new serial connection state is not an empty string
                new_serial_connection_state = ["Disconnected", "Connected to Port"][[True, False].index("disconnect" in serial_connection_state.lower())]  # change the new serial connection state to the disconnected or connected to port state
            self.serial_connection_state = new_serial_connection_state  # update the serial connection state
            self.serial_connection_indicator.configure(text = self.serial_connection_state, bg = self.serial_connection_indicator_colors[self.serial_connection_states_list.index(self.serial_connection_state)])  # update the serial connection indicator text and background color
            self.serial_connect_disconnect_command = ["Connect", "Disconnect"][[True, False].index("disconnect" in self.serial_connection_state.lower())]  # change the text of the serial connect/disconnect command to "Disconnect" or "Connect"
            self.serial_connect_button.configure(text = self.serial_connect_disconnect_command)  # change the text of the serial connect/disconnect button to the text of the serial connect/disconnect command
        except Exception as e:
            print(f"Error in update_serial_connection_indicators: {e}")
    def send_serial_command(self, command, event = None):  # send a command through the serial connection to be executed
        try:  # try to send the command through the serial connection
            if self.serial_connection.is_open:  # if the serial connection is open
                if self.allow_sending_all_ports or self.serial_connection_state != "Connected to Port":  # if the user is allowed to send commands to all serial ports or the user is connected to the wrong serial port
                    try:  # try to write the command to the serial monitor
                        self.write_serial_monitor(">>>", "blue", command)  # write the command to the serial monitor
                        self.command_entrybox.delete(0, "end")  # clear the console entry box
                    except: pass
                    print(">>> " + command)  # print the command
                    serial_command = command + "\n"  # add a newline character to the command to be sent
                    self.serial_connection.write(serial_command.encode("utf-8"))  # send the serial command through the opened serial port to the arduino microcontroller in order to be executed
                else:  # if the user is not allowed to send commands to all serial ports and the user is connected to the wrong serial port
                    if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[0]:  # if the robotic manipulator is controlled manually
                        try:  # try to write the command to the serial monitor
                            self.write_serial_monitor("Warn:", "brown", f"The command \"{command}\" could not be sent to the arduino microcontroller! You are connected to the wrong serial port or/and you are using the wrong baudrate!")  # inform the user that the command could not be sent to the arduino microcontroller
                        except: pass
                        print("You are connected to the wrong serial port or/and you are using the wrong baudrate!")  # print a message to inform the user that the command could not be sent to the arduino microcontroller
                        self.allow_sending_all_ports = ms.askyesno("Asking permission to send commands", f"Do you want to allow sending commands to the serial port \"{self.serial_port}\" with {self.baudrate} bps baudrate?")  # ask the user's permission to send commands to this serial port with this baudrate
                        if not self.allow_sending_all_ports and self.robot_control_thread_flag:  # if the user does not allow sending commands to this serial port with this baudrate and the robot control thread is running
                            self.kill_robot_control_thread()  # kill the robot control thread
            else:  # if the user tries to send a command when there is no serial connection established yet
                if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[0]:  # if the robotic manipulator is controlled manually
                    try:  # try to write the command to the serial monitor
                        self.write_serial_monitor("Warn:", "brown", f"The command \"{command}\" was not sent because there is no serial connection established yet!")  # inform the user that there is no serial connection established yet
                    except: pass
                    ms.showerror("Error", "There is no serial connection! You should first establish a serial connection before trying to send commands!", parent = self.menus_area)  # show an error message to the user that there is no serial connection
        except Exception as e:  # if an error occurs
            self.close_serial_connection()  # close the serial connection and kill the serial communication thread
            self.write_serial_monitor("Error:", "red", "The serial connection has been closed due to an error!")  # inform the user that the serial connection has been closed due to an error
            print("Error: " + str(e))  # print the error message
    def emit_serial_signals_to_console(self, emitted_message):  # emit the serial signals coming from the arduino microcontroller to the console
        try:  # try to emit the serial signals to the console
            if emitted_message == self.closed_serial_connection_message:  # if the emitted message is the closed serial connection message
                self.close_serial_connection()  # close the serial connection and kill the serial communication thread
                self.write_serial_monitor("Error:", "red", "The serial connection has been closed due to an error!")  # inform the user that the serial connection has been closed due to an error
            else:  # if the emitted message is not the closed serial connection message
                message_has_status = "MPos" in emitted_message  # check if the emitted message has the status of the robotic manipulator
                message_has_ok = "ok" in emitted_message  # check if the emitted message has the ok response
                if message_has_status:  # if the emitted message has the status of the robotic manipulator
                    status = emitted_message[1:].split(",")[0]  # get the status of the robotic manipulator
                    if not self.expanded_serial_monitor:  # if the serial monitor is expanded
                        self.update_serial_connection_indicators(status)  # update the serial connection indicator to the status of the robotic manipulator
                if not message_has_status and not message_has_ok:  # if the emitted message is not a status message or an ok response
                    self.write_serial_monitor("<<<", "purple", emitted_message)  # write the emitted message to the serial monitor
                elif message_has_ok and self.show_ok_responses:  # if the emitted message has the ok response and the user wants to see the ok responses
                    self.write_serial_monitor("<<<", "purple", emitted_message)  # write the emitted message to the serial monitor
                elif message_has_status and self.show_status_responses:  # if the emitted message has the status of the robotic manipulator and the user wants to see the status responses
                    self.write_serial_monitor("<<<", "purple", emitted_message)  # write the emitted message to the serial monitor
        except Exception as e:
            print("Error: " + str(e))  # print the error message
    def close_serial_connection(self, event = None):  # close the serial connection and kill the serial communication thread
        self.kill_serial_communication_thread()  # kill the existed and running serial communication thread
        self.serial_connection.close()  # close the serial connection/port
        self.update_serial_connection_indicators("Disconnected")  # update the serial connection indicator to the disconnected state
        self.allow_sending_all_ports = False  # allow the user to send commands to all serial ports
    def write_serial_monitor(self, message_tag, note_color, message):  # write a message to the console serial monitor
        self.text_pointer = self.console_serial_monitor.index('end')  # get the current text pointer of the console serial monitor
        if message_tag == ">>>":  # if the message is a command
            console_current_text = "\n" + message_tag + " " + message
        else:  # if the message is not a command (it explains something to the user, it is an info, a warning or an error message etc.)
            console_current_text = "\n" + message_tag + " " + message
        self.console_serial_monitor.insert("end", console_current_text)  # write the message to the console serial monitor
        self.console_serial_monitor.tag_add("{}".format(message_tag), str(float(self.text_pointer)), format(float(self.text_pointer) + float(len(message_tag) / 100), ".2f"))  # add a tag to the message
        self.console_serial_monitor.tag_configure("{}".format(message_tag), foreground = note_color, font = "Arial 10 bold italic")  # configure the tag
        self.serial_monitor_text = self.console_serial_monitor.get("1.0", "end")  # get the text of the console serial monitor
        self.console_serial_monitor.see("end")  # make the console serial monitor to scroll to the end
    def change_command_starting_text(self, event = None):  # change the starting text of the command to be sent to the arduino microcontroller
        self.command_starting_text = self.command_starting_text_entrybox.get()  # change the starting text of the command to be sent to the arduino microcontroller
        self.write_serial_monitor("Info:", "green", "The starting text of the command has been changed to \"{}\".".format(self.command_starting_text))  # inform the user that the starting text of the command has been changed
    def change_command_ending_text(self, event = None):  # change the ending text of the command to be sent to the arduino microcontroller
        self.command_ending_text = self.command_ending_text_entrybox.get()  # change the ending text of the command to be sent to the arduino microcontroller
        self.write_serial_monitor("Info:", "green", "The ending text of the command has been changed to \"{}\".".format(self.command_ending_text))  # inform the user that the ending text of the command has been changed
    def show_hide_ok_responses_on_console(self, event = None):  # show or hide the ok responses on the console serial monitor
        self.show_ok_responses = not self.show_ok_responses  # show or hide the ok responses on the console serial monitor
        self.show_hide_ok_button.configure(text = self.show_responses_indicators[[False, True].index(self.show_ok_responses)])  # change the text of the show/hide ok responses button
        self.write_serial_monitor("Info:", "green", "The ok responses are now {}.".format(["hidden", "shown"][[False, True].index(self.show_ok_responses)]))  # inform the user that the ok responses are now shown or hidden
    def show_hide_status_responses_on_console(self, event = None):  # show or hide the status responses on the console serial monitor
        self.show_status_responses = not self.show_status_responses  # show or hide the status responses on the console serial monitor
        self.write_serial_monitor("Info:", "green", "The status responses are now {}.".format(["hidden", "shown"][[False, True].index(self.show_status_responses)]))  # inform the user that the status responses are now shown or hidden
        self.show_hide_status_button.configure(text = ["✖", "✔"][[False, True].index(self.show_status_responses)])  # change the text of the show/hide status responses button
    def expand_serial_monitor_menu(self, event = None):  # expand the serial monitor menu
        self.expanded_serial_monitor = not self.expanded_serial_monitor  # expand or collapse the serial monitor menu
        self.build_control_robotic_manipulator_menus(self.submenus_titles[self.main_menu_choice], self.submenus_descriptions[self.main_menu_choice])  # create the control robotic manipulator menus
    def clear_serial_monitor(self, event = None):  # clear the console serial monitor
        self.console_serial_monitor.delete("1.0", "end")  # clear the console serial monitor
        self.serial_monitor_text = ""  # clear the serial monitor text
    def change_chosen_joint_number_control(self, event = None):  # change the chosen control joint number of the robotic manipulator
        self.chosen_joint_number_control = int(self.choose_joint_number_control_combobox.get().split(" ")[-1])  # change the chosen control joint number of the robotic manipulator
        self.chosen_joint_motor_number = 0  # the chosen motor number of the chosen control joint
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_chosen_joint_number_control_2(self, event = None):  # change the chosen control joint number of the robotic manipulator
        self.chosen_joint_number_control = self.joint_control_variable_combobox.current() + 1  # change the chosen control joint number of the robotic manipulator
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def copy_fkine_to_control_values(self, event = None):  # copy the forward kinematics variables to the control variables
        self.control_joints_variables = copy.deepcopy(self.forward_kinematics_variables)  # copy the forward kinematics variables to the control variables 
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_chosen_joint_motors()  # send the right command to the chosen control joint motors
    def set_joints_variables_to_zero(self, event = None):  # set the control joints variables of the robotic manipulator to zero
        for joint_number in range(1, self.joints_number + 1):  # iterate through the joints of the robotic manipulator
            self.control_joints_variables[joint_number - 1] = 0  # set the control joint variable of the chosen control joint to zero
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_all_motors()  # send the right command to all motors
    def change_control_variable_slider(self, event = None):  # change the control variable of the chosen joint of the robotic manipulator
        joint_control_variable = float(self.joint_variable_slider.get())  # the control variable of the chosen control joint
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        self.control_joints_variables[self.chosen_joint_number_control - 1] = [np.deg2rad(joint_control_variable), joint_control_variable][joint_type_index]  # change the control variable of the chosen control joint
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_chosen_joint_motors()  # send the right command to the chosen control joint motors
    def change_chosen_control_variable(self, event = None):  # change the control variable of the chosen joint of the robotic manipulator
        try:  # try to get the entered value
            joint_control_variable = float(self.joint_control_variable_combobox.get())  # the control variable of the chosen control joint
        except:  # if the entered value is not a number
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
            return  # return to the main loop
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        self.control_joints_variables[self.chosen_joint_number_control - 1] = [np.deg2rad(joint_control_variable), joint_control_variable][joint_type_index]  # change the control variable of the chosen control joint
        control_variable_limits = [np.deg2rad(self.control_joints_variables_limits[self.chosen_joint_number_control - 1][0]).tolist(), self.control_joints_variables_limits[self.chosen_joint_number_control - 1][1]][joint_type_index]  # the control variable limits of the chosen control joint
        if joint_control_variable < self.control_joints_variables_limits[self.chosen_joint_number_control - 1][joint_type_index][0]:  # if the entered value is less than the joint variable minimum limit
            self.control_joints_variables[self.chosen_joint_number_control - 1] = control_variable_limits[0]  # change the control variable of the chosen control joint to the minimum limit
        elif joint_control_variable > self.control_joints_variables_limits[self.chosen_joint_number_control - 1][joint_type_index][1]:  # if the entered value is greater than the joint variable maximum limit
            self.control_joints_variables[self.chosen_joint_number_control - 1] = control_variable_limits[1]  # change the control variable of the chosen control joint to the maximum limit
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_chosen_joint_motors()  # send the right command to the chosen control joint motors
    def increase_decrease_control_variable(self, change_type, event = None):  # increase or decrease the control variable of the chosen joint of the robotic manipulator
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        self.control_joints_variables[self.chosen_joint_number_control - 1] += np.sign(change_type) * [(10**(-self.angles_precision) * np.deg2rad([1, 10, 100])).tolist(), (10**(-self.distances_precision) * np.array([1, 10, 100])).tolist()][joint_type_index][np.abs(change_type) - 1]  # increase or decrease the control variable of the chosen control joint by the correct ammount
        control_variable_limits = [np.deg2rad(self.control_joints_variables_limits[self.chosen_joint_number_control - 1][0]).tolist(), self.control_joints_variables_limits[self.chosen_joint_number_control - 1][1]][joint_type_index]  # the control variable limits of the chosen control joint
        if self.control_joints_variables[self.chosen_joint_number_control - 1] < control_variable_limits[0]:  # if the entered value is less than the joint variable minimum limit
            self.control_joints_variables[self.chosen_joint_number_control - 1] = control_variable_limits[0]  # change the control variable of the chosen control joint to the minimum limit
        elif self.control_joints_variables[self.chosen_joint_number_control - 1] > control_variable_limits[1]:  # if the entered value is greater than the joint variable maximum limit
            self.control_joints_variables[self.chosen_joint_number_control - 1] = control_variable_limits[1]  # change the control variable of the chosen control joint to the maximum limit
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_chosen_joint_motors()  # send the right command to the chosen control joint motors
    def change_chosen_joint_motors(self, event = None):  # change the chosen control joint motors
        self.chosen_joint_motor_number = self.joints_motors_list[self.chosen_joint_number_control - 1].index(self.choose_joint_motors_combobox.get())  # the current chosen control joint motor
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def increase_chosen_joint_motors(self, event = None):  # increase the chosen control joint motors
        ask_added_joint_motor = sd.askstring(f"Increase the joint {self.chosen_joint_number_control} motors", "Enter the motor's name to be added:", initialvalue = "", parent = self.menus_area)  # ask the user to enter the new motor's name
        if ask_added_joint_motor != None:  # if the user enters a name
            if ask_added_joint_motor not in self.joints_motors_list[self.chosen_joint_number_control - 1]:  # if the new motor's name is not already in the list of the chosen control joint motors
                self.joints_motors_list[self.chosen_joint_number_control - 1].append(ask_added_joint_motor)  # add the new motor's name to the list of the chosen control joint motors
                self.joints_motors_mult_factors[self.chosen_joint_number_control - 1].append(1)  # add the new motor's multiplication factor to the list of the chosen control joint motors
                self.chosen_joint_motor_number = 0  # initialize the chosen motor number of the chosen control joint
            else:
                ms.showinfo("Info", f"The motor's name \"{ask_added_joint_motor}\" is already in the list of the chosen control joint motors.", parent = self.menus_area)
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def decrease_chosen_joint_motors(self, event = None):  # decrease the chosen control joint motors
        if len(self.joints_motors_list[self.chosen_joint_number_control - 1]) > 1:  # if there are more than one motors in the list of the chosen control joint motors
            ask_deleted_joint_motor = sd.askstring(f"Decrease the joint {self.chosen_joint_number_control} motors", "Enter the motor's name to be deleted:", initialvalue = "", parent = self.menus_area)  # ask the user to enter the motor's name to be deleted
            if ask_deleted_joint_motor != None:  # if the user enters a name
                if ask_deleted_joint_motor in self.joints_motors_list[self.chosen_joint_number_control - 1]:  # if the motor's name is in the list of the chosen control joint motors and there are more than one motors
                    self.joints_motors_mult_factors[self.chosen_joint_number_control - 1].pop(self.joints_motors_list[self.chosen_joint_number_control - 1].index(ask_deleted_joint_motor))  # remove the motor's multiplication factor from the list of the chosen control joint motors
                    self.joints_motors_list[self.chosen_joint_number_control - 1].pop(self.joints_motors_list[self.chosen_joint_number_control - 1].index(ask_deleted_joint_motor))  # remove the motor's name from the list of the chosen control joint motors
                    self.chosen_joint_motor_number = 0  # initialize the chosen motor number of the chosen control joint
                else:
                    ms.showinfo("Info", f"The motor's name \"{ask_deleted_joint_motor}' is not in the list of the chosen control joint motors.", parent = self.menus_area)
        else:
            ms.showinfo("Info", "There is only one motor in the list of the chosen control joint motors, so it can not be deleted. I you want to change its name, you should first add a new motor with the desired name and then delete the old one.", parent = self.menus_area)
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_motors_mult_factors_entrybox(self, event = None):  # change the multiplication factors of the chosen control joint motors
        try:  # try to change the multiplication factors of the chosen control joint motors
            self.joints_motors_mult_factors[self.chosen_joint_number_control - 1][self.chosen_joint_motor_number] = float(self.motors_mult_factors_entrybox.get())  # change the multiplication factors of the chosen control joint motors
        except:  # if an error occurs
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_control_end_effector_slider(self, event = None):  # change the control end-effector state of the robotic manipulator
        self.control_end_effector_variable = float(self.end_effector_slider.get())  # the control end-effector variable of the robotic manipulator
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_end_effector()  # send the right command to the end-effector motors
    def set_end_effector_to_zero(self, event = None):  # set the control end-effector state of the robotic manipulator to zero
        self.control_end_effector_variable = 0  # set the control end-effector variable to zero
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_end_effector()  # send the right command to all motors
    def change_end_effector_motor(self, event = None):  # change the chosen control end-effector motor
        self.end_effector_motor = self.end_effector_motor_entrybox.get()  # change the chosen control end-effector motor
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_end_effector_mult_factor(self, event = None):  # change the multiplication factor of the chosen control end-effector motor
        try:  # try to change the multiplication factor of the control end-effector motor
            self.end_effector_motor_mult_factor = float(self.end_effector_mult_factor_entrybox.get())  # change the multiplication factor of the chosen control end-effector motor
        except:  # if an error occurs
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_robotic_manipulator_control_mode(self, event = None):  # change the control mode of the robotic manipulator
        self.robotic_manipulator_control_mode = self.alternate_matrix_elements(self.robotic_manipulator_control_modes_list, self.robotic_manipulator_control_mode)  # change the control mode of the robotic manipulator
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def send_command_to_chosen_joint_motors(self, event = None):  # send the control joints values (multiplied by some factors) to the chosen control joint motors
        command = ""  # the command to be sent to the console
        command_dict = {}  # initialize a dictionary for the command motors
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        for joint_number in range(1, self.joints_number + 1):  # iterate through the joints of the robotic manipulator
            joint_value = self.control_joints_variables[joint_number - 1]  # the value of the joint
            for k in range(len(self.joints_motors_list[joint_number - 1])):  # iterate through all the motors of the chosen control joint
                motor_name = self.joints_motors_list[joint_number - 1][k]  # the name of the motor
                motor_value = np.round([np.rad2deg(joint_value), joint_value][joint_type_index] * self.joints_motors_mult_factors[joint_number - 1][k], 2)  # the value of the motor
                if motor_name in self.joints_motors_list[self.chosen_joint_number_control - 1]:  # if the motor name is in the list of the chosen control joint motors
                    command_dict[motor_name] = command_dict.get(motor_name, 0) + motor_value  # combine the values of the same motors
        command = " ".join([motor + str(np.round(value, 2)) for motor, value in command_dict.items()])  # rewrite the command string
        command = self.command_starting_text + command + self.command_ending_text  # add the starting and ending texts to the command
        self.send_serial_command(command)  # send the command to the console to be executed
    def send_command_to_all_motors(self, event = None):  # send the control joints values (multiplied by some factors) to all motors
        command_dict = {}  # initialize a dictionary for the command motors
        for joint_number in range(1, self.joints_number + 1):  # iterate through the joints of the robotic manipulator
            joint_value = self.control_joints_variables[joint_number - 1]  # the value of the joint
            joint_type_index = self.joints_types_list.index(self.joints_types[joint_number - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
            for k in range(len(self.joints_motors_list[joint_number - 1])):  # iterate through all the motors of the chosen control joint
                motor_name = self.joints_motors_list[joint_number - 1][k]  # the name of the motor
                motor_value = np.round([np.rad2deg(joint_value), joint_value][joint_type_index] * self.joints_motors_mult_factors[joint_number - 1][k], 2)  # the value of the motor
                command_dict[motor_name] = command_dict.get(motor_name, 0) + motor_value  # combine the values of the same motors
        command = " ".join([motor + str(np.round(value, 2)) for motor, value in command_dict.items()])  # rewrite the command string for the joints motors
        # command += " " + self.end_effector_motor + str(np.round(self.control_end_effector_variable * self.end_effector_motor_mult_factor, 2))  # add the end-effector motor command to the joints motors command
        command = self.command_starting_text + command + self.command_ending_text  # add the starting and ending texts to the command
        self.send_serial_command(command)  # send the command to the console to be executed
    def send_command_to_end_effector(self, event = None):  # send the control end-effector values (multiplied by some factors) to the end-effector motors
        command = self.end_effector_motor + str(np.round(self.control_end_effector_variable * self.end_effector_motor_mult_factor, 2))  # the command to be sent to the console
        command = self.command_starting_text + command + self.command_ending_text  # add the starting and ending texts to the command
        self.send_serial_command(command)  # send the command to the console to be executed
    def update_control_variables_indicators(self, event = None):  # update the indicators of the control variables of the robotic manipulator
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                self.choose_joint_number_control_combobox.set(f"joint {self.chosen_joint_number_control}")  # set the value of the chosen joint number for the control
                joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
                joint_control_variable = [f"{np.rad2deg(self.control_joints_variables[self.chosen_joint_number_control - 1]):.1f}", f"{self.control_joints_variables[self.chosen_joint_number_control - 1]:.3f}"][joint_type_index]  # the control variable of the chosen control joint
                self.joint_type_indicator.configure(text = self.joints_types[self.chosen_joint_number_control - 1] + [" (degrees)", " (meters)"][joint_type_index])  # change the joint type indicator label to the chosen joint type
                self.joint_variable_slider.configure(resolution = [10**(-self.angles_precision), 10**(-self.distances_precision)][joint_type_index], \
                                                    from_ = self.control_joints_variables_limits[self.chosen_joint_number_control - 1][joint_type_index][0], to = self.control_joints_variables_limits[self.chosen_joint_number_control - 1][joint_type_index][1])  # set the slider limits to the current control joint variable limits
                self.joint_variable_slider.set(joint_control_variable)  # set the slider to the control variable of the chosen control joint
                self.joint_control_variable_combobox.set(joint_control_variable)  # set the combobox to the chosen joint variable for the control
                self.joint_control_variable_combobox["values"] = [f"{k + 1}: " + [f"{np.rad2deg(self.control_joints_variables[k]):.1f}", f"{self.control_joints_variables[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])] for k in range(self.joints_number)]  # update the values of the joint control variable combobox
                self.increase_joint_var_1_button.configure(text = ["+0.1", "+0.001"][joint_type_index])
                self.decrease_joint_var_1_button.configure(text = ["-0.1", "-0.001"][joint_type_index])
                self.increase_joint_var_2_button.configure(text = ["+1", "+0.01"][joint_type_index])
                self.decrease_joint_var_2_button.configure(text = ["-1", "-0.01"][joint_type_index])
                self.increase_joint_var_3_button.configure(text = ["+10", "+0.1"][joint_type_index])
                self.decrease_joint_var_3_button.configure(text = ["-10", "-0.1"][joint_type_index])
                self.choose_joint_motors_combobox["values"] = self.joints_motors_list[self.chosen_joint_number_control - 1]  # store the available control joint motors as combobox values
                self.choose_joint_motors_combobox.set(self.joints_motors_list[self.chosen_joint_number_control - 1][self.chosen_joint_motor_number])  # set the combobox to the current control joint motor
                self.motors_mult_factors_entrybox.delete(0, "end")  # clear the motors multiplication factors entry box
                self.motors_mult_factors_entrybox.insert(0, f"{self.joints_motors_mult_factors[self.chosen_joint_number_control - 1][self.chosen_joint_motor_number]:.3f}")  # insert the motors multiplication factors of the chosen control joint to the entry box
                self.end_effector_slider.set(self.control_end_effector_variable)  # set the slider to the control variable of the chosen control joint
                self.end_effector_motor_entrybox.delete(0, "end")  # clear the end-effector motor entry box
                self.end_effector_motor_entrybox.insert(0, self.end_effector_motor)  # set the combobox to the current control joint motor
                self.end_effector_mult_factor_entrybox.delete(0, "end")  # clear the end-effector motor multiplication factor entry box
                self.end_effector_mult_factor_entrybox.insert(0, f"{self.end_effector_motor_mult_factor:.3f}")  # insert the end-effector motor multiplication factor to the entry box
                self.choose_control_mode_button.configure(text = self.robotic_manipulator_control_mode)  # change the text of the appropriate button to the chosen control mode
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_control_variables_indicators: {e}")

    # the functions used for the serial communication thread between the arduino microcontroller and the computer
    def start_serial_communication_thread(self, event = None):  # create and start a serial communication thread
        self.serial_connection_thread_flag = True  # let the serial communication thread run
        self.serial_communication_thread = threading.Thread(target = self.serial_communication_thread_run_function)  # create the thread for the serial communication with the arduino microcontroller
        self.serial_communication_thread.start()  # start the serial communication thread
    def kill_serial_communication_thread(self, event = None):  # kill the existed and running serial communication thread
        self.serial_connection_thread_flag = False  # stop the serial communication thread
    def serial_communication_thread_run_function(self):  # the run function, it continuously checks for the data sent from the serial connection and emits signals for the messages and errors
        self.serial_connection_elapsed_time = time.time()  # the elapsed time since the serial communication thread started
        while True:  # while the thread is running
            if self.serial_connection.is_open:  # if the serial connection is open
                try:
                    bytes_waiting = self.serial_connection.in_waiting  # the number of bytes waiting to be read from the serial connection
                    if time.time() - self.serial_connection_elapsed_time > 1.0:  # if the elapsed time since the thread started is greater than 1.0 seconds
                        self.serial_connection_elapsed_time = time.time()  # update the elapsed time since the thread started
                        self.serial_connection.write("?\n".encode("utf-8"))  # send the command to show the status and position information
                    data_read = str(self.serial_connection.readline().decode("utf-8")).strip()  # read the data from the serial connection
                    if data_read != "":  # if there is data read from the serial connection
                        self.emit_serial_signals_to_console(data_read)  # emit a message containing the data read from the serial connection
                    print("<<< " + data_read)  # print the data read from the serial connection
                except Exception as e:  # if an error occurs
                    self.emit_serial_signals_to_console(self.closed_serial_connection_message)  # emit a message that the serial connection is closed
                    print(self.closed_serial_connection_message + " Error: " + str(e))  # print that the serial connection is closed along with the error message
            if not self.serial_connection_thread_flag:  # if the thread is stopped
                break  # break the while loop

    # for the main menu where the camera is controlled
    def determine_camera(self, event = None):  # determine the camera to use
        self.camera_choice = self.alternate_matrix_elements(self.camera_choices_list, self.camera_choice)  # change the camera choice
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def change_camera_aspect_ratio(self, event = None):  # choose the camera aspect ratio
        self.camera_aspect_ratio = self.camera_aspect_ratios_values[self.camera_aspect_ratios_list.index(self.choose_aspect_ratio_combobox.get())]  # change the camera aspect ratio
        self.camera_resolution = self.camera_resolutions_list[self.camera_aspect_ratios_values.index(self.camera_aspect_ratio)][0]  # change the camera resolution
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def change_camera_resolution(self, event = None):  # choose the camera resolution
        self.camera_resolution = int(self.choose_resolution_combobox.get())  # change the camera resolution
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def change_windows_size_factor_slider(self, event = None):  # change the windows size factor
        self.images_size_factor = self.choose_windows_size_factor_slider.get() / 100.0  # change the windows size factor
    def change_camera_dFoV(self, event = None):  # choose the camera diagonal field of view
        camera_dFoV = sd.askfloat("Camera diagonal field of view", "Enter the diagonal field of view\nof the camera (in degrees):", initialvalue = self.camera_dFoV, minvalue = 10, maxvalue = 180, parent = self.menus_area)  # ask the user to enter the diagonal field of view of the camera
        if camera_dFoV != None:  # if the user enters a number
            self.camera_dFoV = camera_dFoV  # change the diagonal field of view of the camera
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def change_luminance_threshold_slider(self, event = None):
        self.luminance_threshold = self.choose_luminance_threshold_slider.get()  # change the luminance threshold
    def open_close_camera(self, event = None):  # open or close the camera
        if not self.camera_thread_flag:  # if the camera thread is not running
            self.start_camera_thread()  # create and start a camera thread
            if event != None or (self.original_image_is_shown or self.grayscale_image_is_shown or self.camera_pose_estimation_happening or self.obst_plane_pose_estimation_happening or self.draw_2d_plane_on_image_happening):
                self.create_camera_capture_object()  # create the camera capture object
        elif self.camera_thread_flag:  # if the camera thread is running
            if not self.camera_frames_are_shown and (self.original_image_is_shown or self.grayscale_image_is_shown or self.camera_pose_estimation_happening or self.obst_plane_pose_estimation_happening or self.draw_2d_plane_on_image_happening):
                self.create_camera_capture_object()  # create the camera capture object
            if event != None:  # if the open/close camera is called by an event (the button is pressed)
                self.kill_camera_thread()
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def create_camera_capture_object(self, event = None):  # create the camera capture object
        try:  # try to open the camera
            self.camera_capture = cv2.VideoCapture([1, 0][self.camera_choices_list.index(self.camera_choice)], cv2.CAP_DSHOW)  # initialize the camera capture object, 0 for the default camera, 1 for the first external camera
            self.camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_resolution)  # set the height of the captured frame
            self.camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.camera_aspect_ratio * self.camera_resolution))  # set the width of the captured frame
            self.camera_frames_are_shown = True  # the camera frames are shown from now on
            print("The camera has been opened successfully!")  # print a message to inform the user that the camera has been opened successfully
        except Exception as e:  # if an error occurs
            self.camera_frames_are_shown = False  # the camera frames are not shown from now on
            ms.showerror("Error", f"Error opening the camera: {e}", parent = self.menus_area)  # show an error message
            print(f"Error opening the camera: {e}")  # print the error message
    def show_grayscale_image(self, event = None):  # show the grayscale image
        if not self.grayscale_image_is_shown:  # if the grayscale image is not shown
            self.grayscale_image_is_shown = True  # the grayscale image is shown from now on
            self.open_close_camera()  # open the camera
    def choose_camera_optical_axis(self, event = None):  # choose the optical axis of the camera
        optical_axis_x = sd.askfloat("Camera optical axis", "Enter the x component of the optical axis\nof the camera (in meters):", initialvalue = self.camera_optical_axis[0], parent = self.menus_area)  # ask the user to enter the x component of the optical axis of the camera
        if optical_axis_x != None:  # if the user enters a number
            self.camera_optical_axis[0] = optical_axis_x  # change the x component of the optical axis of the camera
        optical_axis_y = sd.askfloat("Camera optical axis", "Enter the y component of the optical axis\nof the camera (in meters):", initialvalue = self.camera_optical_axis[1], parent = self.menus_area)  # ask the user to enter the y component of the optical axis of the camera
        if optical_axis_y != None:  # if the user enters a number
            self.camera_optical_axis[1] = optical_axis_y  # change the y component of the optical axis of the camera
        optical_axis_z = sd.askfloat("Camera optical axis", "Enter the z component of the optical axis\nof the camera (in meters):", initialvalue = self.camera_optical_axis[2], parent = self.menus_area)  # ask the user to enter the z component of the optical axis of the camera
        if optical_axis_z != None:  # if the user enters a number
            self.camera_optical_axis[2] = optical_axis_z  # change the z component of the optical axis of the camera
        if np.linalg.norm(self.camera_optical_axis) != 0:  # if the optical axis of the camera is not the zero vector
            self.camera_optical_axis = self.camera_optical_axis / np.linalg.norm(self.camera_optical_axis)  # normalize the optical axis of the camera
        else:  # if the optical axis of the camera is the zero vector
            self.camera_optical_axis = np.array([0, 0, 1])  # set the optical axis of the camera to the default value
        self.reset_camera_sliders()  # initialize the sliders variables for the camera
    def choose_camera_translation(self, event = None):  # choose the translation of the camera
        camera_translation_x = sd.askfloat("Camera translation", "Enter the x component of the translation\nof the camera (in meters):", initialvalue = self.camera_translation[0], parent = self.menus_area)  # ask the user to enter the x component of the translation of the camera
        if camera_translation_x != None:  # if the user enters a number
            self.camera_translation[0] = camera_translation_x  # change the x component of the translation of the camera
        camera_translation_y = sd.askfloat("Camera translation", "Enter the y component of the translation\nof the camera (in meters):", initialvalue = self.camera_translation[1], parent = self.menus_area)  # ask the user to enter the y component of the translation of the camera
        if camera_translation_y != None:  # if the user enters a number
            self.camera_translation[1] = camera_translation_y  # change the y component of the translation of the camera
        camera_translation_z = sd.askfloat("Camera translation", "Enter the z component of the translation\nof the camera (in meters):", initialvalue = self.camera_translation[2], parent = self.menus_area)  # ask the user to enter the z component of the translation of the camera
        if camera_translation_z != None:  # if the user enters a number
            self.camera_translation[2] = camera_translation_z  # change the z component of the translation of the camera
        self.reset_camera_sliders()  # initialize the sliders variables for the camera
    def choose_camera_orientation(self, event = None):  # choose the orientation of the camera
        orientation_angle = sd.askfloat("Camera orientation", "Enter the orientation angle of the camera,\naround its optical axis (in degrees):", initialvalue = self.camera_orientation, minvalue = -180, maxvalue = 180, parent = self.menus_area)  # ask the user to enter the angle of the orientation of the camera
        if orientation_angle != None:  # if the user enters a number
            self.camera_orientation = orientation_angle  # change the angle of the orientation of the camera
        self.update_camera_control_indicators()  # update the indicators of the camera control variables
    def change_camera_move_sliders(self, event = None):  # change the rotation of the camera around the z axis
        self.camera_z_rotation = np.deg2rad(self.z_rotate_camera_slider.get())  # the rotation angle of the camera around the z axis
        self.camera_translation_displacement = self.normal_translate_camera_slider.get()  # the translation displacement of the camera along its optical axis
        Z_rot_mat = np.array([[np.cos(self.camera_z_rotation), -np.sin(self.camera_z_rotation), 0], [np.sin(self.camera_z_rotation), np.cos(self.camera_z_rotation), 0], [0, 0, 1]])  # the rotation matrix around the z axis
        self.camera_optical_axis = Z_rot_mat @ self.camera_optical_axis_prev  # change the optical axis of the camera
        self.camera_translation = Z_rot_mat @ (self.camera_translation_prev + self.camera_translation_displacement * self.camera_optical_axis_prev)  # change the translation of the camera
        self.update_camera_control_indicators()  # update the indicators of the camera control variables
    def reset_camera_sliders(self, event = None):  # reset the sliders variables for the camera
        self.camera_optical_axis_prev = np.copy(self.camera_optical_axis)  # reset the optical axis of the camera
        self.camera_translation_prev = np.copy(self.camera_translation)  # reset the translation of the camera
        self.camera_z_rotation = 0; self.camera_translation_displacement = 0  # initialize the sliders variables for the camera
        self.update_camera_control_indicators()  # update the indicators of the camera control variables
    def change_ArUco_marker(self, event = None):  # change the ArUco marker
        chosen_ArUco_marker = self.choose_ArUco_marker_combobox.get()  # change the ArUco marker
        if chosen_ArUco_marker in self.saved_ArUco_markers_list:  # if the chosen ArUco marker exists
            self.chosen_ArUco_marker = chosen_ArUco_marker  # change the chosen ArUco marker
        else:  # if the chosen ArUco marker does not exist
            ms.showerror("Error", f"The chosen ArUco marker \"{chosen_ArUco_marker}\" does not exist!", parent = self.menus_area)  # show an error message
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def estimate_camera_pose(self, event = None):  # estimate the pose of the camera
        if not self.camera_pose_estimation_happening and not self.obst_plane_pose_estimation_happening:  # if the camera pose estimation is not happening
            self.camera_aspect_ratio = self.camera_aspect_ratios_values[0]; self.camera_resolution = self.camera_resolutions_list[0][0]  # set the values of the camera resolution and aspect ratio
            self.camera_pose_estimation_happening = True  # the camera pose estimation is happening from now on
            self.open_close_camera()  # open the camera
    def estimate_obstacles_plane_pose(self, event = None):  # estimate the pose of the obstacles plane
        if not self.obst_plane_pose_estimation_happening and not self.camera_pose_estimation_happening:  # if the obstacles plane pose estimation is not happening
            self.obst_plane_pose_estimation_happening = True  # the obstacles plane pose estimation is happening from now on
            self.open_close_camera()  # open the camera
    def apply_moving_average_filter(self, event = None):  # apply the moving average filter for the camera and plane poses estimation
        self.apply_moving_average_poses_estimation = not self.apply_moving_average_poses_estimation  # change the flag of applying the moving average filter
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def recalibrate_camera(self, event = None):  # recalibrate the camera
        calibration_images_paths = self.saved_camera_calibration_images_folder_path + r"/*.jpg"  # the paths of the calibration images
        self.camera_intrinsic_matrix, self.camera_dist_coeffs, self.reprojection_error = cd.calibrate_camera(calibration_images_paths)  # calibrate the camera
        if self.camera_intrinsic_matrix != None:  # if the camera has been calibrated successfully
            cd.save_intrinsic_parameters(self.camera_intrinsic_matrix, self.camera_dist_coeffs, self.reprojection_error, self.camera_intrinsic_parameters_file_path)  # save the intrinsic matrix and distortion coefficients of the camera
            ms.showinfo("Camera calibration", f"The camera has been calibrated successfully! The reprojection error is {np.round(self.reprojection_error, 5)} pixels!", parent = self.menus_area)  # show an info message
        else:  # if the camera has not been calibrated
            ms.showerror("Error", "The camera has not been calibrated! The previous calibration parameters are used.", parent = self.menus_area)  # show an error message
    def show_camera_parameters(self, event = None):  # show the camera parameters
        ms.showinfo("Camera parameters", f"• Intrinsic matrix (in pixels):\n{self.camera_intrinsic_matrix}\n\n\
• Distortion coefficients (dimensionless):\n\
  - Radial distortion coefficients k1, k2, k3: {np.hstack((self.camera_dist_coeffs[:2], self.camera_dist_coeffs[-1]))}\n\
  - Tangential distortion coefficients p1, p2: {self.camera_dist_coeffs[2:4]}\n\n\
• Reprojection error (in pixels): {self.reprojection_error}", parent = self.menus_area)  # show the intrinsic matrix and distortion coefficients of the camera
    def draw_2d_plane_on_image(self, event = None):  # draw the 2D plane on the image
        if not self.draw_2d_plane_on_image_happening:  # if the 2D plane is not being drawn on the image
            self.draw_2d_plane_on_image_happening = True  # the 2D plane is being drawn on the image from now on
            self.open_close_camera()  # open the camera
    def change_shown_workspace_image(self, event = None):  # change the shown workspace image
        shown_workspace_image_name = self.show_workspace_image_combobox.get()  # the name of the shown workspace image
        if shown_workspace_image_name in self.saved_workspace_images_list:  # if the chosen workspace image exists
            self.shown_workspace_image_name = shown_workspace_image_name  # change the shown workspace image
            _, _, _, self.shown_workspace_image_plane_corners, _ = self.load_workspace_image_information(self.shown_workspace_image_name)  # load the information of the shown workspace image
        else:  # if the chosen workspace image does not exist
            ms.showerror("Error", f"The chosen workspace image \"{shown_workspace_image_name}\" does not exist!", parent = self.menus_area)  # show an error message
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def show_workspace_image(self, event = None):  # show the specified workspace image
        if self.shown_workspace_image_name in self.saved_workspace_images_list:  # if the chosen workspace image exists
            if not self.workspace_image_is_shown:  # if the workspace image is not shown
                self.workspace_image_is_shown = True  # the workspace image is shown from now on
                self.open_close_camera()  # open the camera
        else:  # if the chosen workspace image does not exist
            ms.showerror("Error", f"The chosen workspace image \"{self.shown_workspace_image_name}\" does not exist!", parent = self.menus_area)
    def rename_workspace_image(self, event = None):  # rename the specified workspace image
        renamed_image = self.show_workspace_image_combobox.get()  # the name of the chosen workspace image
        if renamed_image != "":  # if the user chooses a workspace image
            new_image_name = sd.askstring("Rename workspace image", f"Enter the new name of the workspace image \"{renamed_image}\":", initialvalue = renamed_image, parent = self.menus_area)  # ask the user to enter the new name of the workspace image
            if new_image_name != None:  # if the user enters a name
                if new_image_name not in self.saved_workspace_images_list:  # if the new name of the workspace image does not exist
                    os.rename(self.saved_workspace_images_infos_folder_path + fr"/{renamed_image}.jpg", self.saved_workspace_images_infos_folder_path + fr"/{new_image_name}.jpg")  # rename the workspace image
                    os.rename(self.saved_workspace_images_infos_folder_path + fr"/{renamed_image}_info.txt", self.saved_workspace_images_infos_folder_path + fr"/{new_image_name}_info.txt")  # rename the information file of the workspace image
                    os.rename(self.saved_obstacles_objects_infos_folder_path + fr"/{renamed_image}", self.saved_obstacles_objects_infos_folder_path + fr"/{new_image_name}")  # rename the folder of the obstacles objects detected in the workspace image
                    self.saved_workspace_images_list[self.saved_workspace_images_list.index(renamed_image)] = new_image_name  # change the name of the chosen workspace image
                    self.shown_workspace_image_name = new_image_name  # show the renamed workspace image
                    self.chosen_detection_workspace_image_name = new_image_name  # choose the renamed workspace image as the detection workspace image
                    if self.chosen_solver_workspace_image_name == renamed_image:  # if the chosen solver workspace image is the renamed image
                        self.chosen_solver_workspace_image_name = new_image_name  # choose the renamed workspace image as the solver workspace image
                    self.update_camera_control_indicators()  # update the indicators of the camera control
                else:  # if the new name of the workspace image already exists
                    ms.showerror("Error", f"The new name of the workspace image \"{new_image_name}\" already exists!", parent = self.menus_area)  # show an error message
        else:  # if the user does not choose a workspace image
            ms.showerror("Error", "Choose a workspace image to rename!", parent = self.menus_area)
    def delete_workspace_image(self, event = None):  # delete the specified workspace image
        self.workspace_image_is_shown = False  # the workspace image is not shown from now on
        deleted_image = self.show_workspace_image_combobox.get()  # the name of the chosen workspace image
        if deleted_image != "":  # if the user chooses a workspace image
            delete_image_accept = ms.askyesno("Delete workspace image", f"Are you sure you want to delete\nthe workspace image \"{deleted_image}\"?", parent = self.menus_area)  # ask the user if they want to delete the chosen workspace image
            if delete_image_accept:  # if the user wants to delete the chosen workspace image
                os.remove(self.saved_workspace_images_infos_folder_path + fr"/{deleted_image}.jpg")  # delete the chosen workspace image
                os.remove(self.saved_workspace_images_infos_folder_path + fr"/{deleted_image}_info.txt")  # delete the information file of the chosen workspace image
                self.saved_workspace_images_list.remove(deleted_image)  # remove the chosen workspace image from the list of the saved workspace images
                if len(self.saved_workspace_images_list) != 0:  # if there are saved workspace images
                    self.shown_workspace_image_name = self.saved_workspace_images_list[0]  # show the first saved workspace image
                    self.chosen_detection_workspace_image_name = self.saved_workspace_images_list[0]  # choose the first saved workspace image as the detection workspace image
                else:  # if there are no saved workspace images
                    self.shown_workspace_image_name = ""  # do not show any workspace image
                    self.chosen_detection_workspace_image_name = ""  # do not choose any workspace image
                if self.chosen_solver_workspace_image_name == deleted_image:  # if the chosen solver workspace image is the deleted image
                    self.chosen_solver_workspace_image_name = self.chosen_detection_workspace_image_name  # choose the first saved workspace image as the solver workspace image
                shutil.rmtree(self.saved_obstacles_objects_infos_folder_path + fr"/{deleted_image}")  # delete the folder of the obstacles objects detected in the workspace image
        self.update_camera_control_indicators()  # update the indicators of the camera control
    def capture_workspace_image(self, event = None):  # capture the workspace image using the camera
        if not self.draw_2d_plane_on_image_happening:  # if the 2D plane is not being drawn on the image
            self.grayscale_image_is_shown = True  # the grayscale image is shown from now on
            self.obst_plane_pose_estimation_happening = True  # the obstacles plane pose estimation is happening from now on
            self.draw_2d_plane_on_image_happening = True  # the 2D plane is being drawn on the image from now on
            ms.showinfo("Workspace image capture instructions", "Capture the workspace image! The camera frame is considered to be fixed. The 2D plane pose is being estimated in order to be saved, along with the dimensions of the plane. \
Press the \"s\" key to save the image (in grayscale format, the whole workspace 2D plane must be inside the frame)!", parent = self.menus_area)  # show an info message
            self.open_close_camera()  # open the camera
    def save_workspace_image_information(self, image, plane_frame_wrt_world, plane_x_length, plane_y_length, plane_corners_image_points, camera_frame_wrt_world, event = None):  # save the workspace image information
        workspace_image_name = f"image_{len(self.saved_workspace_images_list) + 1}"  # the name of the workspace image
        while os.path.exists(self.saved_workspace_images_infos_folder_path + fr"/{workspace_image_name}.jpg"):  # while the file with the same name exists
            workspace_image_name = f"image_{int(workspace_image_name.split('_')[-1]) + 1}"  # change the name of the workspace image
        grayscale_image = np.array(Image.fromarray(image).convert("L").point(lambda pixel: cd.convert_image_pixel_to_grayscale(pixel, [self.luminance_threshold])))  # convert the captured frame to black and white
        cv2.imwrite(self.saved_workspace_images_infos_folder_path + fr"/{workspace_image_name}.jpg", grayscale_image)
        with open(self.saved_workspace_images_infos_folder_path + fr"/{workspace_image_name}_info.txt", "w", encoding = "utf-8") as file:  # open the file in write mode
            file.write("Plane frame wrt world transformation matrix:\n")  # write the transformation matrix of the plane frame wrt world to the file
            np.savetxt(file, plane_frame_wrt_world, fmt = "%.5f", delimiter = " ")  # write the transformation matrix of the plane frame wrt world to the file
            file.write("\nPlane x length (m):\n")  # write the x length of the plane to the file
            file.write(f"{plane_x_length:.3f}\n")  # write the x length of the plane to the file
            file.write("Plane y length (m):\n")  # write the y length of the plane to the file
            file.write(f"{plane_y_length:.3f}\n")  # write the y length of the plane to the file
            file.write("\nPlane corners image points (top-left, top-right, bottom-right, bottom-left) in pixels:\n")  # write the image points of the plane corners to the file
            np.savetxt(file, np.int32(plane_corners_image_points), fmt = "%.0f", delimiter = " ")  # write the image points of the plane corners to the file
            file.write("\nCamera frame wrt world transformation matrix:\n")  # write the transformation matrix of the camera frame wrt world to the file
            np.savetxt(file, camera_frame_wrt_world, fmt = "%.5f", delimiter = " ")  # write the transformation matrix of the camera frame wrt world to the file
            file.close()  # close the file
        os.mkdir(self.saved_obstacles_objects_infos_folder_path + fr"/{workspace_image_name}")  # create a folder for the obstacles objects detected in the workspace image
        if workspace_image_name not in self.saved_workspace_images_list:  # if the name of the saved file is not in the values of the combobox that contains the names of the saved files
            self.saved_workspace_images_list.append(workspace_image_name)  # add the file name to the list of the saved files
        ms.showinfo("Workspace image information saved!", f"The workspace image and its information have been saved successfully in the files \"{workspace_image_name}.jpg\" and \"{workspace_image_name}_info.txt\" respectively!", parent = self.menus_area)  # show an info message
    def load_workspace_image_information(self, workspace_image_name, event = None):  # load the information of the workspace image
        if workspace_image_name in self.saved_workspace_images_list:  # if the chosen workspace image exists
            loaded_image_info_file = open(self.saved_workspace_images_infos_folder_path + fr"/{workspace_image_name}_info.txt", "r", encoding = "utf-8")  # open the information file of the chosen workspace image in read mode
            loaded_image_info_lines = loaded_image_info_file.readlines()  # read all the lines of the information file
            loaded_image_info_file.close()  # close the information file
            plane_frame_wrt_world = np.array([np.array([float(k) for k in line.split()]) for line in loaded_image_info_lines[1:5]])  # the transformation matrix of the plane frame wrt world
            plane_x_length = float(loaded_image_info_lines[7])  # the x length (in meters) of the plane
            plane_y_length = float(loaded_image_info_lines[9])  # the y length (in meters) of the plane
            plane_image_points = np.int32([np.array([float(k) for k in line.split()]) for line in loaded_image_info_lines[12:16]]).reshape(-1, 2)  # the image points (in pixels) of the plane corners
            camera_frame_wrt_world = np.array([np.array([float(k) for k in line.split()]) for line in loaded_image_info_lines[18:]])  # the transformation matrix of the camera frame wrt world
            return plane_frame_wrt_world, plane_x_length, plane_y_length, plane_image_points, camera_frame_wrt_world  # return the loaded information of the workspace image
        else:  # if the chosen workspace image does not exist
            ms.showerror("Error", f"The chosen workspace image \"{workspace_image_name}\" does not exist!", parent = self.menus_area)  # show an error message
            return np.eye(4), 0, 0, np.zeros((4, 2)), np.eye(4)  # return the default values
    def update_camera_control_indicators(self, event = None):  # update the indicators of the camera control variables
        self.camera_wrt_world_transformation_matrix = gf.xy_plane_transformation(self.camera_optical_axis, self.camera_orientation, self.camera_translation)  # the total transformation matrix of the camera
        try:
            self.determine_camera_button.configure(text = self.camera_choice)  # change the text of the determine camera button
            self.choose_resolution_combobox["values"] = self.camera_resolutions_list[self.camera_aspect_ratios_values.index(self.camera_aspect_ratio)]  # the available camera resolutions for the chosen camera aspect ratio
            self.choose_aspect_ratio_combobox.set(self.camera_aspect_ratios_list[self.camera_aspect_ratios_values.index(self.camera_aspect_ratio)])  # set the value of the camera aspect ratio combobox
            self.choose_resolution_combobox.set(self.camera_resolution)  # set the value of the camera resolution combobox
            self.choose_windows_size_factor_slider.set(int(100.0 * self.images_size_factor))  # set the value of the windows size factor slider
            self.choose_dFoV_button.configure(text = f"{self.camera_dFoV:.1f}")  # change the text of the choose camera dFoV button
            min_dist_camera_2d_plane = cd.min_distance_from_camera(self.obstacles_2d_plane_x_length, self.obstacles_2d_plane_y_length, int(self.camera_aspect_ratio * self.camera_resolution), self.camera_resolution, self.camera_dFoV)  # calculate the minimum distance from camera to 2D plane
            self.find_min_dist_2d_plane_indicator.configure(text = f"{min_dist_camera_2d_plane:.3f}")  # change the text of the find min dist 2D plane indicator
            self.choose_luminance_threshold_slider.set(self.luminance_threshold)  # set the value of the luminance threshold slider
            if self.camera_thread_flag:  # if the camera thread is running
                self.open_close_camera_button.configure(text = "close\ncamera")  # change the text of the open/close camera button
            else:  # if the camera is closed
                self.open_close_camera_button.configure(text = "open\ncamera")  # change the text of the open/close camera button
            self.camera_transformation_indicator.configure(text = str(np.round(self.camera_wrt_world_transformation_matrix, 3)))  # change the text of the camera transformation indicator
            self.choose_camera_optical_axis_button.configure(text = str([np.round(self.camera_optical_axis[k], 3) for k in range(len(self.camera_optical_axis))]))  # change the text of the choose camera optical axis button
            self.choose_camera_translation_button.configure(text = str([np.round(self.camera_translation[k], 3) for k in range(len(self.camera_translation))]))  # change the text of the choose camera translation button
            self.choose_camera_orientation_button.configure(text = f"{self.camera_orientation:.1f}")  # change the text of the choose camera orientation button
            self.z_rotate_camera_slider.set(np.rad2deg(self.camera_z_rotation))
            self.normal_translate_camera_slider.set(self.camera_translation_displacement)
            self.choose_ArUco_marker_combobox.set(self.chosen_ArUco_marker)  # set the value of the ArUco marker combobox
            self.apply_moving_average_filter_button.configure(text = ["no", "yes"][[False, True].index(self.apply_moving_average_poses_estimation)])  # change the text of the moving average filter button
            self.show_workspace_image_combobox["values"] = self.saved_workspace_images_list  # set the values of the show workspace image combobox
            self.show_workspace_image_combobox.set(self.shown_workspace_image_name)  # set the value of the show workspace image combobox
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_camera_control_indicators: {e}")

    # the functions used for the thread that continuously captures the camera frames
    def start_camera_thread(self, event = None):  # create and start a camera thread
        self.camera_thread_flag = True  # let the camera thread run
        self.camera_thread = threading.Thread(target = self.camera_thread_run_function)  # create the thread for the camera
        self.camera_thread.start()  # start the camera thread
    def kill_camera_thread(self, event = None):  # kill the existed and running camera thread
        self.camera_thread_flag = False  # stop the camera thread
        self.camera_frames_are_shown = False  # the camera frame is not shown from now on
        self.original_image_is_shown = False  # the original image is not shown from now on
        self.grayscale_image_is_shown = False  # the grayscale image is not shown from now on
        self.workspace_image_is_shown = False  # the workspace image is not shown from now on
        self.camera_pose_estimation_happening = False  # the camera pose estimation is not happening from now on
        self.obst_plane_pose_estimation_happening = False  # the obstacles plane pose estimation is not happening from now on
        self.draw_2d_plane_on_image_happening = False  # the 2D plane is not being drawn on the image from now on
        self.obstacles_boundaries_are_shown = False  # the obstacles boundaries are not shown from now on
        self.update_camera_control_indicators()  # update the indicators of the camera control
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def camera_thread_run_function(self):  # the run function, it continuously captures and displays the camera frames
        chosen_ArUco_marker = self.chosen_ArUco_marker  # the chosen ArUco marker
        camera_intrinsic_matrix = self.camera_intrinsic_matrix  # the intrinsic matrix of the camera
        camera_dist_coeffs = self.camera_dist_coeffs  # the distortion coefficients of the camera
        ArUco_wrt_camera_transformation_matrix_prev = np.eye(4)  # the previous transformation matrix of the ArUco marker with respect to the camera frame
        poses_estimations_counter = 0  # the counter for the estimations of the plane and camera poses
        while True:  # while the thread is running
            if self.workspace_image_is_shown:  # if the workspace image is shown
                if self.shown_workspace_image_name in self.saved_workspace_images_list:  # if the shown workspace image exists
                    workspace_image = cv2.imread(self.saved_workspace_images_infos_folder_path + fr"/{self.shown_workspace_image_name}.jpg")  # read the chosen workspace image
                    for k in range(len(self.shown_workspace_image_plane_corners)):  # for each corner of the plane
                        cv2.line(workspace_image, tuple(self.shown_workspace_image_plane_corners[k]), tuple(self.shown_workspace_image_plane_corners[(k + 1) % len(self.shown_workspace_image_plane_corners)]), (255, 0, 255), 2)  # draw the 2D plane on the workspace image
                    cd.show_images_on_screen([workspace_image], [self.workspace_image_window_name], self.images_size_factor, True, False, 0)  # show the chosen workspace image
            if self.obstacles_boundaries_are_shown:  # if the obstacles boundaries are shown
                if self.chosen_detection_workspace_image_name in self.saved_workspace_images_list:  # if the chosen workspace image exists
                    workspace_image = cv2.imread(self.saved_workspace_images_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}.jpg")  # read the chosen workspace image
                    grayscale_image = np.array(Image.fromarray(workspace_image).convert("L").point(lambda pixel: cd.convert_image_pixel_to_grayscale(pixel, [self.luminance_threshold]))).astype(np.uint8)  # convert the chosen workspace image to black and white
                    obstacles_boundaries_detected_image_points, outer_boundary_index, boundaries_image = cd.detect_image_boundaries(grayscale_image, self.workspace_image_plane_image_points, self.removed_boundaries_list, 10 ** (-self.boundaries_precision_parameter / 2.5), self.boundaries_minimum_vertices)  # detect the boundaries of the obstacles in the workspace image
                    for k in range(len(self.workspace_image_plane_image_points)):  # for each corner of the plane
                        cv2.line(boundaries_image, tuple(self.workspace_image_plane_image_points[k]), tuple(self.workspace_image_plane_image_points[(k + 1) % len(self.workspace_image_plane_image_points)]), (255, 0, 255), 2)  # draw the 2D plane on the boundaries image
                    cv2.putText(boundaries_image, "Press the \"d\" key to save the detected boundaries", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)  # write a message on the image frame
                    cd.show_images_on_screen([boundaries_image], [self.obstacles_boundaries_window_name], self.images_size_factor, True, False, 0)  # show the boundaries of the obstacles on the workspace image
                    cv2.setMouseCallback(self.obstacles_boundaries_window_name, self.remove_restore_boundaries, param = obstacles_boundaries_detected_image_points)  # set the mouse callback function for the obstacles boundaries window, to remove/restore boundaries
            if self.camera_thread_flag and self.camera_frames_are_shown:  # if the camera thread is running
                confirm_capture, frame = self.camera_capture.read()  # capture a frame using the camera capture object and store it in the variable frame
                if confirm_capture:  # if the frame was captured successfully
                    frame_height, frame_width = frame.shape[:2]  # the height and width of the captured frame
                    cd.show_images_on_screen([frame], [self.camera_original_window_name], self.images_size_factor, True, False, 0)  # show the captured frame on the screen
                    if not self.apply_moving_average_poses_estimation:  # if the moving average filter for poses estimation is not applied
                        poses_estimations_counter = 0  # reset the counter of the estimations
                    if self.camera_pose_estimation_happening or self.obst_plane_pose_estimation_happening or self.draw_2d_plane_on_image_happening:  # if the camera pose estimation or the obstacles plane pose estimation or the 2D plane drawing on the image is happening
                        ArUco_wrt_camera_transformation_matrix, ArUco_marker_pose_image = cd.estimate_ArUco_marker_pose(frame, chosen_ArUco_marker, camera_intrinsic_matrix, camera_dist_coeffs)  # estimate the pose of the camera using an ArUco marker
                        camera_ArUco_distance = np.linalg.norm(ArUco_wrt_camera_transformation_matrix[:3, 3])  # the distance between the camera and the ArUco marker
                        cv2.putText(ArUco_marker_pose_image, f"Camera - ArUco marker distance: {camera_ArUco_distance:.3f} m", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)  # write the distance between the camera and the ArUco marker on the image
                        cd.show_images_on_screen([ArUco_marker_pose_image], [self.estimate_ArUco_pose_window_name], self.images_size_factor, True, False, 0)  # show the ArUco marker pose image on the screen
                        if np.linalg.norm(ArUco_wrt_camera_transformation_matrix) == 0:  # if the pose of the ArUco marker is not estimated successfully
                            ArUco_wrt_camera_transformation_matrix = ArUco_wrt_camera_transformation_matrix_prev  # keep the previous transformation matrix of the ArUco marker with respect to the camera frame
                        else:  # if the pose of the ArUco marker is estimated successfully
                            ArUco_wrt_camera_transformation_matrix = (ArUco_wrt_camera_transformation_matrix_prev * poses_estimations_counter + ArUco_wrt_camera_transformation_matrix) / (poses_estimations_counter + 1)  # smooth the transformation matrix of the ArUco marker with respect to the camera frame applying a moving average filter
                            ArUco_wrt_camera_transformation_matrix_prev = ArUco_wrt_camera_transformation_matrix  # update the previous transformation matrix of the ArUco marker with respect to the camera frame
                            poses_estimations_counter += 1  # increase the counter of the estimations
                    if self.grayscale_image_is_shown:  # if the grayscale image is shown
                        grayscale_image = np.array(Image.fromarray(frame).convert("L").point(lambda pixel: cd.convert_image_pixel_to_grayscale(pixel, [self.luminance_threshold])))  # convert the captured frame to black and white
                        cd.show_images_on_screen([grayscale_image], [self.grayscale_image_window_name], self.images_size_factor, True, False, 0)  # show the grayscale image on the screen
                    if self.camera_pose_estimation_happening:  # if the camera pose estimation is happening
                        camera_wrt_world_transformation_matrix = self.obst_plane_wrt_world_transformation_matrix @ self.ArUco_wrt_plane_transformation_matrix @ np.linalg.inv(ArUco_wrt_camera_transformation_matrix)  # change the transformation matrix of the camera
                        self.camera_optical_axis, self.camera_orientation, self.camera_translation = gf.get_components_from_xy_plane_transformation(camera_wrt_world_transformation_matrix)  # change the optical axis, orientation and translation of the camera
                        self.reset_camera_sliders()  # reset the sliders variables for the camera
                    if self.obst_plane_pose_estimation_happening:  # if the obstacles plane pose estimation is happening
                        obst_plane_wrt_world_transformation_matrix = self.camera_wrt_world_transformation_matrix @ ArUco_wrt_camera_transformation_matrix @ np.linalg.inv(self.ArUco_wrt_plane_transformation_matrix)  # change the transformation matrix of the obstacles
                        self.obstacles_2d_plane_normal_vector, self.obstacles_2d_plane_orientation, self.obstacles_2d_plane_translation = gf.get_components_from_xy_plane_transformation(obst_plane_wrt_world_transformation_matrix)  # change the normal vector, orientation and translation of the obstacles plane
                        self.reset_2d_plane_sliders()  # reset the sliders variables for the 2D plane
                    if self.draw_2d_plane_on_image_happening:  # if the 2D plane is being drawn on the image
                        x_length = self.obstacles_2d_plane_x_length; y_length = self.obstacles_2d_plane_y_length  # the lengths of the 2D plane
                        initial_obst_plane_points_wrt_ArUco = np.array([[-x_length/2, y_length/2, 0, 1], [x_length/2, y_length/2, 0, 1], [x_length/2, -y_length/2, 0, 1], [-x_length/2, -y_length/2, 0, 1]])
                        transformed_obst_plane_points_wrt_ArUco = np.dot(np.linalg.inv(self.ArUco_wrt_plane_transformation_matrix), initial_obst_plane_points_wrt_ArUco.T).T[:, :3]  # the points of the 2D plane wrt the camera frame
                        drawn_plane_corners_image_points, drawn_shape_image = cd.draw_shape_on_image_plane(frame, transformed_obst_plane_points_wrt_ArUco, ArUco_wrt_camera_transformation_matrix, camera_intrinsic_matrix, camera_dist_coeffs)  # draw the 2D plane on the image
                        cv2.putText(drawn_shape_image, "Press the \"s\" key to save the grayscale image!", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)  # write a message on the image frame
                        cd.show_images_on_screen([drawn_shape_image], [self.draw_shape_image_window_name], self.images_size_factor, True, False, 0)  # show the frame with the 2D plane drawn on it
                else:  # if the frame was not captured successfully
                    self.kill_camera_thread()  # kill the existed and running camera thread
                    ms.showerror("Error", "Couldn't capture frame with the camera!")  # show an error message
                    print("Couldn't capture frame with the camera!")  # print a message to inform the user that the frame was not captured successfully
            pressed_key = cv2.waitKey(1)  # wait for 1 millisecond for a key to be pressed
            if (pressed_key == ord("a") or pressed_key == ord("A")) and self.grayscale_image_is_shown:  # if the user presses the "a" or "A" key
                if self.luminance_threshold > self.luminance_threshold_limits[0]: self.luminance_threshold -= 1  # decrease the luminance threshold
            elif (pressed_key == ord("w") or pressed_key == ord("W")) and self.grayscale_image_is_shown:  # if the user presses the "w" or "W" key
                if self.luminance_threshold < self.luminance_threshold_limits[1]: self.luminance_threshold += 1  # increase the luminance threshold
            elif (pressed_key == ord("e") or pressed_key == ord("E")) and self.obstacles_boundaries_are_shown:  # if the user presses the "e" or "E" key
                if self.boundaries_precision_parameter > self.boundaries_precision_limits[0]: self.boundaries_precision_parameter -= 1  # decrease the boundaries precision parameter
            elif (pressed_key == ord("r") or pressed_key == ord("R")) and self.obstacles_boundaries_are_shown:  # if the user presses the "r" or "R" key
                if self.boundaries_precision_parameter < self.boundaries_precision_limits[1]: self.boundaries_precision_parameter += 1  # increase the boundaries precision parameter
            elif (pressed_key == ord("t") or pressed_key == ord("T")) and self.obstacles_boundaries_are_shown:  # if the user presses the "t" or "T" key
                if self.boundaries_minimum_vertices > self.boundaries_minimum_vertices_limits[0]: self.boundaries_minimum_vertices -= 1  # decrease the minimum number of vertices of the boundaries
            elif (pressed_key == ord("y") or pressed_key == ord("Y")) and self.obstacles_boundaries_are_shown:  # if the user presses the "y" or "Y" key
                if self.boundaries_minimum_vertices < self.boundaries_minimum_vertices_limits[1]: self.boundaries_minimum_vertices += 1  # increase the minimum number of vertices of the boundaries
            elif (pressed_key == ord("f") or pressed_key == ord("F")) and (self.obst_plane_pose_estimation_happening or self.camera_pose_estimation_happening or self.draw_2d_plane_on_image_happening):  # if the user presses the "f" or "F" key
                self.apply_moving_average_poses_estimation = not self.apply_moving_average_poses_estimation  # apply or not the moving average filter for poses estimation
            elif (pressed_key == ord("s") or pressed_key == ord("S")) and self.draw_2d_plane_on_image_happening:  # if the user presses the "s" or "S" key
                drawn_plane_corners_image_points_x_min = int(np.min(drawn_plane_corners_image_points[:, 0]))  # the minimum x coordinate of the plane corners image points
                drawn_plane_corners_image_points_x_max = int(np.max(drawn_plane_corners_image_points[:, 0]))  # the maximum x coordinate of the plane corners image points
                drawn_plane_corners_image_points_y_min = int(np.min(drawn_plane_corners_image_points[:, 1]))  # the minimum y coordinate of the plane corners image points
                drawn_plane_corners_image_points_y_max = int(np.max(drawn_plane_corners_image_points[:, 1]))  # the maximum y coordinate of the plane corners image points
                if drawn_plane_corners_image_points_x_min >= 0 and drawn_plane_corners_image_points_x_max < frame_width and drawn_plane_corners_image_points_y_min >= 0 and drawn_plane_corners_image_points_y_max < frame_height:  # if the 2D plane is inside the frame
                    self.save_workspace_image_information(frame, self.obst_plane_wrt_world_transformation_matrix, self.obstacles_2d_plane_x_length, self.obstacles_2d_plane_y_length, drawn_plane_corners_image_points, self.camera_wrt_world_transformation_matrix)  # save the workspace image information
                else:  # if the 2D plane is not inside the frame
                    ms.showerror("Error while saving", "The whole workspace 2D plane must be inside the frame in order for the image to be saved!")  # show an error message
            elif (pressed_key == ord("d") or pressed_key == ord("D")) and self.obstacles_boundaries_are_shown:  # if the user presses the "d" or "D" key
                if outer_boundary_index != -1:  # if an outer boundary was detected
                    # remove the excluded boundaries from the detected boundaries and move the outer boundary to the end of the list
                    outer_boundary = obstacles_boundaries_detected_image_points[outer_boundary_index]  # get the outer boundary
                    remaining_obstacles_boundaries_detected = [obstacles_boundaries_detected_image_points[k] for k in range(len(obstacles_boundaries_detected_image_points)) if k != outer_boundary_index and k not in self.removed_boundaries_list]  # remove the excluded boundaries and the outer boundary from the detected boundaries
                    remaining_obstacles_boundaries_detected.append(outer_boundary)  # add the outer boundary to the end of the list
                    # convert the obstacles boundaries image points to xy plane world points and save the obstacles objects detected in the workspace image
                    obstacles_boundaries_detected_xy_obstplane_points = cd.convert_boundaries_image_points_to_obstplane_points(remaining_obstacles_boundaries_detected, self.workspace_image_plane_frame_wrt_world, self.workspace_obstacles_height_detection, \
                                                                                                                        self.workspace_image_camera_frame_wrt_world, self.camera_intrinsic_matrix, self.camera_dist_coeffs)  # convert the obstacles boundaries image points to xy obstacles plane points
                    obstacles_boundaries_to_be_saved = []  # the boundaries of the obstacles to be saved (need to remove the duplicate consecutive points)
                    for boundary in obstacles_boundaries_detected_xy_obstplane_points:  # for each boundary of the obstacles
                        obstacles_boundaries_to_be_saved.append([boundary[0]])  # add the first point of the current boundary to the list of the obstacles boundaries to be saved
                        for k in range(1, len(boundary)):  # for each point of the current boundary
                            if np.linalg.norm(boundary[k] - boundary[k - 1]) >= 0.0001:  # if the distance between the current point and the previous point is greater than or equal to 0.1 mm
                                obstacles_boundaries_to_be_saved[-1].append(boundary[k])  # add the current point to the list of the obstacles boundaries to be saved
                    shutil.rmtree(self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}")  # delete the existing folder of the obstacles objects detected in the workspace image
                    os.mkdir(self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}")  # create a new folder for the obstacles objects detected in the workspace image
                    inner_boundaries_counter = 0  # the counter for the inner boundaries
                    for k in range(len(obstacles_boundaries_to_be_saved)):  # for each boundary of the obstacles
                        if k == len(obstacles_boundaries_to_be_saved) - 1:  # if the boundary is the outer boundary
                            file_name = "outer"  # the name of the file to save the outer boundary
                        else:  # if the boundary is an inner boundary
                            inner_boundaries_counter += 1  # increase the counter for the inner boundaries
                            file_name = f"inner_{inner_boundaries_counter}"  # the name of the file to save the inner boundary
                        with open(self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}/{file_name}.txt", "w", encoding = "utf-8") as file:  # open the file in write mode 
                            file.write(f"Boundary height (m): {self.workspace_obstacles_height_saved:.4f}\n")  # write the height of the boundary to the file
                            file.write("\nBoundary (x, y) points transformed on the world xy plane (m):\n")  # write the points of the boundary to the file
                            np.savetxt(file, np.array(obstacles_boundaries_to_be_saved[k]).reshape(-1, 2), fmt = "%.4f", delimiter = " ")  # write the points of the boundary to the file
                            file.close()  # close the file
                    self.chosen_workspace_saved_obstacles_objects_list = [file[:-4] for file in os.listdir(self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}") if file.endswith(".txt")]  # the list to store the saved obstacles objects for the chosen workspace image
                    ms.showinfo("Boundaries saved", "The detected obstacles have been saved successfully!")  # show an info message
                else:  # if no outer boundary was detected
                    ms.showerror("Error while saving", "The detected obstacles where not saved because there is no outer boundary detected (if it exists, it is drawn in red)! Try removing some boundaries!")  # show an error message
            elif (pressed_key == ord("q") or pressed_key == ord("Q")) or not self.camera_thread_flag:  # if the user presses the "q" or "Q" key or the camera thread is stopped
                self.kill_camera_thread()  # kill the existed and running camera thread
                cv2.destroyAllWindows()  # close all OpenCV windows that were opened
                if self.camera_frames_are_shown: self.camera_capture.release()  # release/free the camera capture object
                print("The camera has been disconnected successfully!")  # print a message to inform the user that the camera has been disconnected successfully
                break  # break the while loop
            self.update_camera_control_indicators()  # update the indicators of the camera control
            self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles

    # for the main menu where the workspace obstacles are created
    def choose_2d_plane_x_length(self, event = None):  # choose the x length of the 2D plane
        x_length = sd.askfloat("Obstacles 2D plane x length", "Enter the x length of the 2D plane in meters:", initialvalue = self.obstacles_2d_plane_x_length, minvalue = 0, parent = self.menus_area)  # ask the user to enter the x length of the 2D plane
        if x_length != None:  # if the user enters a number
            self.obstacles_2d_plane_x_length = x_length  # change the x length of the 2D plane
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def choose_2d_plane_y_length(self, event = None):  # choose the y length of the 2D plane
        y_length = sd.askfloat("Obstacles 2D plane y length", "Enter the y length of the 2D plane in meters:", initialvalue = self.obstacles_2d_plane_y_length, minvalue = 0, parent = self.menus_area)  # ask the user to enter the y length of the 2D plane
        if y_length != None:  # if the user enters a number
            self.obstacles_2d_plane_y_length = y_length  # change the y length of the 2D plane
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def choose_2d_plane_normal_vector(self, event = None):  # choose the normal vector of the 2D plane
        normal_vector_x = sd.askfloat("Obstacles 2D plane normal vector", "Enter the x component of the normal vector of the 2D plane \n(it will be auto normalized):", initialvalue = self.obstacles_2d_plane_normal_vector[0], parent = self.menus_area)  # ask the user to enter the x component of the normal vector of the 2D plane
        if normal_vector_x != None:  # if the user enters a number
            self.obstacles_2d_plane_normal_vector[0] = normal_vector_x  # change the x component of the normal vector of the 2D plane
        normal_vector_y = sd.askfloat("Obstacles 2D plane normal vector", "Enter the y component of the normal vector of the 2D plane \n(it will be auto normalized):", initialvalue = self.obstacles_2d_plane_normal_vector[1], parent = self.menus_area)  # ask the user to enter the y component of the normal vector of the 2D plane
        if normal_vector_y != None:  # if the user enters a number
            self.obstacles_2d_plane_normal_vector[1] = normal_vector_y  # change the y component of the normal vector of the 2D plane
        normal_vector_z = sd.askfloat("Obstacles 2D plane normal vector", "Enter the z component of the normal vector of the 2D plane \n(it will be auto normalized):", initialvalue = self.obstacles_2d_plane_normal_vector[2], parent = self.menus_area)  # ask the user to enter the z component of the normal vector of the 2D plane
        if normal_vector_z != None:  # if the user enters a number
            self.obstacles_2d_plane_normal_vector[2] = normal_vector_z  # change the z component of the normal vector of the 2D plane
        if np.linalg.norm(self.obstacles_2d_plane_normal_vector) != 0:  # if the normal vector of the 2D plane is not the zero vector
            self.obstacles_2d_plane_normal_vector = self.obstacles_2d_plane_normal_vector / np.linalg.norm(self.obstacles_2d_plane_normal_vector)  # normalize the normal vector of the 2D plane
        else:  # if the normal vector of the 2D plane is the zero vector
            self.obstacles_2d_plane_normal_vector = np.array([0, 0, 1])  # set the normal vector of the 2D plane to the default value
        self.reset_2d_plane_sliders()  # initialize the sliders variables for the 2D plane
    def choose_2d_plane_translation(self, event = None):  # choose the translation of the 2D plane
        translation_x = sd.askfloat("Obstacles 2D plane translation", "Enter the x component of the translation of the 2D plane (in meters):", initialvalue = self.obstacles_2d_plane_translation[0], parent = self.menus_area)  # ask the user to enter the x component of the translation of the 2D plane
        if translation_x != None:  # if the user enters a number
            self.obstacles_2d_plane_translation[0] = translation_x  # change the x component of the translation of the 2D plane
        translation_y = sd.askfloat("Obstacles 2D plane translation", "Enter the y component of the translation of the 2D plane (in meters):", initialvalue = self.obstacles_2d_plane_translation[1], parent = self.menus_area)  # ask the user to enter the y component of the translation of the 2D plane
        if translation_y != None:  # if the user enters a number
            self.obstacles_2d_plane_translation[1] = translation_y  # change the y component of the translation of the 2D plane
        translation_z = sd.askfloat("Obstacles 2D plane translation", "Enter the z component of the translation of the 2D plane (in meters):", initialvalue = self.obstacles_2d_plane_translation[2], parent = self.menus_area)  # ask the user to enter the z component of the translation of the 2D plane
        if translation_z != None:  # if the user enters a number
            self.obstacles_2d_plane_translation[2] = translation_z  # change the z component of the translation of the 2D plane
        self.reset_2d_plane_sliders()  # initialize the sliders variables for the 2D plane
    def choose_2d_plane_orientation(self, event = None):  # choose the orientation of the 2D plane
        orientation_angle = sd.askfloat("Obstacles 2D plane orientation", "Enter the orientation angle of the 2D plane,\naround its normal vector (in degrees):", initialvalue = self.obstacles_2d_plane_orientation, minvalue = -180, maxvalue = 180, parent = self.menus_area)  # ask the user to enter the angle of the orientation of the 2D plane
        if orientation_angle != None:  # if the user enters a number
            self.obstacles_2d_plane_orientation = orientation_angle  # change the angle of the orientation of the 2D plane
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def change_2d_plane_move_sliders(self, event = None):  # change the rotation of the 2D plane around the z axis
        self.obstacles_2d_plane_z_rotation = np.deg2rad(self.z_rotate_2d_plane_slider.get())  # the rotation angle of the 2D plane around the z axis
        self.obstacles_2d_plane_translation_displacement = self.normal_translate_2d_plane_slider.get()  # the translation displacement of the 2D plane along its normal vector
        Z_rot_mat = np.array([[np.cos(self.obstacles_2d_plane_z_rotation), -np.sin(self.obstacles_2d_plane_z_rotation), 0], [np.sin(self.obstacles_2d_plane_z_rotation), np.cos(self.obstacles_2d_plane_z_rotation), 0], [0, 0, 1]])  # the rotation matrix around the z axis
        self.obstacles_2d_plane_normal_vector = Z_rot_mat @ self.obstacles_2d_plane_normal_vector_prev  # change the normal vector of the 2D plane
        self.obstacles_2d_plane_translation = Z_rot_mat @ (self.obstacles_2d_plane_translation_prev + self.obstacles_2d_plane_translation_displacement * self.obstacles_2d_plane_normal_vector_prev)  # change the translation of the 2D plane
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def reset_2d_plane_sliders(self, event = None):  # reset the sliders variables for the 2D plane
        self.obstacles_2d_plane_normal_vector_prev = np.copy(self.obstacles_2d_plane_normal_vector)  # reset the normal vector of the 2D plane
        self.obstacles_2d_plane_translation_prev = np.copy(self.obstacles_2d_plane_translation)  # reset the translation of the 2D plane
        self.obstacles_2d_plane_z_rotation = 0; self.obstacles_2d_plane_translation_displacement = 0  # initialize the sliders variables for the 2D plane
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def define_ArUco_marker_position(self, event = None):  # define the position of the ArUco marker
        self.ArUco_marker_position = self.alternate_matrix_elements(self.ArUco_marker_positions_list, self.ArUco_marker_position)  # change the position of the ArUco marker
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def define_ArUco_marker_orientation(self, event = None):  # define the orientation of the ArUco marker
        self.ArUco_marker_orientation = self.alternate_matrix_elements(self.ArUco_marker_orientations_list, self.ArUco_marker_orientation)  # change the orientation of the ArUco marker
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def compute_ArUco_wrt_plane_transformation(self, event = None):  # compute the transformation matrix of the ArUco marker with respect to the 2D plane
        angle = np.deg2rad(self.ArUco_marker_orientation)  # the angle of the ArUco marker orientation around the plane normal vector
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # the rotation matrix of the ArUco marker
        if self.ArUco_marker_position == "center":  # if the ArUco marker position is the center
            return np.eye(4) @ rotation  # the transformation matrix from the plane to the ArUco marker
        elif self.ArUco_marker_position == "top-left corner":  # if the ArUco marker position is the top-left corner
            return np.array([[1, 0, 0, -self.obstacles_2d_plane_x_length / 2.0], [0, 1, 0, self.obstacles_2d_plane_y_length / 2.0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ rotation  # the transformation matrix from the plane to the ArUco marker
        elif self.ArUco_marker_position == "up-middle edge":  # if the ArUco marker position is the up-middle edge
            return np.array([[1, 0, 0, 0], [0, 1, 0, self.obstacles_2d_plane_y_length / 2.0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ rotation  # the transformation matrix from the plane to the ArUco marker
        elif self.ArUco_marker_position == "top-right corner":  # if the ArUco marker position is the top-right corner
            return np.array([[1, 0, 0, self.obstacles_2d_plane_x_length / 2.0], [0, 1, 0, self.obstacles_2d_plane_y_length / 2.0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ rotation  # the transformation matrix from the plane to the ArUco marker
        elif self.ArUco_marker_position == "right-middle edge":  # if the ArUco marker position is the right-middle edge
            return np.array([[1, 0, 0, self.obstacles_2d_plane_x_length / 2.0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ rotation  # the transformation matrix from the plane to the ArUco marker
        elif self.ArUco_marker_position == "bottom-right corner":  # if the ArUco marker position is the bottom-right corner
            return np.array([[1, 0, 0, self.obstacles_2d_plane_x_length / 2.0], [0, 1, 0, -self.obstacles_2d_plane_y_length / 2.0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ rotation  # the transformation matrix from the plane to the ArUco marker
        elif self.ArUco_marker_position == "down-middle edge":  # if the ArUco marker position is the down-middle edge
            return np.array([[1, 0, 0, 0], [0, 1, 0, -self.obstacles_2d_plane_y_length / 2.0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ rotation  # the transformation matrix from the plane to the ArUco marker
        elif self.ArUco_marker_position == "bottom-left corner":  # if the ArUco marker position is the bottom-left corner
            return np.array([[1, 0, 0, -self.obstacles_2d_plane_x_length / 2.0], [0, 1, 0, -self.obstacles_2d_plane_y_length / 2.0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ rotation  # the transformation matrix from the plane to the ArUco marker
        elif self.ArUco_marker_position == "left-middle edge":  # if the ArUco marker position is the left-middle edge
            return np.array([[1, 0, 0, -self.obstacles_2d_plane_x_length / 2.0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) @ rotation  # the transformation matrix from the plane to the ArUco marker
        else:  # if the ArUco marker position is not defined
            return np.eye(4) @ rotation  # the transformation matrix from the plane to the ArUco marker
    def choose_plane_singularities_tolerance(self, event = None):  # choose the tolerance for the singularities of the robotic manipulator on the 2D plane
        singularities_tolerance = sd.askfloat("Tolerance for singularities", "Enter the tolerance for searching the robotic\nmanipulator singularities on the 2D plane:", initialvalue = self.plane_singularities_tolerance, minvalue = 0.0, maxvalue = 1e-1, parent = self.menus_area)
        if singularities_tolerance != None:  # if the user enters a number
            self.plane_singularities_tolerance = singularities_tolerance  # change the tolerance for the singularities of the robotic manipulator on the 2D plane
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def choose_plane_singularities_samples(self, event = None):  # choose the divisions of the 2D plane
        singularities_divs = sd.askinteger("Sampling of 2D plane", "Enter the number of samples for\neach side of the 2D plane:", initialvalue = self.plane_singularities_samples, minvalue = 10, maxvalue = 100, parent = self.menus_area)
        if singularities_divs != None:  # if the user enters a number
            self.plane_singularities_samples = singularities_divs  # change the points samples of the 2D plane
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def find_plane_singularities(self, event = None):  # find the singularities of the robotic manipulator on the 2D plane
        if self.robotic_manipulator_is_built:  # if the robotic manipulator model is built
            obst_plane_wrt_world_transformation_matrix_rev = gf.xy_plane_transformation(-self.obstacles_2d_plane_normal_vector, self.obstacles_2d_plane_orientation, self.obstacles_2d_plane_translation)
            kin.find_kinematic_singularities_on_plane(self.built_robotic_manipulator, self.obstacles_2d_plane_x_length, self.obstacles_2d_plane_y_length, obst_plane_wrt_world_transformation_matrix_rev, \
                                                                                self.plane_singularities_samples, self.plane_singularities_samples, self.plane_singularities_tolerance, self.invkine_tolerance, True)  # find the singularities of the robotic manipulator on the 2D plane
        else:  # if no robotic manipulator model is built
            ms.showerror("Error", "Please build the robotic manipulator model first!", parent = self.menus_area)  # show an error message
    def save_obstacles_transformation(self, event = None):  # save the current transformation of the obstacles
        ask_file_name = sd.askstring("Save obstacles transformation", "Enter the file name to save the obstacles transformation:", initialvalue = f"plane_{len(self.saved_obstacles_transformations_list) + 1}", parent = self.menus_area)  # ask the user to enter the file name to save the obstacles transformation
        if ask_file_name != None and ask_file_name != "":  # if the user enters a file name
            overwrite_file_accept = False  # the user's choice to overwrite the file
            if (ask_file_name + ".txt") in os.listdir(self.saved_obstacles_transformations_folder_path):  # if the file already exists
                overwrite_file_accept = ms.askyesno("Overwrite file", f"The file \"{ask_file_name}.txt\" already exists. Do you want to overwrite it?", parent = self.menus_area)  # ask the user if they want to overwrite the file
            if (ask_file_name + ".txt") not in os.listdir(self.saved_obstacles_transformations_folder_path) or overwrite_file_accept:  # if the file does not exist or the user wants to overwrite it
                with open(self.saved_obstacles_transformations_folder_path + fr"/{ask_file_name}.txt", "w", encoding = "utf-8") as file:  # open the file in write mode
                    file.write("Plane x length (m): " + f"{self.obstacles_2d_plane_x_length:.3f}" + "\n")  # write the x length of the 2D plane to the file
                    file.write("Plane y length (m): " + f"{self.obstacles_2d_plane_y_length:.3f}" + "\n")  # write the y length of the 2D plane to the file
                    file.write("Plane normal vector: " + str([np.round(self.obstacles_2d_plane_normal_vector[k], 3) for k in range(len(self.obstacles_2d_plane_normal_vector))]) + "\n")  # write the normal vector of the 2D plane to the file
                    file.write("Plane translation (m): " + str([np.round(self.obstacles_2d_plane_translation[k], 3) for k in range(len(self.obstacles_2d_plane_translation))]) + "\n")  # write the translation of the 2D plane to the file
                    file.write("Plane orientation (degrees): " + f"{self.obstacles_2d_plane_orientation:.1f}" + "\n")  # write the orientation of the 2D plane to the file
                file.close()  # close the file
                if ask_file_name not in self.saved_obstacles_transformations_list:  # if the name of the saved file is not in the values of the combobox that contains the names of the saved files
                    self.saved_obstacles_transformations_list.append(ask_file_name)  # add the file name to the list of the saved files
                ms.showinfo("Plane/Obstacles transformation saved!", f"The obstacles transformation has been saved successfully in \"{ask_file_name}.txt\"!", parent = self.menus_area)  # show an info message that the obstacles transformation has been saved successfully                
            else:  # if the file exists and the user does not want to overwrite it
                ms.showinfo("File not saved", "The file has not been saved!", parent = self.menus_area)  # show an information message
        else:  # if the user does not enter a name for the file
            ms.showerror("Error", "You have not entered a name for the file!", parent = self.menus_area)  # show an error message
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def load_obstacles_transformation(self, event = None):  # load the specified transformation of the obstacles
        load_file_accept = False  # the user's choice to load the file
        chosen_plane_obstacles_transformation_name = self.load_obstacles_transformation_combobox.get()  # the name of the chosen loaded file that contains the obstacles transformation
        if chosen_plane_obstacles_transformation_name in self.saved_obstacles_transformations_list:  # if the chosen file exists
            if event != None:  # if the function is called by an event
                load_file_accept = ms.askyesno("Confirm loading", f"Are you sure you want to load the file \"{chosen_plane_obstacles_transformation_name}.txt\"?")  # ask the user if they want to load the chosen file
            else:
                load_file_accept = True  # the file is loaded automatically
            if load_file_accept:  # if the user wants to load the chosen file
                self.chosen_plane_obstacles_transformation_name = chosen_plane_obstacles_transformation_name  # change the plane and obstacles transformation name
                transformation_file = open(self.saved_obstacles_transformations_folder_path + fr"/{self.chosen_plane_obstacles_transformation_name}.txt", "r", encoding = "utf-8")  # open the chosen file in read mode
                transformation_file_lines = transformation_file.readlines()  # read all the lines of the file
                transformation_file.close()  # close the file
                self.obstacles_2d_plane_x_length = float(transformation_file_lines[0].split(": ")[1])  # the x length of the 2D plane
                self.obstacles_2d_plane_y_length = float(transformation_file_lines[1].split(": ")[1])  # the y length of the 2D plane
                self.obstacles_2d_plane_normal_vector = np.array([float(k) for k in transformation_file_lines[2].split(": ")[1][1:-2].split(", ")])  # the normal vector of the 2D plane
                self.obstacles_2d_plane_translation = np.array([float(k) for k in transformation_file_lines[3].split(": ")[1][1:-2].split(", ")])  # the translation of the 2D plane
                self.obstacles_2d_plane_orientation = float(transformation_file_lines[4].split(": ")[1])  # the orientation of the 2D plane
                self.reset_2d_plane_sliders()  # initialize the sliders variables for the 2D plane
        else:  # if the chosen file does not exist
            ms.showerror("Error", f"The chosen plane / obstacles transformation \"{chosen_plane_obstacles_transformation_name}.txt\" does not exist!", parent = self.menus_area)  # show an error message
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def load_workspace_image_for_detection(self, event = None):  # change the chosen workspace image for the obstacles detection
        chosen_detection_workspace_image_name = self.load_workspace_image_combobox.get()  # the name of the chosen workspace image
        if chosen_detection_workspace_image_name in self.saved_workspace_images_list:  # if the chosen workspace image exists
            self.chosen_detection_workspace_image_name = chosen_detection_workspace_image_name  # change the chosen workspace image for the obstacles detection
            self.chosen_workspace_saved_obstacles_objects_list = [file[:-4] for file in os.listdir(self.saved_obstacles_objects_infos_folder_path + fr"/{chosen_detection_workspace_image_name}") if file.endswith(".txt")]  # the list to store the saved obstacles objects for the chosen workspace image
            if len(self.chosen_workspace_saved_obstacles_objects_list) != 0:  # if there are saved obstacles objects for the chosen workspace image
                if self.chosen_obstacle_object_name not in self.chosen_workspace_saved_obstacles_objects_list:  # if the chosen obstacle object name is not in the saved obstacles objects for the chosen workspace image
                    self.chosen_obstacle_object_name = self.chosen_workspace_saved_obstacles_objects_list[0]  # the chosen obstacle name from the saved obstacles objects, if there are saved obstacles objects for the chosen workspace image
                chosen_obstacle_file_lines = open(self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}/{self.chosen_obstacle_object_name}.txt", "r", encoding = "utf-8").readlines()  # read the lines of the chosen obstacle object file
                self.workspace_obstacles_height_saved = float(chosen_obstacle_file_lines[0].split(": ")[1])  # the saved height of the obstacles for the chosen workspace image
            else:  # if there are no saved obstacles objects for the chosen workspace image
                self.chosen_obstacle_object_name = ""  # reset the chosen obstacle object name, if there are no saved obstacles objects for the chosen workspace image
            self.workspace_image_plane_frame_wrt_world, self.workspace_image_plane_x_length, self.workspace_image_plane_y_length, self.workspace_image_plane_image_points, self.workspace_image_camera_frame_wrt_world = self.load_workspace_image_information(self.chosen_detection_workspace_image_name)  # load the information of the chosen workspace image
            self.removed_boundaries_list = []  # initialize the list to store the removed boundaries of the workspace obstacles
            self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
        else:  # if the chosen file does not exist
            ms.showerror("Error", f"The chosen workspace image \"{chosen_detection_workspace_image_name}\" does not exist!", parent = self.menus_area)  # show an error message
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def change_boundaries_precision(self, event = None):  # change the precision parameter of the boundaries
        self.boundaries_precision_parameter = self.boundaries_precision_slider.get()  # the precision parameter of the boundaries
        self.removed_boundaries_list = []  # restore all the removed boundaries
    def change_boundaries_vertices_lower_limit(self, event = None):  # change the lower limit of the boundaries vertices
        self.boundaries_minimum_vertices = self.boundaries_vertices_lower_limit_slider.get()  # the minimum number of the boundaries vertices
        self.removed_boundaries_list = []  # restore all the removed boundaries
    def choose_obstacles_height_saved(self, event = None):  # choose the obstacles height to be finally saved
        obst_height = sd.askfloat("Obstacle's height to be saved", "Enter the obstacles' height (in millimeters)\nto be finally saved for the current workspace:", initialvalue = 1000.0 * self.workspace_obstacles_height_saved, minvalue = 1.0, maxvalue = 1000.0 * self.workspace_obstacles_max_height, parent = self.menus_area)
        if obst_height != None:  # if the user enters a number
            self.workspace_obstacles_height_saved = np.round(obst_height, 1) / 1000.0  # change the obstacles height to be finally saved
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def choose_obstacles_height_detection(self, event = None):  # choose the obstacles height used for the obstacles boundaries detection
        obst_height = sd.askfloat("Obstacle's height for detection", "Enter the obstacles' height (in millimeters)\nfor the boundaries detection:", initialvalue = 1000.0 * self.workspace_obstacles_height_detection, minvalue = 0.0, maxvalue = 1000.0 * self.workspace_obstacles_max_height, parent = self.menus_area)
        if obst_height != None:  # if the user enters a number
            self.workspace_obstacles_height_detection = np.round(obst_height, 1) / 1000.0  # change the obstacles height for the boundaries detection
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def detect_compute_boundaries_workspace(self, event = None):  # detect and compute the boundaries of the workspace
        if self.chosen_detection_workspace_image_name in self.saved_workspace_images_list:  # if the chosen workspace image exists
            if not self.obstacles_boundaries_are_shown:  # if the boundaries of the workspace obstacles are not shown
                self.obstacles_boundaries_are_shown = True  # the boundaries of the workspace obstacles are shown from now on
                ms.showinfo("Boundaries detection instructions", f"Detect the boundaries of the chosen workspace image! You can press \"e\"/\"r\" to decrease/increase the boundaries precision and \"t\"/\"y\" to decrease/increase the boundaries vertices lower limit. \
You can also press left click on the number of a boundary to remove it and right click to restore the removed boundaries. The current obstacles' heights for saving and for the boundaries detection are {1000.0 * self.workspace_obstacles_height_saved} mm and {1000.0 * self.workspace_obstacles_height_detection} mm respectively \
(press the proper buttons if you want to change them). When you finish press \"d\" to store the remaining detected boundaries!", parent = self.menus_area)
                self.open_close_camera()  # open the camera
        else:  # if the chosen workspace image does not exist
            ms.showerror("Error", f"The chosen workspace image \"{self.chosen_detection_workspace_image_name}\" does not exist!", parent = self.menus_area)  # show an error message
    def remove_restore_boundaries(self, event, x, y, flags, param):  # remove or restore the boundaries of the workspace obstacles
        # param is the list of the detected boundaries of the workspace obstacles
        boundaries_initial_points = [boundary[0] for boundary in param]  # the initial points of the detected boundaries
        if len(boundaries_initial_points) > 0:  # if there are detected boundaries remaining
            mouse_boundaries_distances = np.linalg.norm(np.array(boundaries_initial_points) - np.array([x, y]), axis = 1)  # the distances between the mouse position and the initial points of the detected boundaries
            if event == cv2.EVENT_LBUTTONDOWN:  # if the left mouse button is pressed
                minimum_distance_index = np.argmin(mouse_boundaries_distances)  # the nearest boundary to the mouse position
                if minimum_distance_index not in self.removed_boundaries_list:  # if the boundary is not yet removed
                    self.removed_boundaries_list.append(minimum_distance_index)  # add the index of the removed boundary to the list
            elif event == cv2.EVENT_RBUTTONDOWN:  # if the right mouse button is pressed
                self.removed_boundaries_list = self.removed_boundaries_list[:-1]  # restore the removed boundaries
    def load_chosen_obstacle_boundary_data(self, event = None):  # load the obstacles meshes data from the chosen file
        chosen_obstacle_object_name = self.load_obstacles_data_combobox.get()  # the name of the chosen obstacle object
        if chosen_obstacle_object_name in self.chosen_workspace_saved_obstacles_objects_list:  # if the chosen obstacle object exists
            self.chosen_obstacle_object_name = chosen_obstacle_object_name  # change the chosen obstacle object name
        else:  # if the chosen obstacle object does not exist
            ms.showerror("Error", f"The chosen obstacle object \"{chosen_obstacle_object_name}\" does not exist!", parent = self.menus_area)  # show an error message
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def choose_xy_axis_res_mesh(self, event = None):  # choose the resolution of the xy axis for the obstacle mesh
        self.xy_res_obstacle_mesh = self.alternate_matrix_elements(self.xy_res_obstacle_mesh_list, self.xy_res_obstacle_mesh)  # change the resolution of the xy axis for the obstacle mesh
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def choose_z_axis_resolution_mesh(self, event = None):  # choose the resolution of the z axis for the obstacle mesh
        self.z_res_obstacle_mesh = self.alternate_matrix_elements(self.z_res_obstacle_mesh_list, self.z_res_obstacle_mesh)  # change the resolution of the z axis for the obstacle mesh
        self.update_workspace_obstacles_indicators()  # update the indicators of the workspace obstacles
    def create_obstacle_mesh_stl(self, event = None):  # create the obstacle mesh in STL format
        if len(self.chosen_workspace_saved_obstacles_objects_list) != 0:  # if there are saved obstacles objects
            obstacle_object_path = self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}/{self.chosen_obstacle_object_name}"  # the path of the chosen obstacle object
            chosen_obstacle_file_lines = open(obstacle_object_path + ".txt", "r", encoding = "utf-8").readlines()  # read the lines of the chosen obstacle object file
            obstacle_height = 1000.0 * float(chosen_obstacle_file_lines[0].split(": ")[1])  # the height of the obstacle mesh in millimeters
            obstacle_boundary_points = np.array([[float(k) * 1000.0 for k in line.split(" ")] for line in chosen_obstacle_file_lines[3:]]).reshape(-1, 2)  # the boundary points of the obstacle mesh in millimeters
            points_limits = [[min(obstacle_boundary_points[:, 0]), max(obstacle_boundary_points[:, 0])], [min(obstacle_boundary_points[:, 1]), max(obstacle_boundary_points[:, 1])]]  # the limits of the boundary points in the x and y directions in millimeters
            pbo.convert_boundary_to_3D_mesh_stl(obstacle_boundary_points, obstacle_height, points_limits, [self.xy_res_obstacle_mesh, self.xy_res_obstacle_mesh], self.z_res_obstacle_mesh, obstacle_object_path + ".stl", True)  # convert the boundary points to a 3D shape in STL format
        else:  # if there are no saved obstacles objects
            ms.showerror("Error", "There are no saved obstacles objects! Press \"detect boundaries\" button to create them!", parent = self.menus_area)  # show an error message
    def create_all_obstacles_stl(self, event = None):  # create all the obstacles meshes in STL format
        if len(self.chosen_workspace_saved_obstacles_objects_list) != 0:  # if there are saved obstacles objects
            for k in range(len(self.chosen_workspace_saved_obstacles_objects_list)):  # iterate through all the saved obstacles objects
                obstacle_object_path = self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_detection_workspace_image_name}/{self.chosen_workspace_saved_obstacles_objects_list[k]}"  # the path of the chosen obstacle object
                chosen_obstacle_file_lines = open(obstacle_object_path + ".txt", "r", encoding = "utf-8").readlines()  # read the lines of the chosen obstacle object file
                obstacle_height = 1000.0 * float(chosen_obstacle_file_lines[0].split(": ")[1])  # the height of the obstacle mesh in millimeters
                obstacle_boundary_points = np.array([[float(k) * 1000.0 for k in line.split(" ")] for line in chosen_obstacle_file_lines[3:]]).reshape(-1, 2)  # the boundary points of the obstacle mesh in millimeters
                points_limits = [[min(obstacle_boundary_points[:, 0]), max(obstacle_boundary_points[:, 0])], [min(obstacle_boundary_points[:, 1]), max(obstacle_boundary_points[:, 1])]]  # the limits of the boundary points in the x and y directions in millimeters
                pbo.convert_boundary_to_3D_mesh_stl(obstacle_boundary_points, obstacle_height, points_limits, [self.xy_res_obstacle_mesh, self.xy_res_obstacle_mesh], self.z_res_obstacle_mesh, obstacle_object_path + ".stl", False)  # convert the boundary points to a 3D shape in STL format
        else:  # if there are no saved obstacles objects
            ms.showerror("Error", "There are no saved obstacles objects! Press \"detect boundaries\" button to create them!", parent = self.menus_area)  # show an error message
    def update_workspace_obstacles_indicators(self, event = None):  # update the indicators of the workspace obstacles
        self.obst_plane_wrt_world_transformation_matrix = gf.xy_plane_transformation(self.obstacles_2d_plane_normal_vector, self.obstacles_2d_plane_orientation, self.obstacles_2d_plane_translation)  # the total transformation matrix of the obstacles
        self.ArUco_wrt_plane_transformation_matrix = self.compute_ArUco_wrt_plane_transformation()  # compute the transformation matrix from the plane to the ArUco marker
        try:
            self.obstacles_transformation_indicator.configure(text = str(np.round(self.obst_plane_wrt_world_transformation_matrix, 3)))  # change the text of the obstacles transformation indicator
            self.choose_2d_plane_x_length_button.configure(text = f"{self.obstacles_2d_plane_x_length:.3f}")  # change the text of the choose 2D plane x length button
            self.choose_2d_plane_y_length_button.configure(text = f"{self.obstacles_2d_plane_y_length:.3f}")  # change the text of the choose 2D plane y length button
            self.choose_2d_plane_normal_vector_button.configure(text = str([np.round(self.obstacles_2d_plane_normal_vector[k], 3) for k in range(len(self.obstacles_2d_plane_normal_vector))]))  # change the text of the choose 2D plane normal vector button
            self.choose_2d_plane_translation_button.configure(text = str([np.round(self.obstacles_2d_plane_translation[k], 3) for k in range(len(self.obstacles_2d_plane_translation))]))  # change the text of the choose 2D plane translation button
            self.choose_2d_plane_orientation_button.configure(text = f"{self.obstacles_2d_plane_orientation:.1f}")  # change the text of the choose 2D plane orientation button
            self.z_rotate_2d_plane_slider.set(np.rad2deg(self.obstacles_2d_plane_z_rotation))
            self.normal_translate_2d_plane_slider.set(self.obstacles_2d_plane_translation_displacement)
            self.plane_singularities_tolerance_button.configure(text = f"{np.round(self.plane_singularities_tolerance, 7)}")  # change the text of the choose plane singularities tolerance button
            self.plane_singularities_samples_button.configure(text = f"{self.plane_singularities_samples}")  # change the text of the choose plane singularities divs button
            self.define_ArUco_marker_position_button.configure(text = self.ArUco_marker_position)  # change the text of the define ArUco marker position button
            self.define_ArUco_marker_orientation_button.configure(text = self.ArUco_marker_orientation)  # change the text of the define ArUco marker orientation button
            camera_plane_z_axis_alignment = int(np.round(100 * abs(1 - np.arccos(np.round(np.dot(self.camera_optical_axis, self.obstacles_2d_plane_normal_vector), 3)) / (np.pi/2))))  # the alignment of the camera optical axis with the obstacles plane normal vector
            self.camera_plane_z_axis_alignment_indicator.configure(text = f"{camera_plane_z_axis_alignment} %")  # change the text of the camera plane z axis alignment indicator
            self.load_obstacles_transformation_combobox["values"] = self.saved_obstacles_transformations_list  # set the values of the obstacles transformation combobox to the saved obstacles transformations list
            self.load_obstacles_transformation_combobox.set(self.chosen_plane_obstacles_transformation_name)  # set the combobox to the chosen obstacles transformation
            self.load_workspace_image_combobox["values"] = self.saved_workspace_images_list  # set the values of the workspace image combobox to the saved workspace images list
            self.load_workspace_image_combobox.set(self.chosen_detection_workspace_image_name)  # set the combobox to the chosen workspace image
            self.load_obstacles_data_combobox["values"] = self.chosen_workspace_saved_obstacles_objects_list  # set the values of the obstacles data combobox to the chosen workspace saved obstacles objects list
            self.load_obstacles_data_combobox.set(self.chosen_obstacle_object_name)  # set the combobox to the first element of the chosen workspace saved obstacles objects list
            self.obstacles_height_saved_button.configure(text = f"{1000.0 * self.workspace_obstacles_height_saved:.1f}")  # change the text of the choose obstacles height saved button
            self.boundaries_precision_slider.set(self.boundaries_precision_parameter)  # set the boundaries precision slider to the chosen value
            self.boundaries_vertices_lower_limit_slider.set(self.boundaries_minimum_vertices)  # set the boundaries vertices lower limit slider to the chosen value
            self.obstacles_height_detection_button.configure(text = f"{1000.0 * self.workspace_obstacles_height_detection:.1f}")  # change the text of the choose obstacles height detection button
            self.xy_axis_res_mesh_button.configure(text = self.xy_res_obstacle_mesh)  # change the text of the choose xy axis res mesh button
            self.z_axis_res_mesh_button.configure(text = self.z_res_obstacle_mesh)  # change the text of the choose z axis res mesh button
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_workspace_obstacles_indicators: {e}")
    
    # for the main menu where the obstalces avoidance problem is solved
    def load_workspace_obstacles_infos_for_solver(self, event = None):  # load the information (the boundaries more specifically) of the chosen workspace obstacles
        if event != None:  # if the function is called by an event
            self.clear_control_law_outputs_lists()  # clear the control law outputs list
        if event != None or (event == None and len(self.obstacles_boundaries_for_solver) == 0):  # if the function is called by the user or there are no obstacles boundaries detected yet
            chosen_solver_workspace_image_name = self.load_workspace_obstacles_objects_combobox.get()  # the name of the chosen workspace image
            if chosen_solver_workspace_image_name in self.saved_workspace_images_list:  # if the chosen workspace image exists
                self.chosen_solver_workspace_image_name = chosen_solver_workspace_image_name  # set the chosen workspace image name
                self.boundaries_are_detected = False  # the boundaries of the obstacles are not detected
                self.positions_on_plane_are_correct = False  # the positions of the obstacles on the plane are not correct
                self.transformations_are_built = False  # the workspace transformations are not built
                self.hntf2d_solver = None  # the solver for the 2D obstacles avoidance problem gets initialized to the default value
                self.obstacles_boundaries_for_solver = []  # initialize the boundaries of the obstacles for the solver
                self.obstacles_height_for_solver = 0.0  # initialize the height of the obstacles for the solver
                self.chosen_workspace_saved_obstacles_objects_list = [file[:-4] for file in os.listdir(self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_solver_workspace_image_name}") if file.endswith(".txt")]  # the list to store the saved obstacles objects for the chosen workspace image
                if len(self.chosen_workspace_saved_obstacles_objects_list) != 0:  # if there are saved obstacles objects for the chosen workspace image
                    for k in range(len(self.chosen_workspace_saved_obstacles_objects_list)):  # iterate through all the saved obstacles objects
                        obstacle_object_path = self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_solver_workspace_image_name}/{self.chosen_workspace_saved_obstacles_objects_list[k]}"  # the path of the chosen obstacle object
                        chosen_obstacle_file_lines = open(obstacle_object_path + ".txt", "r", encoding = "utf-8").readlines()  # read the lines of the chosen obstacle object file
                        obstacle_boundary_points = np.array([[float(k) for k in line.split(" ")] for line in chosen_obstacle_file_lines[3:]]).reshape(-1, 2)  # the boundary points of the obstacle mesh in millimeters
                        self.obstacles_boundaries_for_solver.append(obstacle_boundary_points)  # add the obstacle boundary points to the list of the obstacles boundaries for the solver
                    self.obstacles_height_for_solver = float(open(self.saved_obstacles_objects_infos_folder_path + fr"/{self.chosen_solver_workspace_image_name}/{self.chosen_workspace_saved_obstacles_objects_list[0]}.txt", "r", encoding = "utf-8").readlines()[0].split(": ")[1])  # the height of the obstacles for the solver
                    inner_boundaries_number = len(self.chosen_workspace_saved_obstacles_objects_list) - 1  # the number of the inner boundaries (total minus the outer boundary)
                    self.boundaries_are_detected = True  # the boundaries of the obstacles are detected
                    self.obstacles_infos_text = f"✓ {inner_boundaries_number} inner and 1 outer boundaries\nObstacles height: {1000.0 * self.obstacles_height_for_solver} mm"  # the text to show the obstacles information
                else:  # if there are no saved obstacles objects for the chosen workspace image
                    self.boundaries_are_detected = False  # the boundaries of the obstacles are not detected
                    self.obstacles_infos_text = "✗ No obstacles have been detected\nyet for this workspace image!"  # the text to show that no obstacles have been detected for the chosen workspace image
            else:  # if the chosen workspace image does not exist
                ms.showerror("Error", f"The chosen workspace image \"{chosen_solver_workspace_image_name}\" does not exist!")  # show an error message
            self.workspace_image_plane_frame_wrt_world, self.workspace_image_plane_x_length, self.workspace_image_plane_y_length, self.workspace_image_plane_image_points, self.workspace_image_camera_frame_wrt_world = self.load_workspace_image_information(self.chosen_solver_workspace_image_name)  # load the information of the chosen workspace image
            self.camera_wrt_world_transformation_matrix = np.array(self.workspace_image_camera_frame_wrt_world, dtype = np.float32)  # the transformation matrix of the loaded camera frame with respect to the world frame
            self.obstacles_2d_plane_x_length = self.workspace_image_plane_x_length  # the length along the x axis of the loaded 2D plane
            self.obstacles_2d_plane_y_length = self.workspace_image_plane_y_length  # the length along the y axis of the loaded 2D plane
            self.obst_plane_wrt_world_transformation_matrix = np.array(self.workspace_image_plane_frame_wrt_world, dtype = np.float32)  # the transformation matrix of the loaded 2D plane with respect to the world frame
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def plot_obstacles_objects(self, event = None):  # plot the obstacles objects of the workspace with their real world dimensions
        if self.boundaries_are_detected:  # if the boundaries of the obstacles are detected
            chosen_solver_workspace_image_name = self.load_workspace_obstacles_objects_combobox.get()  # the name of the chosen workspace image
            p_x_length = 100.0 * self.obstacles_2d_plane_x_length; p_y_length = 100.0 * self.obstacles_2d_plane_y_length  # the lengths of the 2D obstacles plane in centimeters
            start_pos_workspace_plane = 100.0 * (self.start_pos_workspace_plane + np.array([self.obstacles_2d_plane_x_length / 2.0, self.obstacles_2d_plane_y_length / 2.0]))  # the start position of the robot's end-effector on the 2D workspace plane in centimeters
            target_pos_workspace_plane = 100.0 * (self.target_pos_workspace_plane + np.array([self.obstacles_2d_plane_x_length / 2.0, self.obstacles_2d_plane_y_length / 2.0]))  # the target position of the robot's end-effector on the 2D workspace plane in centimeters
            obstacles_boundaries_for_solver = [100.0 * boundary for boundary in self.obstacles_boundaries_for_solver]  # the boundaries of the obstacles for the solver in centimeters
            fig = plt.figure()  # create a figure
            ax = fig.add_subplot(111)  # create a subplot
            outer_plot, = ax.plot(obstacles_boundaries_for_solver[-1][:, 0] + p_x_length/2, obstacles_boundaries_for_solver[-1][:, 1] + p_y_length/2, "r", linewidth = 1)  # plot the outer obstacle boundary
            for k in range(len(obstacles_boundaries_for_solver) - 1):  # for each inner obstacle boundary
                inner_plot, = ax.plot(obstacles_boundaries_for_solver[k][:, 0] + p_x_length/2, obstacles_boundaries_for_solver[k][:, 1] + p_y_length/2, "b", linewidth = 1)  # plot the inner obstacles boundaries
                ax.text(np.mean(obstacles_boundaries_for_solver[k][:, 0]) + p_x_length/2, np.mean(obstacles_boundaries_for_solver[k][:, 1]) + p_y_length/2, f"{k + 1}", fontsize = 7, color = "k")  # write the text of the inner obstacles boundaries
            plane_plot, = ax.plot(np.array([0, p_x_length, p_x_length, 0, 0]), np.array([p_y_length, p_y_length, 0, 0, p_y_length]), "k", linewidth = 2)  # plot the 2D obstacles plane
            self.start_point_plot, = ax.plot(start_pos_workspace_plane[0], start_pos_workspace_plane[1], "mo", markersize = 5)  # plot the start position of the robot's end-effector on the 2D workspace plane
            self.target_point_plot, = ax.plot(target_pos_workspace_plane[0], target_pos_workspace_plane[1], "go", markersize = 5)  # plot the target position of the robot's end-effector on the 2D workspace plane
            self.start_point_plot_text = ax.text(start_pos_workspace_plane[0], start_pos_workspace_plane[1], "Start", fontsize = 7, color = "m")  # write the text of the start position of the robot's end-effector on the 2D workspace plane
            self.target_point_plot_text = ax.text(target_pos_workspace_plane[0], target_pos_workspace_plane[1], "Target", fontsize = 7, color = "g")  # write the text of the target position of the robot's end-effector on the 2D workspace plane
            if len(obstacles_boundaries_for_solver) > 1:  # if there are inner obstacles boundaries
                ax.legend([plane_plot, outer_plot, inner_plot], ["Obstacles plane", "Outer boundary", "Inner boundaries"], fontsize = 8, loc = "upper right")  # show the legend of the plot
            else:  # if there are no inner obstacles boundaries and there is only the outer boundary
                ax.legend([plane_plot, outer_plot], ["Obstacles plane", "Outer boundary"], fontsize = 8, loc = "upper right")  # show the legend of the plot
            ax.set_title(f"Obstacles objects on the real workspace of the image \"{chosen_solver_workspace_image_name}\", with their real dimensions in cm\n(The bottom-left corner of the plane is considered to be the point (0, 0) here)\n(use mouse left and right click to mark the start and target positions)")  # set the title of the plot
            ax.set_xlabel("x (cm)"); ax.set_ylabel("y (cm)"); ax.set_aspect("equal")  # set the labels of the x and y axis and the aspect ratio of the plot to be equal
            ax.grid(True)  # show the grid
            fig.canvas.mpl_connect("button_press_event", lambda event: self.mark_points_on_obstacles_plot(event, ax))  # connect a function to the plot
            plt.show()  # show the plot
        else:  # if there are no obstacles boundaries
            ms.showerror("Error", "No obstacles have been detected yet for this workspace image!")  # show an error message
    def mark_points_on_obstacles_plot(self, event, fig_ax):  # mark points on the obstacles plot
        if event.inaxes:  # if the event is inside the axis space
            x, y = event.xdata, event.ydata  # get the coordinates of the mouse click
            if event.button == 1:  # if the user presses mouse left click
                try:
                    self.start_point_plot.remove()  # try to remove the plotted start point
                    self.start_point_plot_text.remove()  # try to remove the plotted start point text
                except: pass
                self.start_point_plot, = fig_ax.plot(x, y, "mo", markersize = 5)  # plot the start position of the robot's end-effector on the 2D workspace plane
                self.start_point_plot_text = fig_ax.text(x, y, "Start", fontsize = 7, color = "m")  # write the text of the start position of the robot's end-effector on the 2D workspace plane
                self.start_pos_workspace_plane = np.array([x / 100.0 - self.obstacles_2d_plane_x_length / 2.0, y / 100.0 - self.obstacles_2d_plane_y_length / 2.0])  # change the start position of the robot's end-effector on the 2D workspace plane
            elif event.button == 3:  # if the user presses mouse right click
                try:
                    self.target_point_plot.remove()  # try to remove the plotted target point
                    self.target_point_plot_text.remove()  # try to remove the plotted target point text
                except: pass
                self.target_point_plot, = fig_ax.plot(x, y, "go", markersize = 5)  # plot the target position of the robot's end-effector on the 2D workspace plane
                self.target_point_plot_text = fig_ax.text(x, y, "Target", fontsize = 7, color = "g")  # write the text of the target position of the robot's end-effector on the 2D workspace plane
                self.target_pos_workspace_plane = np.array([x / 100.0 - self.obstacles_2d_plane_x_length / 2.0, y / 100.0 - self.obstacles_2d_plane_y_length / 2.0])  # change the target position of the robot's end-effector on the 2D workspace plane
            elif event.button == 2:  # if the user presses mouse middle click
                try:
                    self.random_point_plot.remove()  # try to remove the plotted random point
                    self.random_point_plot_text.remove()  # try to remove the plotted random point text
                except: pass
                self.random_point_plot, = fig_ax.plot(x, y, 'ko', markersize = 5)  # plot a random point on the 2D workspace plane
                self.random_point_plot_text = fig_ax.text(x, y, f"({x:.1f}, {y:.1f})", fontsize = 7, color = "k")  # write the coordinates of the random point
            plt.draw()  # redraw the plot
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def define_workspace_plane(self, event = None):  # define the workspace plane for the obstacles avoidance solver
        self.workspace_plane_creation_parameter = self.alternate_matrix_elements(self.workspace_plane_creation_parameters_list, self.workspace_plane_creation_parameter)  # change the workspace plane creation parameters
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def set_start_pos_on_plane(self, event = None):  # set the start position of the robot's end-effector on the 2D workspace plane for the obstacles avoidance solver
        start_pos_previous = np.array(self.start_pos_workspace_plane) + np.array([self.obstacles_2d_plane_x_length / 2.0, self.obstacles_2d_plane_y_length / 2.0])  # the previous start position of the robot's end-effector on the 2D workspace plane
        initial_x_pos_on_plane = sd.askfloat("Choose the start position on plane", f"Suppose that the origin of the plane is the bottom-left corner of the plane and the x-axis,\ny-axis are extended to the bottom-right and top-left corners of the plane, respectively.\n\
Enter the start x coordinate of the robot's end-effector on the 2D workspace plane (in meters).\nThe value must be in the range [0, {self.workspace_image_plane_x_length:.3f}]:", initialvalue = start_pos_previous[0], minvalue = 0, maxvalue = self.workspace_image_plane_x_length, parent = self.menus_area)  # ask the user to enter the start x pos of the robot's end-effector on the 2D workspace plane
        if initial_x_pos_on_plane != None:  # if the user enters a number
            self.start_pos_workspace_plane[0] = initial_x_pos_on_plane - self.obstacles_2d_plane_x_length / 2.0  # change the start x pos of the robot's end-effector on the 2D workspace plane
        initial_y_pos_on_plane = sd.askfloat("Choose the start position on plane", f"Enter the start y coordinate of the robot's end-effector on the\n2D workspace plane (in meters). The value must be in the range [0, {self.workspace_image_plane_y_length:.3f}]:", initialvalue = start_pos_previous[1], minvalue = 0, maxvalue = self.workspace_image_plane_y_length, parent = self.menus_area)  # ask the user to enter the start y pos of the robot's end-effector on the 2D workspace plane
        if initial_y_pos_on_plane != None:  # if the user enters a number
            self.start_pos_workspace_plane[1] = initial_y_pos_on_plane - self.obstacles_2d_plane_y_length / 2.0  # change the start y pos of the robot's end-effector on the 2D workspace plane
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def set_target_pos_on_plane(self, event = None):  # set the final position of the robot's end-effector on the 2D workspace plane for the obstacles avoidance solver
        target_pos_previous = np.array(self.target_pos_workspace_plane) + np.array([self.obstacles_2d_plane_x_length / 2.0, self.obstacles_2d_plane_y_length / 2.0])  # the previous target position of the robot's end-effector on the 2D workspace plane
        final_x_pos_plane = sd.askfloat("Choose the final position on plane", f"Suppose that the origin of the plane is the bottom-left corner of the plane and the x-axis,\ny-axis are extended to the bottom-right and top-left corners of the plane, respectively.\n\
Enter the final x coordinate of the robot's end-effector on the 2D workspace plane (in meters).\nThe value must be in the range [0, {self.workspace_image_plane_x_length:.3f}]:", initialvalue = target_pos_previous[0], minvalue = 0, maxvalue = self.workspace_image_plane_x_length, parent = self.menus_area)  # ask the user to enter the final x pos of the robot's end-effector on the 2D workspace plane
        if final_x_pos_plane != None:  # if the user enters a number
            self.target_pos_workspace_plane[0] = final_x_pos_plane - self.obstacles_2d_plane_x_length / 2.0  # change the final x pos of the robot's end-effector on the 2D workspace plane
        final_y_pos_plane = sd.askfloat("Choose the final position on plane", f"Enter the final y coordinate of the robot's end-effector on the\n2D workspace plane (in meters). The value must be in the range [0, {self.workspace_image_plane_y_length:.3f}]:", initialvalue = target_pos_previous[1], minvalue = 0, maxvalue = self.workspace_image_plane_y_length, parent = self.menus_area)  # ask the user to enter the final y pos of the robot's end-effector on the 2D workspace plane
        if final_y_pos_plane != None:  # if the user enters a number
            self.target_pos_workspace_plane[1] = final_y_pos_plane - self.obstacles_2d_plane_y_length / 2.0  # change the final y pos of the robot's end-effector on the 2D workspace plane
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def check_start_target_pos_on_plane(self, event = None):  # check the start and target positions of the robot's end-effector on the 2D workspace plane
        initial_target_pos_inside_outer_boundary = True  # the flag that indicates if the start and target positions are inside the outer boundary
        initial_target_pos_outside_inner_boundaries = True  # the flag that indicates if the start and target positions are outside the inner boundaries
        if len(self.obstacles_boundaries_for_solver) != 0:  # if there are obstacles boundaries
            if not pbo.points_in_polygon_detection(self.obstacles_boundaries_for_solver[-1], [self.start_pos_workspace_plane, self.target_pos_workspace_plane]).all():  # if the start and target positions are not inside the outer boundary
                initial_target_pos_inside_outer_boundary = False  # change the flag to indicate that the start and target positions are not inside the outer boundary
            for k in range(len(self.obstacles_boundaries_for_solver) - 1):  # for each inner obstacle boundary
                if pbo.points_in_polygon_detection(self.obstacles_boundaries_for_solver[k], [self.start_pos_workspace_plane, self.target_pos_workspace_plane]).any():  # if the start and target positions are inside an inner boundary
                    initial_target_pos_outside_inner_boundaries = False  # change the flag to indicate that the start and target positions are inside an inner boundary
            if initial_target_pos_inside_outer_boundary and initial_target_pos_outside_inner_boundaries:  # if the start and target positions are inside the outer boundary and outside the inner boundaries
                self.positions_on_plane_are_correct = True  # the start and target positions are set correctly
                self.check_positions_on_plane_text = "✓ Correct! The start and target positions are inside the\nouter boundary and outside the inner boundaries!"  # the text to show that the start and target positions are inside the outer boundary and outside the inner boundaries
            else:  # if the start and target positions are not inside the outer boundary or inside an inner boundary
                self.positions_on_plane_are_correct = False  # the start and target positions are not set correctly
                self.check_positions_on_plane_text = "✗ Wrong! The start and/or target positions are\nnot defined correctly! Choose new values!"  # the text to show that the start and target positions are not inside the outer boundary or inside an inner boundary
        else:  # if there are no obstacles boundaries
            self.positions_on_plane_are_correct = False  # the start and target positions are not set correctly
            self.check_positions_on_plane_text = "✗ No obstacles have been detected\nyet for this workspace image!"  # the text to show that no obstacles have been detected for the chosen workspace image            
    def start_building_workspace_transformations(self, event = None):  # start building the workspace transformations
        if self.boundaries_are_detected:  # if the boundaries of the obstacles are detected and the start and target positions are set correctly
            self.hntf2d_solver = cl.hntf2d_control_law(p_init = self.start_pos_workspace_plane, p_d = self.target_pos_workspace_plane, outer_boundary = self.obstacles_boundaries_for_solver[-1], inner_boundaries = self.obstacles_boundaries_for_solver[:-1], \
                                                        k_d = self.k_d, k_i = self.k_i, w_phi = self.w_phi, vp_max = self.vp_max, dp_min = self.dp_min)  # create a new hntf2d control law object
            self.hntf2d_solver.create_new_harmonic_map_object()  # create a new harmonic map object
            plt.close('all')  # close all the opened plots
            if not self.hntf2d_solver.harmonic_map_object_is_built:  # if the harmonic map object is not built
                ms.showerror("Error", "The harmonic map object to start the transformations process could not be built!")  # show an error message
                self.hntf2d_solver = None  # the solver for the 2D obstacles avoidance problem gets initialized to the default value
        else:  # if the boundaries of the obstacles are not detected or the start and target positions are not set correctly
            ms.showerror("Error", "The workspace transformations cannot be built because no obstacles boundaries are detected!")
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def build_realws_to_unit_disk_transformation(self, event = None):  # build the transformation from the real workspace to the unit disk
        if self.hntf2d_solver != None:  # if the hntf2d control law object has been created
            self.hntf2d_solver.realws_to_disk_transformation()  # calculate the transformation from the real workspace to the unit disk
            if not self.hntf2d_solver.wtd_transformation_is_built:  # if the transformation from the real workspace to the unit disk is not built correctly
                ms.showerror("Error", "The transformation from the real workspace to the unit disk could not be built!")  # show an error message
            test_point = self.hntf2d_solver.realws_to_unit_disk_mapping(np.zeros(2))  # test the transformation from the real workspace to the unit disk, at the origin (0, 0)
            if np.all(np.isnan(test_point)):  # if the transformation from the real workspace to the unit disk is not correct
                ms.showerror("Error", "The transformation from the real workspace to the unit disk is not correct! This can be due to the detected obstacles boundaries, so try to retake the workspace image!")  # show an error message
        else:  # if the hntf2d control law object has not been created yet
            ms.showerror("Error", "Press the \"start\" button first to begin building the workspace transformations (following the defined order)!")  # show an error message
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def build_unit_disk_to_R2_transformation(self, event = None):  # build the transformation from the unit disk to the R2 plane
        if self.hntf2d_solver != None:  # if the hntf2d control law object has been created
            self.hntf2d_solver.disk_to_R2_transformation()  # calculate the transformation from the unit disk to the R2 plane
            if not self.hntf2d_solver.dtp_transformation_is_built:  # if the transformation from the unit disk to the R2 plane is not built correctly
                ms.showerror("Error", "The transformation from the unit disk to the R2 plane could not be built!")  # show an error message
        else:  # if the hntf2d control law object has not been created yet
            ms.showerror("Error", "Press the \"start\" button first to begin building the workspace transformations (following the defined order)!")  # show an error message
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def realws_to_unit_disk_plot_interact(self, event = None):  # show the details of the transformation from the real workspace to the unit disk
        if self.hntf2d_solver != None:  # if the hntf2d control law object has been created
            if self.hntf2d_solver.wtd_transformation_is_built:  # if the transformation from the real workspace to the unit disk is built
                text_margin = 0.01; text_fontsize = 8; legend_fontsize = 7; outer_boundaries_linewidth = 1; inner_boundaries_linewidth = 1; positions_markersize = 5; punctures_markersize = 4  # some plot parameters
                path_linewidth = 2; path_fontsize = text_fontsize+1; path_positions_markersize = positions_markersize+1  # some plot parameters
                fig = plt.figure()
                # plot the real workspace
                ax1 = fig.add_subplot(121)
                outer_plot, = ax1.plot(self.hntf2d_solver.outer_boundary[:, 0], self.hntf2d_solver.outer_boundary[:, 1], "red", linewidth = outer_boundaries_linewidth)
                for i in range(len(self.hntf2d_solver.inner_boundaries)):
                    inner_plot, = ax1.plot(self.hntf2d_solver.inner_boundaries[i][:, 0], self.hntf2d_solver.inner_boundaries[i][:, 1], "blue", linewidth = inner_boundaries_linewidth)
                    ax1.text(np.mean(self.hntf2d_solver.inner_boundaries[i][:, 0]), np.mean(self.hntf2d_solver.inner_boundaries[i][:, 1]), f"{i+1}", fontsize = text_fontsize)
                p_init_plot, = ax1.plot(self.hntf2d_solver.p_init[0], self.hntf2d_solver.p_init[1], "magenta", marker = "s", markersize = path_positions_markersize)
                p_d_plot, = ax1.plot(self.hntf2d_solver.p_d[0], self.hntf2d_solver.p_d[1], "green", marker = "s", markersize = path_positions_markersize)
                if len(self.realws_path_control_law_output) != 0:  # if at least one path has been calculated by the control law
                    if self.all_paths_are_shown:  # if all paths are shown
                        for k, path in enumerate(self.realws_paths_list):  # for each path calculated by the control law
                            if self.path_chosen != k:  # if the current path is not the chosen one
                                realws_path_plot, = ax1.plot(np.array(path)[:, 0], np.array(path)[:, 1], self.paths_colors[k], linewidth = path_linewidth)
                            ax1.text(np.array(path)[int(len(path)/2), 0], np.array(path)[int(len(path)/2), 1], f"{k + 1}", fontweight = "bold", fontsize = path_fontsize)
                            p_init_plot, = ax1.plot(np.array(path)[0, 0], np.array(path)[0, 1], "magenta", marker = "s", markersize = positions_markersize)
                            p_d_plot, = ax1.plot(np.array(path)[-1, 0], np.array(path)[-1, 1], "green", marker = "s", markersize = positions_markersize)
                    realws_path_plot, = ax1.plot(np.array(self.realws_path_control_law_output)[:, 0], np.array(self.realws_path_control_law_output)[:, 1], "black", linewidth = path_linewidth)
                    if len(self.hntf2d_solver.inner_boundaries) != 0:  # if there are inner boundaries
                        ax1.legend([outer_plot, inner_plot, p_init_plot, p_d_plot, realws_path_plot], ["Outer boundary", "Inner boundaries", "Start position", "Target position", "Control law path"], fontsize = legend_fontsize, loc = "upper right")
                    else:  # if there are no inner boundaries
                        ax1.legend([outer_plot, p_init_plot, p_d_plot, realws_path_plot], ["Outer boundary", "Start position", "Target position", "Control law path"], fontsize = legend_fontsize, loc = "upper right")
                else:  # if no path has been calculated yet by the control law
                    if len(self.hntf2d_solver.inner_boundaries) != 0:  # if there are inner boundaries
                        ax1.legend([outer_plot, inner_plot, p_init_plot, p_d_plot], ["Outer boundary", "Inner boundaries", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                    else:  # if there are no inner boundaries
                        ax1.legend([outer_plot, p_init_plot, p_d_plot], ["Outer boundary", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                ax1.set_title(f"Original real workspace")
                ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_aspect("equal"); ax1.grid(True)
                # plot the unit disk
                ax2 = fig.add_subplot(122)
                unit_disk_plot, = ax2.plot(self.hntf2d_solver.unit_circle[:, 0], self.hntf2d_solver.unit_circle[:, 1], "red", linewidth = outer_boundaries_linewidth)
                for i in range(len(self.hntf2d_solver.q_i_disk)):
                    punctures_plot, = ax2.plot(self.hntf2d_solver.q_i_disk[i][0], self.hntf2d_solver.q_i_disk[i][1], "blue", marker = "o", markersize = punctures_markersize)
                    ax2.text(self.hntf2d_solver.q_i_disk[i][0] + text_margin, self.hntf2d_solver.q_i_disk[i][1] + text_margin, f"{i+1}", fontsize = text_fontsize)
                q_init_plot, = ax2.plot(self.hntf2d_solver.q_init_disk[0], self.hntf2d_solver.q_init_disk[1], "magenta", marker = "s", markersize = path_positions_markersize)
                q_d_plot, = ax2.plot(self.hntf2d_solver.q_d_disk[0], self.hntf2d_solver.q_d_disk[1], "green", marker = "s", markersize = path_positions_markersize)
                if len(self.unit_disk_path_control_law_output) != 0:  # if at least one path has been calculated by the control law
                    if self.all_paths_are_shown:  # if all paths are shown
                        for k, path in enumerate(self.unit_disk_paths_list):  # for each path calculated by the control law
                            if self.path_chosen != k:  # if the current path is not the chosen one
                                unit_disk_path_plot, = ax2.plot(np.array(path)[:, 0], np.array(path)[:, 1], self.paths_colors[k], linewidth = path_linewidth)
                            ax2.text(np.array(path)[int(len(path)/2), 0], np.array(path)[int(len(path)/2), 1], f"{k + 1}", fontweight = "bold", fontsize = path_fontsize)
                            q_init_plot, = ax2.plot(np.array(path)[0, 0], np.array(path)[0, 1], "magenta", marker = "s", markersize = positions_markersize)
                            q_d_plot, = ax2.plot(np.array(path)[-1, 0], np.array(path)[-1, 1], "green", marker = "s", markersize = positions_markersize)
                    unit_disk_path_plot, = ax2.plot(np.array(self.unit_disk_path_control_law_output)[:, 0], np.array(self.unit_disk_path_control_law_output)[:, 1], "black", linewidth = path_linewidth)
                    if len(self.hntf2d_solver.q_i_disk) != 0:  # if there are inner boundaries
                        ax2.legend([unit_disk_plot, punctures_plot, q_init_plot, q_d_plot, unit_disk_path_plot], ["Unit disk", "Punctures", "Start position", "Target position", "Control law path"], fontsize = legend_fontsize, loc = "upper right")
                    else:  # if there are no inner boundaries
                        ax2.legend([unit_disk_plot, q_init_plot, q_d_plot, unit_disk_path_plot], ["Unit disk", "Start position", "Target position", "Control law path"], fontsize = legend_fontsize, loc = "upper right")
                else:  # if no path has been calculated yet by the control law
                    if len(self.hntf2d_solver.q_i_disk) != 0:  # if there are inner boundaries
                        ax2.legend([unit_disk_plot, punctures_plot, q_init_plot, q_d_plot], ["Unit disk", "Punctures", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                    else:  # if there are no inner boundaries
                        ax2.legend([unit_disk_plot, q_init_plot, q_d_plot], ["Unit disk", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                ax2.set_title(f"Real workspace ->\n-> Unit disk transformation")
                ax2.set_xlabel("u"); ax2.set_ylabel("v"); ax2.set_aspect("equal"); ax2.grid(True)
                fig.canvas.mpl_connect("button_press_event", lambda event: self.transformations_plots_buttons_actions(event, "realws_to_unit_disk", fig, ax1, ax2))  # connect a function to the plot
                fig.canvas.mpl_connect("key_press_event", lambda event: self.transformations_plots_keys_actions(event, "realws_to_unit_disk", fig, ax1, ax2))  # connect a function to the plot
                plt.show()
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def unit_disk_to_R2_plot_interact(self, event = None):  # show the details of the transformation from the unit disk to the R2 plane
        if self.hntf2d_solver != None:  # if the hntf2d control law object has been created
            if self.hntf2d_solver.dtp_transformation_is_built:  # if the transformation from the unit disk to the R2 plane is built
                text_margin = 0.01; text_fontsize = 8; legend_fontsize = 7; unit_circle_linewidth = 1; positions_markersize = 5; punctures_markersize = 4  # some plot parameters
                path_linewidth = 2; path_fontsize = text_fontsize+1; path_positions_markersize = positions_markersize+1  # some plot parameters
                fig = plt.figure()
                # plot the unit disk
                ax1 = fig.add_subplot(121)
                unit_disk_plot, = ax1.plot(self.hntf2d_solver.unit_circle[:, 0], self.hntf2d_solver.unit_circle[:, 1], "red", linewidth = unit_circle_linewidth)
                for i in range(len(self.hntf2d_solver.q_i_disk)):
                    punctures_plot, = ax1.plot(self.hntf2d_solver.q_i_disk[i][0], self.hntf2d_solver.q_i_disk[i][1], "blue", marker = "o", markersize = punctures_markersize)
                    ax1.text(self.hntf2d_solver.q_i_disk[i][0] + text_margin, self.hntf2d_solver.q_i_disk[i][1] + text_margin, f"{i+1}", fontsize = text_fontsize)
                q_init_disk_plot, = ax1.plot(self.hntf2d_solver.q_init_disk[0], self.hntf2d_solver.q_init_disk[1], "magenta", marker = "s", markersize = path_positions_markersize)
                q_d_disk_plot, = ax1.plot(self.hntf2d_solver.q_d_disk[0], self.hntf2d_solver.q_d_disk[1], "green", marker = "s", markersize = path_positions_markersize)
                if len(self.unit_disk_path_control_law_output) != 0:  # if at least one path has been calculated by the control law
                    if self.all_paths_are_shown:  # if all paths are shown
                        for k, path in enumerate(self.unit_disk_paths_list):  # for each path calculated by the control law
                            if self.path_chosen != k:  # if the current path is not the chosen one
                                unit_disk_path_plot, = ax1.plot(np.array(path)[:, 0], np.array(path)[:, 1], self.paths_colors[k], linewidth = path_linewidth)
                            ax1.text(np.array(path)[int(len(path)/2), 0], np.array(path)[int(len(path)/2), 1], f"{k + 1}", fontweight = "bold", fontsize = path_fontsize)
                            q_init_disk_plot, = ax1.plot(np.array(path)[0, 0], np.array(path)[0, 1], "magenta", marker = "s", markersize = positions_markersize)
                            q_d_disk_plot, = ax1.plot(np.array(path)[-1, 0], np.array(path)[-1, 1], "green", marker = "s", markersize = positions_markersize)
                    unit_disk_path_plot, = ax1.plot(np.array(self.unit_disk_path_control_law_output)[:, 0], np.array(self.unit_disk_path_control_law_output)[:, 1], "black", linewidth = path_linewidth)
                    if len(self.hntf2d_solver.q_i_disk) != 0:  # if there are inner boundaries
                        ax1.legend([unit_disk_plot, punctures_plot, q_init_disk_plot, q_d_disk_plot, unit_disk_path_plot], ["Unit disk", "Punctures", "Start position", "Target position", "Control law path"], fontsize = legend_fontsize, loc = "upper right")
                    else:  # if there are no inner boundaries
                        ax1.legend([unit_disk_plot, q_init_disk_plot, q_d_disk_plot, unit_disk_path_plot], ["Unit disk", "Start position", "Target position", "Control law path"], fontsize = legend_fontsize, loc = "upper right")
                else:  # if no path has been calculated yet by the control law
                    if len(self.hntf2d_solver.q_i_disk) != 0:  # if there are inner boundaries
                        ax1.legend([unit_disk_plot, punctures_plot, q_init_disk_plot, q_d_disk_plot], ["Unit disk", "Punctures", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                    else:  # if there are no inner boundaries
                        ax1.legend([unit_disk_plot, q_init_disk_plot, q_d_disk_plot], ["Unit disk", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                ax1.set_title(f"Real workspace ->\n-> Unit disk transformation")
                ax1.set_xlabel("u"); ax1.set_ylabel("v"); ax1.set_aspect("equal")
                ax1.grid(True)
                # plot the R2 plane
                ax2 = fig.add_subplot(122)
                for i in range(len(self.hntf2d_solver.q_i)):
                    displayed_punctures_plot, = ax2.plot(self.hntf2d_solver.q_i[i][0], self.hntf2d_solver.q_i[i][1], "blue", marker = "o", markersize = punctures_markersize)
                    ax2.text(self.hntf2d_solver.q_i[i][0] + 3.0*text_margin, self.hntf2d_solver.q_i[i][1] + 3.0*text_margin, f"{i+1}", fontsize = text_fontsize)
                q_init_R2_plot, = ax2.plot(self.hntf2d_solver.q_init[0], self.hntf2d_solver.q_init[1], "magenta", marker = "s", markersize = path_positions_markersize)
                q_d_R2_plot, = ax2.plot(self.hntf2d_solver.q_d[0], self.hntf2d_solver.q_d[1], "green", marker = "s", markersize = path_positions_markersize)
                if len(self.R2_plane_path_control_law_output) != 0:  # if at least one path has been calculated by the control law
                    if self.all_paths_are_shown:  # if all paths are shown
                        for k, path in enumerate(self.R2_plane_paths_list):  # for each path calculated by the control law
                            if self.path_chosen != k:  # if the current path is not the chosen one
                                R2_plane_path_plot, = ax2.plot(np.array(path)[:, 0], np.array(path)[:, 1], self.paths_colors[k], linewidth = path_linewidth)
                            ax2.text(np.array(path)[int(len(path)/2), 0], np.array(path)[int(len(path)/2), 1], f"{k + 1}", fontweight = "bold", fontsize = path_fontsize)
                            q_init_R2_plot, = ax2.plot(np.array(path)[0, 0], np.array(path)[0, 1], "magenta", marker = "s", markersize = positions_markersize)
                            q_d_R2_plot, = ax2.plot(np.array(path)[-1, 0], np.array(path)[-1, 1], "green", marker = "s", markersize = positions_markersize)
                    R2_plane_path_plot, = ax2.plot(np.array(self.R2_plane_path_control_law_output)[:, 0], np.array(self.R2_plane_path_control_law_output)[:, 1], "black", linewidth = path_linewidth)
                    if len(self.hntf2d_solver.q_i) != 0:  # if there are inner boundaries
                        ax2.legend([displayed_punctures_plot, q_init_R2_plot, q_d_R2_plot, R2_plane_path_plot], ["Punctures", "Start position", "Target position", "Control law path"], fontsize = legend_fontsize, loc = "upper right")
                    else:  # if there are no inner boundaries
                        ax2.legend([q_init_R2_plot, q_d_R2_plot, R2_plane_path_plot], ["Start position", "Target position", "Control law path"], fontsize = legend_fontsize, loc = "upper right")
                else:  # if no path has been calculated yet by the control law
                    if len(self.hntf2d_solver.q_i) != 0:  # if there are inner boundaries
                        ax2.legend([displayed_punctures_plot, q_init_R2_plot, q_d_R2_plot], ["Punctures", "Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                    else:  # if there are no inner boundaries
                        ax2.legend([q_init_R2_plot, q_d_R2_plot], ["Start position", "Target position"], fontsize = legend_fontsize, loc = "upper right")
                ax2.set_title(f"Unit disk ->\n-> R2 plane transformation")
                ax2.set_xlabel("x"); ax2.set_ylabel("y"); ax2.set_aspect("equal")
                points_max_norm = max([np.linalg.norm(self.hntf2d_solver.q_i[k]) for k in range(len(self.hntf2d_solver.q_i))] + [np.linalg.norm(self.hntf2d_solver.q_init), np.linalg.norm(self.hntf2d_solver.q_d)])  # calculate the maximum norm of the points
                ax2.set_xlim(-1.5 * points_max_norm, 1.5 * points_max_norm); ax2.set_ylim(-1.5 * points_max_norm, 1.5 * points_max_norm)
                ax2.grid(True)
                fig.canvas.mpl_connect("button_press_event", lambda event: self.transformations_plots_buttons_actions(event, "unit_disk_to_R2", fig, ax1, ax2))  # connect a function to the plot
                fig.canvas.mpl_connect("key_press_event", lambda event: self.transformations_plots_keys_actions(event, "unit_disk_to_R2", fig, ax1, ax2))  # connect a function to the plot
                plt.show()
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def transformations_plots_buttons_actions(self, event, transformation, fig, fig_ax1, fig_ax2):  # handle the mouse clicks events in the transformations plots
        if event.inaxes:  # if the event is inside the axis space
            x, y = event.xdata, event.ydata  # get the coordinates of the mouse click
            # handle mouse clicks events
            if event.button == 2:  # if the user presses mouse middle click
                try:
                    self.input_point_plot.remove()  # try to remove the plotted input point
                    self.input_point_plot_text.remove()  # try to remove the plotted input point text
                    self.output_point_plot.remove()  # try to remove the plotted output point
                    self.output_point_plot_text.remove()  # try to remove the plotted output point text
                except: pass
                input_point = np.array([x, y])  # the input point
                if transformation == "realws_to_unit_disk":  # if the transformation is from the real workspace to the unit disk
                    output_point = self.hntf2d_solver.realws_to_unit_disk_mapping(input_point)  # map the input point to the proper output_point
                elif transformation == "unit_disk_to_R2":  # if the transformation is from the unit disk to the R2 plane
                    output_point = self.hntf2d_solver.unit_disk_to_R2_mapping(input_point)  # map the input point to the proper output_point
                self.input_point_plot, = fig_ax1.plot(x, y, "ko", markersize = 5)  # plot the input point
                self.input_point_plot_text = fig_ax1.text(x, y, f"({x:.3f}, {y:.3f})", fontsize = 7, color = "k")  # write the text of the input point
                self.output_point_plot, = fig_ax2.plot(output_point[0], output_point[1], "ko", markersize = 5)  # plot the output point
                self.output_point_plot_text = fig_ax2.text(output_point[0], output_point[1], f"({output_point[0]:.3f}, {output_point[1]:.3f})", fontsize = 7, color = "k")  # write the text of the output point
                plt.draw()  # redraw the plot
    def transformations_plots_keys_actions(self, event, transformation, fig, fig_ax1, fig_ax2):  # handle the key press events in the transformations plots
        if event.inaxes:  # if the event is inside the axis space
            # handle key press events
            if event.key in ["a", "A"]:  # if the user presses "a" or "A" key
                self.all_paths_are_shown = not self.all_paths_are_shown  # change the flag to show all the paths
            elif event.key in ["d", "D"]:  # if the user presses "d" or "D" key
                self.remove_result_from_control_law_outputs(self.path_chosen)  # remove the chosen path from the control law outputs list
                self.path_chosen = 0  # choose the first path in the list of the remaining real workspace paths
                self.choose_result_from_control_law_outputs(self.path_chosen)  # select the new chosen path from the control law outputs list
            elif event.key in ["g", "G"]:  # if the user presses "g" or "G" key
                ms.showinfo("Control law chosen path info", f"Chosen path: {self.path_chosen + 1}\n" + self.show_chosen_path_information(self.path_chosen), parent = self.menus_area)  # show an info message
            elif event.key in [str(i) for i in range(1, 10)]:  # if the user presses a number key from 1 to 9
                if int(event.key) <= len(self.realws_paths_list):  # if a valid path number is chosen
                    self.path_chosen = int(event.key) - 1  # change the chosen path
                    self.choose_result_from_control_law_outputs(self.path_chosen)  # select the new chosen path from the control law outputs list
            if event.key in ["a", "A", "d", "D"] or event.key in [str(i) for i in range(1, 10)]:  # if the user presses a proper key
                plt.close(fig)  # close the current plot
                if transformation == "realws_to_unit_disk":  # if the transformation is from the real workspace to the unit disk
                    self.realws_to_unit_disk_plot_interact()  # show the details of the transformation from the real workspace to the unit disk again
                elif transformation == "unit_disk_to_R2":  # if the transformation is from the unit disk to the R2 plane
                    self.unit_disk_to_R2_plot_interact()  # show the details of the transformation from the unit disk to the R2 plane again
    def choose_result_from_control_law_outputs(self, chosen_path_index, event = None):  # choose an output result from the control law outputs list
        if len(self.realws_paths_list) != 0:  # if at least one path has been calculated by the control law
            if not self.solver_enable_error_correction: last_index = -1  # if the error correction is not enabled
            else: last_index = len(self.realws_paths_list[chosen_path_index])  # if the error correction is enabled
            if chosen_path_index < len(self.realws_paths_list):  # if the chosen path index is valid
                self.realws_path_control_law_output = copy.deepcopy(self.realws_paths_list[chosen_path_index][:last_index])  # change the real workspace path control law output
                self.unit_disk_path_control_law_output = copy.deepcopy(self.unit_disk_paths_list[chosen_path_index][:last_index])  # change the unit disk path control law output
                self.R2_plane_path_control_law_output = copy.deepcopy(self.R2_plane_paths_list[chosen_path_index][:last_index])  # change the R2 plane path control law output
                self.realws_velocities_control_law_output = copy.deepcopy(self.realws_velocities_sequences_list[chosen_path_index][:last_index])  # change the real workspace velocities control law output
                self.start_pos_workspace_plane = copy.deepcopy(self.realws_paths_list[chosen_path_index][0])  # change the start position of the workspace plane
                self.target_pos_workspace_plane = copy.deepcopy(self.realws_paths_list[chosen_path_index][-1])  # change the target position of the workspace plane
                self.refresh_obstacles_avoidance_solver_parameters()  # refresh the obstacles avoidance solver parameters
    def remove_result_from_control_law_outputs(self, removed_path_index, event = None):  # remove an output result from the control law outputs list
        if len(self.realws_paths_list) > 1 and removed_path_index < len(self.realws_paths_list):  # if there is more than one path in the list of the real workspace paths and the removed path index is valid
            self.realws_paths_list.pop(removed_path_index)  # remove the chosen path from the list of the real workspace paths
            self.unit_disk_paths_list.pop(removed_path_index)  # remove the chosen path from the list of the unit disk paths
            self.R2_plane_paths_list.pop(removed_path_index)  # remove the chosen path from the list of the R2 plane paths
            self.realws_velocities_sequences_list.pop(removed_path_index)  # remove the chosen path velocities from the list of the real workspace velocities
            self.paths_parameters_list.pop(removed_path_index)  # remove the chosen path parameters from the list of the paths parameters
    def show_chosen_path_information(self, chosen_path_index, event = None):  # show some information about the chosen path
        if chosen_path_index < len(self.realws_paths_list):  # if the chosen path index is valid
            chosen_path = self.realws_paths_list[chosen_path_index]  # the chosen path
            path_start = chosen_path[0]  # the start position of the path
            path_stop = chosen_path[-2]  # the final position of the path
            pd = chosen_path[-1]  # the desired target-position
            solver_time_step_dt = float(np.linalg.norm(chosen_path[1] - path_start) / np.linalg.norm(self.realws_velocities_sequences_list[chosen_path_index][0]))  # the solver time step dt
            if not self.solver_enable_error_correction:  # if the error correction is not enabled
                chosen_path = chosen_path[:-1]  # remove the desired target-position
            kd, ki, w_phi, dp_min, vp_max = copy.deepcopy(self.paths_parameters_list[chosen_path_index])  # the control law parameters
            info_text = f"• Initial pos (cm): {np.round(100.0 * path_start, 1)}, Target pos (cm): {np.round(100.0 * pd, 1)}\n\
• Control law parameters:\n\
  - kd = {kd:.2f}\n\
  - ki = {np.round(ki, 2)}\n\
  - w_phi = {w_phi:.2f}\n\
  - dp_min (cm) = {100.0 * dp_min:.1f}\n\
  - vp_max (cm/sec)= {100.0 * vp_max:.1f}\n\
• Number of iterations = {len(chosen_path)}\n\
• Convergence error (cm) = {100.0 * (np.linalg.norm(pd - path_stop)):.2f}\n\
• Total path time (sec) = {np.round(len(chosen_path) * solver_time_step_dt, 3)}\n\
• Total path length (cm) = {np.round(100.0 * np.sum([np.linalg.norm(chosen_path[k] - chosen_path[k - 1]) for k in range(1, len(chosen_path))]), 1)}"
            return info_text  # return the chosen path information
        else:  # if the chosen path index is not valid
            return "The path is not valid!"  # return an error message
    def clear_control_law_outputs_lists(self, event = None):  # clear the control law outputs list
        self.realws_path_control_law_output = []  # reset the real workspace path control law output
        self.unit_disk_path_control_law_output = []  # reset the unit disk path control law output
        self.R2_plane_path_control_law_output = []  # reset the R2 plane path control law output
        self.realws_velocities_control_law_output = []  # reset the real workspace velocities control law output
        self.robot_joints_control_law_output = []  # reset the robot joints control law output
        self.realws_paths_list = []  # clear the list of the real workspace paths
        self.unit_disk_paths_list = []  # clear the list of the unit disk paths
        self.R2_plane_paths_list = []  # clear the list of the R2 plane paths
        self.realws_velocities_sequences_list = []  # clear the list of the real workspace velocities
        self.paths_parameters_list = []  # clear the list of the paths parameters
    def refresh_obstacles_avoidance_solver_parameters(self, event = None):  # refresh the obstacles avoidance solver parameters
        if self.hntf2d_solver != None:  # if the hntf2d control law object has been created
            self.hntf2d_solver.p_init = copy.deepcopy(self.start_pos_workspace_plane)  # change the initial position of the obstacles avoidance solver
            self.hntf2d_solver.p_d = copy.deepcopy(self.target_pos_workspace_plane)  # change the target position of the obstacles avoidance solver
            self.hntf2d_solver.k_d = self.k_d  # change the k_d parameter of the obstacles avoidance solver
            self.hntf2d_solver.k_i = copy.deepcopy(self.k_i)  # change the k_i parameters of the obstacles avoidance solver
            self.hntf2d_solver.w_phi = self.w_phi  # change the w_phi parameter of the obstacles avoidance solver
            self.hntf2d_solver.vp_max = self.vp_max  # change the vp_max parameter of the obstacles avoidance solver
            self.hntf2d_solver.dp_min = self.dp_min  # change the dp_min parameter of the obstacles avoidance solver
            self.hntf2d_solver.start_target_positions_transformations()  # transform the start and target positions
    def check_workspace_transformations_built(self, event = None):  # check which workspace transformations have been built
        if self.hntf2d_solver == None:  # if the hntf2d control law object has not been created yet
            self.check_transformations_built_text = "Press start!"  # change the text of the label that shows the built transformations
        if self.hntf2d_solver != None:  # if the hntf2d control law object has been created
            self.transformations_are_built = False  # the workspace transformations are not built
            if not self.hntf2d_solver.wtd_transformation_is_built and not self.hntf2d_solver.dtp_transformation_is_built:  # if both transformations are not built
                self.check_transformations_built_text = "1. ✗     2. ✗"  # change the text of the label that shows the built transformations
            else:  # if at least one transformation is built
                if self.hntf2d_solver.wtd_transformation_is_built:  # if the transformation from the real workspace to the unit disk is built
                    self.check_transformations_built_text = "1. ✓     2. ✗"  # change the text of the label that shows the built transformations
                if self.hntf2d_solver.dtp_transformation_is_built:  # if the transformation from the unit disk to the R2 plane is built
                    self.check_transformations_built_text = "1. ✗     2. ✓"  # change the text of the label that shows the built transformations
                if self.hntf2d_solver.wtd_transformation_is_built and self.hntf2d_solver.dtp_transformation_is_built:  # if both transformations are built
                    self.transformations_are_built = True  # the workspace transformations are built
                    self.check_transformations_built_text = "1. ✓     2. ✓"  # change the text of the label that shows the built transformations
    def change_k_d_parameter(self, event = None):  # change the k_d parameter of the obstacles avoidance solver
        k_d_parameter = sd.askfloat("Change the k_d parameter", "Enter the new value of the k_d parameter:", initialvalue = self.k_d, minvalue = self.k_d_limits[0], maxvalue = self.k_d_limits[1], parent = self.menus_area)  # ask the user to enter the new value of the k_d parameter
        if k_d_parameter != None:  # if the user enters a number
            self.k_d = k_d_parameter  # change the k_d parameter of the obstacles avoidance solver
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def change_chosen_k_i_number(self, event = None):  # change the chosen k_i number of the obstacles avoidance solver
        self.k_i_chosen_num = self.choose_k_i_parameters_combobox.current() + 1  # change the chosen k_i number of the obstacles avoidance solver
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def change_chosen_k_i_value(self, event = None):  # change the chosen k_i value of the obstacles avoidance solver
        try:  # try to get the entered value
            if len(self.k_i) != 0:  # if there are k_i parameters
                new_k_i_value = float(self.choose_k_i_parameters_combobox.get())  # the chosen k_i parameter value
                self.k_i[self.k_i_chosen_num - 1] = new_k_i_value  # change the chosen k_i value of the obstacles avoidance solver
                if new_k_i_value < self.k_i_limits[0]:  # if the entered value is less than the minimum limit
                    self.k_i[self.k_i_chosen_num - 1] = self.k_i_limits[0]  # change the chosen k_i value to the minimum limit
                elif new_k_i_value > self.k_i_limits[1]:  # if the entered value is greater than the maximum limit
                    self.k_i[self.k_i_chosen_num - 1] = self.k_i_limits[1]  # change the chosen k_i value to the maximum limit
        except:  # if the entered value is not a number
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def change_w_phi_parameter(self, event = None):  # change the w_phi parameter of the obstacles avoidance solver
        w_phi_parameter = sd.askfloat("Change the w_phi parameter", "Enter the new value of the w_phi parameter:", initialvalue = self.w_phi, minvalue = self.w_phi_limits[0], maxvalue = self.w_phi_limits[1], parent = self.menus_area)  # ask the user to enter the new value of the w_phi parameter
        if w_phi_parameter != None:  # if the user enters a number
            self.w_phi = w_phi_parameter  # change the w_phi parameter of the obstacles avoidance solver
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def change_vp_max_parameter(self, event = None):  # change the vp_max parameter of the obstacles avoidance solver
        vp_max_parameter = sd.askfloat("Change the vp_max parameter", "Enter the new value of the vp_max parameter:", initialvalue = 100.0 * self.vp_max, minvalue = 100.0 * self.vp_max_limits[0], maxvalue = 100.0 * self.vp_max_limits[1], parent = self.menus_area)  # ask the user to enter the new value of the vp_max parameter
        if vp_max_parameter != None:  # if the user enters a number
            self.vp_max = vp_max_parameter / 100.0  # change the vp_max parameter (in meters/sec) of the obstacles avoidance solver
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def change_dp_min_parameter(self, event = None):  # change the dp_min parameter of the obstacles avoidance solver
        dp_min_parameter = sd.askfloat("Change the dp_min parameter", "Enter the new value of the dp_min parameter:", initialvalue = 100.0 * self.dp_min, minvalue = 100.0 * self.dp_min_limits[0], maxvalue = 100.0 * self.dp_min_limits[1], parent = self.menus_area)  # ask the user to enter the new value of the dp_min parameter
        if dp_min_parameter != None:  # if the user enters a number
            self.dp_min = dp_min_parameter / 100.0  # change the dp_min parameter (in meters) of the obstacles avoidance solver
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def change_solver_dt(self, event = None):  # change the time step for the control law
        solver_dt = sd.askfloat("Choose the solver time step", "Enter the time step dt (in seconds)\nfor the control law:", initialvalue = self.solver_time_step_dt, minvalue = self.solver_time_step_dt_limits[0], maxvalue = self.solver_time_step_dt_limits[1], parent = self.menus_area)  # ask the user to enter the new value of the solver's time step
        if solver_dt != None:  # if the user enters a number
            self.solver_time_step_dt = solver_dt  # change the time step for the control law
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def change_solver_max_iter(self, event = None):  # change the maximum number of iterations for the control law
        solver_max_iter = sd.askinteger("Choose the solver maximum iterations", "Enter the maximum number of iterations\nfor the control law:", initialvalue = self.solver_maximum_iterations, minvalue = self.solver_maximum_iterations_limits[0], maxvalue = self.solver_maximum_iterations_limits[1], parent = self.menus_area)  # ask the user to enter the new value of the solver's maximum number of iterations
        if solver_max_iter != None:  # if the user enters a number
            self.solver_maximum_iterations = solver_max_iter  # change the maximum number of iterations for the control law
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def calculate_maximum_path_time(self, event = None):  # calculate the maximum path time (in seconds) allowed by the solver 
        return self.solver_time_step_dt * self.solver_maximum_iterations  # return the maximum path time (in seconds) allowed by the solver
    def calculate_maximum_path_length(self, event = None):  # calculate the maximum path length (in meters) allowed by the solver
        return self.vp_max * self.calculate_maximum_path_time()  # return the maximum path length (in meters) allowed by the solver
    def change_solver_error_tol(self, event = None):  # change the error tolerance for the control law
        solver_error_tol = sd.askfloat("Choose the solver error tolerance", "Enter the error tolerance (in centimeters)\nfor the target position convergence of the control law:", initialvalue = 100.0 * self.solver_error_tolerance, minvalue = 100.0 * self.solver_error_tolerance_limits[0], maxvalue = 100.0 * self.solver_error_tolerance_limits[1], parent = self.menus_area)  # ask the user to enter the new value of the solver's error tolerance
        if solver_error_tol != None:  # if the user enters a number
            self.solver_error_tolerance = solver_error_tol / 100.0  # change the error tolerance for the control law
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def change_solver_error_correction(self, event = None):  # change the error correction state for the control law
        self.solver_enable_error_correction = not self.solver_enable_error_correction  # change the error correction state for the control law
        self.choose_result_from_control_law_outputs(self.path_chosen)  # select the new chosen path from the control law outputs list
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def save_control_law_parameters(self, event = None):  # save the control law parameters of the obstacles avoidance solver
        ask_file_name = sd.askstring("Save control law parameters", "Enter the file name to save the control law parameters:", initialvalue = f"parameters_{len(self.saved_control_law_parameters_list) + 1}", parent = self.menus_area)  # ask the user to enter the file name to save the control law parameters
        if ask_file_name != None and ask_file_name != "":  # if the user enters a file name
            overwrite_file_accept = False  # the user's choice to overwrite the file
            if (ask_file_name + ".txt") in os.listdir(self.saved_control_law_parameters_folder_path):  # if the file already exists
                overwrite_file_accept = ms.askyesno("Overwrite file", f"The file \"{ask_file_name}.txt\" already exists. Do you want to overwrite it?", parent = self.menus_area)  # ask the user if they want to overwrite the file
            if (ask_file_name + ".txt") not in os.listdir(self.saved_control_law_parameters_folder_path) or overwrite_file_accept:  # if the file does not exist or the user wants to overwrite it
                with open(self.saved_control_law_parameters_folder_path + fr"/{ask_file_name}.txt", "w", encoding = "utf-8") as file:  # open the file in write mode
                    file.write(f"k_d: {self.k_d:.2f}\n")  # write the k_d parameter to the file
                    file.write(f"k_i: " + str([float(f"{self.k_i[k]:.2f}") for k in range(len(self.k_i))]) + "\n")  # write the k_i parameters to the file
                    file.write(f"w_phi: {self.w_phi:.2f}\n")  # write the w_phi parameter to the file
                    file.write(f"dp_min: {self.dp_min:.3f}\n")  # write the dp_min parameter to the file
                    file.write(f"vp_max: {self.vp_max:.3f}\n")  # write the vp_max parameter to the file
                file.close()  # close the file
                if ask_file_name not in self.saved_control_law_parameters_list:  # if the name of the saved file is not in the values of the combobox that contains the names of the saved files
                    self.saved_control_law_parameters_list.append(ask_file_name)  # add the file name to the list of the saved files
                ms.showinfo("Control law parameters saved!", f"The control law parameters have been saved successfully in \"{ask_file_name}.txt'!", parent = self.menus_area)  # show an info message that the control law parameters have been saved successfully
            else:  # if the file exists and the user does not want to overwrite it
                ms.showinfo("File not saved", "The file has not been saved!", parent = self.menus_area)  # show an information message
        else:  # if the user does not enter a name for the file
            ms.showerror("Error", "You have not entered a name for the file!", parent = self.menus_area)  # show an error message
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def load_control_law_parameters(self, event = None):  # load the control law parameters of the obstacles avoidance solver
        load_file_accept = False  # the user's choice to load the file
        chosen_control_law_parameters_name = self.load_control_parameters_combobox.get()  # the name of the chosen loaded file that contains the control law parameters
        if chosen_control_law_parameters_name in self.saved_control_law_parameters_list:  # if the chosen file exists
            if event != None:  # if the function is called by an event
                load_file_accept = ms.askyesno("Confirm loading", f"Are you sure you want to load the control law parameters from the file \"{chosen_control_law_parameters_name}.txt\"?")  # ask the user if they want to load the chosen file
            else:
                load_file_accept = True  # the file is loaded automatically
            if load_file_accept:  # if the user wants to load the chosen file
                self.chosen_control_law_parameters_name = chosen_control_law_parameters_name  # change the control law parameters name
                parameters_file = open(self.saved_control_law_parameters_folder_path + fr"/{self.chosen_control_law_parameters_name}.txt", "r", encoding = "utf-8")  # open the chosen file in read mode
                parameters_file_lines = parameters_file.readlines()  # read all the lines of the file
                parameters_file.close()  # close the file
                self.k_d = float(parameters_file_lines[0].split(": ")[1])  # the k_d parameter of the obstacles avoidance solver
                self.k_i = np.array([float(k) for k in parameters_file_lines[1].split(": ")[1][1:-2].split(", ")])  # the k_i parameters of the obstacles avoidance solver
                self.w_phi = float(parameters_file_lines[2].split(": ")[1])  # the w_phi parameter of the obstacles avoidance solver
                self.dp_min = float(parameters_file_lines[3].split(": ")[1])  # the dp_min parameter of the obstacles avoidance solver
                self.vp_max = float(parameters_file_lines[4].split(": ")[1])  # the vp_max parameter of the obstacles avoidance solver
        else:  # if the chosen file does not exist
            ms.showerror("Error", f"The chosen control law parameters \"{chosen_control_law_parameters_name}.txt\" do not exist!", parent = self.menus_area)  # show an error message               
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def plot_navigation_potential_field_values(self, event = None):  # plot the navigation function of the obstacles avoidance solver
        if self.hntf2d_solver != None and self.transformations_are_built:  # if the workspace transformations are built
            points_max_norm = max([np.linalg.norm(self.hntf2d_solver.q_i[k]) for k in range(len(self.hntf2d_solver.q_i))] + [np.linalg.norm(self.hntf2d_solver.q_init), np.linalg.norm(self.hntf2d_solver.q_d)])  # calculate the maximum norm of the points
            grid_points_size = self.navigation_field_plot_points_divs  # the number of points in the grid
            grid_scale_factor = 1.5  # the scale factor of the grid
            x = np.linspace(-grid_scale_factor * points_max_norm, grid_scale_factor * points_max_norm, grid_points_size)  # the x values
            y = np.linspace(-grid_scale_factor * points_max_norm, grid_scale_factor * points_max_norm, grid_points_size)  # the y values
            X, Y = np.meshgrid(x, y)  # the meshgrid of the x and y values
            P = np.array([X.flatten(), Y.flatten()]).T  # the points to evaluate the navigation function
            Z = np.zeros_like(X.flatten())  # the values of the navigation function
            for k in range(len(P)):  # for each point p
                Z[k] = self.hntf2d_solver.navigation_psi(P[k])  # the values of the navigation function
            # plot the navigation function
            general_fontsize = 7; text_margin = 0.02
            fig = plt.figure()
            ax = fig.add_subplot(111, projection = "3d")
            values_map = mcolors.Normalize(vmin = np.min(Z), vmax = np.max(Z))
            color_map = plt.get_cmap("viridis")
            ax.plot_surface(X, Y, Z.reshape(X.shape), cmap = color_map, edgecolor = "none")
            cbar = plt.colorbar(mappable = plt.cm.ScalarMappable(norm = values_map, cmap = color_map), ax = ax)
            cbar.set_label("True values of the navigation field", fontsize = general_fontsize+2)
            q_init_plot, = ax.plot([self.hntf2d_solver.q_init[0]], [self.hntf2d_solver.q_init[1]], [self.hntf2d_solver.navigation_psi(self.hntf2d_solver.q_init)], "ms", markersize = 5)
            q_d_plot, = ax.plot([self.hntf2d_solver.q_d[0]], [self.hntf2d_solver.q_d[1]], [self.hntf2d_solver.navigation_psi(self.hntf2d_solver.q_d)], "gs", markersize = 5)
            for i in range(len(self.hntf2d_solver.q_i)):
                q_i_plot, = ax.plot([self.hntf2d_solver.q_i[i][0]], [self.hntf2d_solver.q_i[i][1]], [self.hntf2d_solver.navigation_psi(self.hntf2d_solver.q_i[i])], "bo", markersize = 5)
                ax.text(self.hntf2d_solver.q_i[i][0] + text_margin, self.hntf2d_solver.q_i[i][1] + text_margin, self.hntf2d_solver.navigation_psi(self.hntf2d_solver.q_i[i]) + text_margin, f"{i+1}", fontsize = general_fontsize)
            if len(self.R2_plane_path_control_law_output) != 0:  # if a path (on the transformed R2 plane) has been calculated by the control law
                path_navigation_field_values = np.zeros(len(self.R2_plane_path_control_law_output))  # the values of the navigation function on the control law transformed path points
                for k in range(len(self.R2_plane_path_control_law_output)):  # loop over the control law transformed path points
                    path_navigation_field_values[k] = self.hntf2d_solver.navigation_psi(self.R2_plane_path_control_law_output[k])  # the values of the navigation function on the control law transformed path points
                R2planews_path_plot, = ax.plot(np.array(self.R2_plane_path_control_law_output)[:, 0], np.array(self.R2_plane_path_control_law_output)[:, 1], path_navigation_field_values, "black", linewidth = 2)
                ax.legend([q_init_plot, q_d_plot, q_i_plot, R2planews_path_plot], ["Start", "Target", "Obstacles", "Path found"], fontsize = general_fontsize, loc = "upper right")
            else:  # if no path (on the transformed R2 plane) has been calculated yet by the control law
                ax.legend([q_init_plot, q_d_plot, q_i_plot], ["Start", "Target", "Obstacles"], fontsize = general_fontsize, loc = "upper right")
            ax.set_title(f"Navigation function psi\nk_d = {self.k_d:.2f}, k_i = {self.k_i}, w_phi = {self.w_phi:.2f}")
            ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("psi(x, y)")
            plt.show()
        else:  # if the workspace transformations are not built
            ms.showerror("Error", "The workspace transformations have not been built yet!", parent = self.menus_area)  # show an error message
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def plot_navigation_potential_field_gradients(self, event = None):  # plot the gradient of the navigation function of the obstacles avoidance solver
        if self.hntf2d_solver != None and self.transformations_are_built:  # if the workspace transformations are built
            points_max_norm = max([np.linalg.norm(self.hntf2d_solver.q_i[k]) for k in range(len(self.hntf2d_solver.q_i))] + [np.linalg.norm(self.hntf2d_solver.q_init), np.linalg.norm(self.hntf2d_solver.q_d)])  # calculate the maximum norm of the points
            grid_points_size = self.navigation_field_plot_points_divs  # the number of points in the grid
            grid_scale_factor = 1.5  # the scale factor of the grid
            x = np.linspace(-grid_scale_factor * points_max_norm, grid_scale_factor * points_max_norm, grid_points_size)  # the x values
            y = np.linspace(-grid_scale_factor * points_max_norm, grid_scale_factor * points_max_norm, grid_points_size)  # the y values
            X, Y = np.meshgrid(x, y)  # the meshgrid of the x and y values
            P = np.array([X.flatten(), Y.flatten()]).T  # the points to evaluate the navigation function
            gradients = np.zeros((len(P), 2))  # the gradients of the navigation function
            gradients_scaled = np.zeros((len(X.flatten()), 2))  # the scaled gradients of the navigation function
            gradients_desired_norm = 1.2 * (2.0 * grid_scale_factor * points_max_norm) / grid_points_size  # the desired norm of the plotted gradients
            reject_point = False  # the flag to reject the point
            for k in range(len(P)):  # for each point p
                for i in range(len(self.hntf2d_solver.q_i)):
                    if np.linalg.norm(P[k] - self.hntf2d_solver.q_i[i]) < 1.0 * (2.0 * grid_scale_factor * points_max_norm) / 100.0:  # if the point p is on an obstacle
                        reject_point = True  # set the flag to reject the point
                        break
                if not reject_point:  # if the point p is not near an obstacle
                    gradients[k] = self.hntf2d_solver.navigation_psi_gradient(P[k])  # the gradient of the navigation function at point p
                    gradient_norm = np.sqrt(gradients[k, 0]**2 + gradients[k, 1]**2)  # the norm of the current gradient
                    gradients_scaled[k] = (gradients[k] / (gradient_norm + 1e-10)) * gradients_desired_norm  # the normalized and rescaled gradient of the navigation function at point p
                reject_point = False  # reset the flag to reject the point
            # plot the gradients
            general_fontsize = 7; text_margin = 0.02
            fig = plt.figure()
            ax = fig.add_subplot(111)
            norms_cap_value = 100  # the cap value of the norms of the gradients
            gradients_true_norms = np.linalg.norm(gradients, axis = 1)  # the true norms of the gradients
            gradients_capped_norms = np.minimum(gradients_true_norms, norms_cap_value)  # the capped norms of the gradients
            norms_map = mcolors.Normalize(vmin = 0.0, vmax = np.max(gradients_capped_norms))
            color_map = plt.get_cmap("turbo")
            ax.quiver(X, Y, -gradients_scaled[:, 0], -gradients_scaled[:, 1], color = color_map(norms_map(gradients_capped_norms)), angles = "xy", scale_units = "xy", scale = 1)
            cbar = plt.colorbar(mappable = plt.cm.ScalarMappable(norm = norms_map, cmap = color_map), ax = ax)
            cbar.set_label("True values of the gradients norms", fontsize = general_fontsize+2)
            q_init_plot, = ax.plot([self.hntf2d_solver.q_init[0]], [self.hntf2d_solver.q_init[1]], "ms", markersize = 5)
            q_d_plot, = ax.plot([self.hntf2d_solver.q_d[0]], [self.hntf2d_solver.q_d[1]], "gs", markersize = 5)
            for i in range(len(self.hntf2d_solver.q_i)):
                q_i_plot, = ax.plot([self.hntf2d_solver.q_i[i][0]], [self.hntf2d_solver.q_i[i][1]], "bo", markersize = 5)
                ax.text(self.hntf2d_solver.q_i[i][0] + text_margin, self.hntf2d_solver.q_i[i][1] + text_margin, f"{i+1}", fontsize = general_fontsize)
            if len(self.R2_plane_path_control_law_output) != 0:  # if a path (on the transformed R2 plane) has been calculated by the control law
                R2planews_path_plot, = ax.plot(np.array(self.R2_plane_path_control_law_output)[:, 0], np.array(self.R2_plane_path_control_law_output)[:, 1], "black", linewidth = 2)
                ax.legend([q_init_plot, q_d_plot, q_i_plot, R2planews_path_plot], ["Start", "Target", "Obstacles", "Path found"], fontsize = general_fontsize, loc = "upper right")
            else:  # if no path (on the transformed R2 plane) has been calculated yet by the control law
                ax.legend([q_init_plot, q_d_plot, q_i_plot], ["Start", "Target", "Obstacles"], fontsize = general_fontsize, loc = "upper right")
            ax.set_title(f"Negative gradients of the navigation function\nk_d = {self.k_d:.2f}, k_i = {self.k_i}, w_phi = {self.w_phi:.2f}\n(press mouse right click on the plot to view the gradients)")
            ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_aspect("equal")
            fig.canvas.mpl_connect("button_press_event", lambda event: self.mark_navigation_field_gradients(event, ax))  # connect a function to the plot
            plt.show()
        else:  # if the workspace transformations are not built
            ms.showerror("Error", "The workspace transformations have not been built yet!", parent = self.menus_area)  # show an error message
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def mark_navigation_field_gradients(self, event, fig_ax):  # mark the navigation field gradients on the plot
        if event.inaxes:  # if the event is inside the axis space
            x, y = event.xdata, event.ydata  # get the coordinates of the mouse click
            if event.button == 3:  # if the user presses mouse right click
                try:
                    self.gradient_plot.remove()  # try to remove the plotted gradient
                    self.gradient_plot_text.remove()  # try to remove the plotted gradient text
                except: pass
                point = np.array([x, y])  # the point
                self.gradient_plot, = fig_ax.plot(x, y, "ko", markersize = 5)  # plot the gradient
                self.gradient_plot_text = fig_ax.text(x, y, f"({x:.3f}, {y:.3f}):\n-grad: {np.round(-self.hntf2d_solver.navigation_psi_gradient(point), 3)}\ngrad norm: {np.round(np.linalg.norm(self.hntf2d_solver.navigation_psi_gradient(point)), 3)}", fontsize = 10, color = "k")  # write the text of the gradient
            plt.draw()  # redraw the plot
    def change_field_plot_points_divs(self, event = None):  # change the number of points divisions in the navigation field plot
        self.navigation_field_plot_points_divs = self.alternate_matrix_elements(self.navigation_field_plot_points_divs_list, self.navigation_field_plot_points_divs)  # change the number of points divisions in the navigation field plot
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def apply_control_law_obstacles_avoidance(self, event = None):  # apply the control law for the obstacles avoidance
        if not self.robot_control_thread_flag:  # if the robot control thread is not running
            if self.hntf2d_solver != None and self.transformations_are_built and self.positions_on_plane_are_correct:  # if the workspace transformations are built
                p_list, p_dot_list, q_disk_list, q_list, control_law_success = self.hntf2d_solver.run_control_law(self.solver_time_step_dt, self.solver_maximum_iterations, self.solver_error_tolerance)  # apply the control law and compute the path positions and velocities on the real and transformed workspaces
                print("The control process has been completed!")  # print a message
                self.realws_path_control_law_output = copy.deepcopy(p_list)  # the path positions on the real workspace, generated by the control law
                self.realws_velocities_control_law_output = copy.deepcopy(p_dot_list)  # the velocities on the real workspace, generated by the control law
                self.unit_disk_path_control_law_output = copy.deepcopy(q_disk_list)  # the path positions on the unit disk, generated by the control law
                self.R2_plane_path_control_law_output = copy.deepcopy(q_list)  # the path positions on the R2 plane, generated by the control law
                self.robot_joints_control_law_output = []  # reset the robot joints control law output
                if len(self.realws_paths_list) < self.max_paths_stored:  # if the total number of paths is less than the maximum allowed number that can be stored
                    self.realws_paths_list.append(copy.deepcopy(self.realws_path_control_law_output) + [copy.deepcopy(self.hntf2d_solver.p_d)])  # append the real workspace path to the list of the real workspace paths
                    self.realws_velocities_sequences_list.append(copy.deepcopy(self.realws_velocities_control_law_output) + [copy.deepcopy((self.hntf2d_solver.p_d - p_list[-1]) / self.solver_time_step_dt)])  # append the real workspace velocities to the list of the real workspace velocities
                    self.unit_disk_paths_list.append(copy.deepcopy(self.unit_disk_path_control_law_output) + [copy.deepcopy(self.hntf2d_solver.q_d_disk)])  # append the unit disk path to the list of the unit disk paths
                    self.R2_plane_paths_list.append(copy.deepcopy(self.R2_plane_path_control_law_output) + [copy.deepcopy(self.hntf2d_solver.q_d)])  # append the R2 plane path to the list of the R2 plane paths
                    self.paths_parameters_list.append([self.k_d, copy.deepcopy(self.k_i), self.w_phi, self.dp_min, self.vp_max])  # append the control law parameters to the list of the control law parameters
                self.path_chosen = len(self.realws_paths_list) - 1  # choose the last path in the list of the real workspace paths
                self.choose_result_from_control_law_outputs(self.path_chosen)  # choose the last path in the list of the control law outputs
                if control_law_success:  # if the control law is successful
                    ms.showinfo("Control law success", f"The control law has been applied successfully!\n" + self.show_chosen_path_information(self.path_chosen), parent = self.menus_area)  # show an info message
                else:  # if the control law is not successful
                    ms.showinfo("Control law failure", f"The control law has failed to converge within the maximum number of iterations ({self.solver_maximum_iterations})!\n• Convergence error = {100.0 * (np.linalg.norm(p_list[-1] - self.hntf2d_solver.p_d)):.2f} cm", parent = self.menus_area)  # show an info message
            else:
                ms.showerror("Error", "The workspace transformations have not been built yet and/or the start and target positions are not set correctly!", parent = self.menus_area)  # show an error message
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def compute_robot_trajectory_obstacles_avoidance(self, event = None):  # compute the robot trajectory for the obstacles avoidance
        if not self.robot_control_thread_flag:  # if the robot control thread is not running
            if self.robotic_manipulator_is_built:  # if a robotic manipulator is built
                if len(self.realws_velocities_control_law_output) != 0:  # if the control law has been applied
                    self.robot_joints_control_law_output = []  # initialize the robot joints control law output
                    self.robot_joints_control_law_output.append(np.array(self.control_joints_variables))  # append the current control joints variables to the list of the robot joints control law output
                    self.control_or_kinematics_variables_visualization = self.control_or_kinematics_variables_visualization_list[0]  # choose the control joints variables for visualization
                    start_pos_workspace_plane = self.realws_path_control_law_output[0]  # the start position of the current path on the workspace plane
                    ve_z = 0.05  # the linear velocity (in m/sec) of the end-effector along its z-axis
                    reset_robot_steps = 100  # the number of steps to reset the robot's position
                    initial_robot_steps = int(2.0 * self.obstacles_height_for_solver / ve_z / self.move_robot_time_step_dt)  # the number of steps to move the robot away from its initial configuration
                    start_1_robot_steps = 100
                    # the number of steps to move the robot to its start configuration above the workspace plane
                    start_2_robot_steps = int(self.obstacles_height_for_solver / ve_z / self.move_robot_time_step_dt)  # the number of steps to move the robot from its start configuration above the workspace plane to the start configuration on the workspace plane
                    # compute the start configurations of the end-effector above and on the workspace plane, based on the obstacles height, and calculate some of the robot's reachable poses
                    obstacles_2d_plane_normal_vector, obstacles_2d_plane_orientation, obstacles_2d_plane_translation = gf.get_components_from_xy_plane_transformation(self.obst_plane_wrt_world_transformation_matrix)  # get the components of the obstacles 2D plane transformation
                    R_plane = self.obst_plane_wrt_world_transformation_matrix[0:3, 0:3]  # the rotation matrix of the workspace plane wrt the world frame
                    obst_top_plane_position = obstacles_2d_plane_translation + obstacles_2d_plane_normal_vector * self.obstacles_height_for_solver  # the position of the top plane of the obstacles
                    obst_top_plane_wrt_world_transformation_matrix = gf.xy_plane_transformation(obstacles_2d_plane_normal_vector, obstacles_2d_plane_orientation, obst_top_plane_position)  # the transformation matrix of the top plane of the obstacles wrt the world frame
                    end_effector_start_position_1 = obst_top_plane_wrt_world_transformation_matrix @ np.array([start_pos_workspace_plane[0], start_pos_workspace_plane[1], 0.0, 1.0])  # the start position of the end-effector on the plane tangent to the obstacles
                    end_effector_start_configuration_1 = gf.xy_plane_transformation(-obstacles_2d_plane_normal_vector, obstacles_2d_plane_orientation, obst_top_plane_position)  # initialize the transformation matrix of the end-effector on the plane tangent to the obstacles
                    end_effector_start_configuration_1[:, 3] = end_effector_start_position_1  # the transformation matrix of the end-effector on the plane tangent to the obstacles
                    end_effector_start_position_2 = self.obst_plane_wrt_world_transformation_matrix @ np.array([start_pos_workspace_plane[0], start_pos_workspace_plane[1], 0.0, 1.0])  # the start position of the end-effector on the workspace plane
                    end_effector_start_configuration_2 = gf.xy_plane_transformation(-obstacles_2d_plane_normal_vector, obstacles_2d_plane_orientation, obstacles_2d_plane_translation)  # initialize the transformation matrix of the end-effector on the workspace plane
                    end_effector_start_configuration_2[:, 3] = end_effector_start_position_2  # the transformation matrix of the end-effector on the workspace plane
                    robot_joints_configurations, robot_reachable_positions = kin.find_reachable_workspace(self.built_robotic_manipulator, self.joints_range_divisions, False)  # find some of the robot's reachable configurations and the corresponding reachable positions
                    robot_reachable_positions = [np.concatenate((robot_reachable_positions[k], np.array([1.0])), axis = 0) for k in range(len(robot_reachable_positions))]  # add the homogeneous coordinate to the reachable positions
                    robot_joints_configurations = [robot_joints_configurations[k] for k in range(len(robot_joints_configurations)) if (np.linalg.inv(obst_top_plane_wrt_world_transformation_matrix) @ robot_reachable_positions[k])[2] >= 1e-3]  # filter the robot's reachable configurations keeping only the ones above the obstacles plane
                    # move the robot from its initial configuration to the zero configuration (reset the robot's pose to its default zero state)
                    go_to_zero_success = True  # the flag to check if the robot can go to its zero configuration without hitting the obstacles plane
                    qr_initial = self.robot_joints_control_law_output[-1]  # the initial configuration of the robot's joints
                    qr_zero = np.zeros_like(self.robot_joints_control_law_output[-1])  # the zero configuration of the robot's joints
                    robot_joints_configurations.sort(key = lambda x: np.linalg.norm(x - self.robot_joints_control_law_output[-1]))  # sort (in ascending order) the robot's reachable configurations based on their distance from the current robot's joints configuration
                    if np.linalg.norm(qr_initial - qr_zero) >= 1e-3:  # if the robot is not already in its zero configuration
                        # move the robot away from its initial configuration (usually the robot's end-effector is on the workspace plane)
                        ve = np.concatenate((np.array([0.0, 0.0, -ve_z]), np.zeros(3)), axis = 0)  # set the velocity of the end-effector wrt its own frame
                        qr_old = self.robot_joints_control_law_output[-1]  # the previous configuration of the robot's joints
                        for j in range(initial_robot_steps):  # loop through the number of steps to move the robot away from its initial configuration
                            qr_dot, joints_velocity_found = kin.compute_inverse_differential_kinematics(self.built_robotic_manipulator, self.robot_joints_control_law_output[-1], ve, "end-effector")  # compute the joints velocities
                            if joints_velocity_found:  # if valid joints velocities are found
                                qr_new = qr_old + qr_dot * self.move_robot_time_step_dt  # the new configuration of the robot's joints
                                self.robot_joints_control_law_output.append(qr_new)  # append the new configuration of the robot's joints to the list of the robot joints control law output
                                pos_new = np.array(self.built_robotic_manipulator.fkine(qr_new), dtype = float)[:, 3]  # the new position (in homogeneous coordinates) of the end-effector wrt the world frame
                                collision_happening = (np.linalg.inv(self.obst_plane_wrt_world_transformation_matrix) @ pos_new)[2] <= -1e-3  # check if the end-effector is below the obstacles plane
                                if collision_happening:  # if the end-effector is below the obstacles plane, meaning there is a collision
                                    break  # break the loop
                                qr_old = qr_new  # the configuration of the robot's joints for the next step
                            else:  # if valid joints velocities are not found
                                break  # break the loop
                        # move the robot to its zero configuration
                        current_robot_joints_number = len(self.robot_joints_control_law_output)  # the current number of the robot's joints configurations
                        initial_robot_configuration_joints_sequence = copy.deepcopy(self.robot_joints_control_law_output)  # the sequence of the robot's joints configurations to move the robot away from its initial configuration
                        qr_initial = self.robot_joints_control_law_output[-1]  # the initial configuration of the robot's joints
                        for qr_middle in robot_joints_configurations:  # loop through the robot's reachable joints configurations
                            go_to_zero_success = True  # reset the flag to check if the robot can go to its zero configuration without hitting the obstacles plane
                            for i in range(int(reset_robot_steps / 2.0)):  # loop through the number of steps to reset the robot's position
                                qr_new = qr_initial + (i + 1) * (qr_middle - qr_initial) / int(reset_robot_steps / 2.0)  # the new configuration of the robot's joints
                                self.robot_joints_control_law_output.append(qr_new)  # append the new configuration of the robot's joints to the list of the robot joints control law output
                            for j in range(int(reset_robot_steps / 2.0)):  # loop through the number of steps to reset the robot's position
                                qr_new = qr_middle + (j + 1) * (qr_zero - qr_middle) / int(reset_robot_steps / 2.0)  # the new configuration of the robot's joints
                                self.robot_joints_control_law_output.append(qr_new)  # append the new configuration of the robot's joints to the list of the robot joints control law output
                            for k in range(len(self.robot_joints_control_law_output[current_robot_joints_number:])):  # loop through the number of steps to check if the robot hits the obstacles plane
                                qr = self.robot_joints_control_law_output[current_robot_joints_number + k]  # the current configuration of the robot's joints
                                pos = np.array(self.built_robotic_manipulator.fkine(qr), dtype = float)[:, 3]  # the current position (in homogeneous coordinates) of the end-effector wrt the world frame
                                collision_happening = (np.linalg.inv(obst_top_plane_wrt_world_transformation_matrix) @ pos)[2] <= -1e-3  # check if the end-effector is below the obstacles plane
                                if collision_happening:  # if the end-effector is below the obstacles plane, meaning there is a collision
                                    go_to_zero_success = False  # the robot cannot go to its zero configuration without hitting the obstacles plane
                                    self.robot_joints_control_law_output = copy.deepcopy(initial_robot_configuration_joints_sequence)  # reset the robot's joints to their initial configuration
                                    break  # break the loop
                            if go_to_zero_success:  # if the robot does not hit the obstacles plane
                                break  # break the loop
                    # move the robot from its zero configuration to its start configuration above the workspace plane
                    if go_to_zero_success:  # if the robot can go to its zero configuration without hitting the obstacles plane
                        # check if the start configurations of the end-effector on the workspace plane can be achieved by the robot
                        invkine_success = False  # the flag to check if the inverse kinematics has succeeded or not
                        invkine_tolerance = 1e-10  # the tolerance of the inverse kinematics
                        while not invkine_success and invkine_tolerance <= 1e-5:  # while the inverse kinematics has not succeeded
                            _, invkine_success_1 = kin.compute_inverse_kinematics(self.built_robotic_manipulator, end_effector_start_configuration_1, invkine_tolerance)  # compute the robot's joints configuration (if possible)
                            _, invkine_success_2 = kin.compute_inverse_kinematics(self.built_robotic_manipulator, end_effector_start_configuration_2, invkine_tolerance)  # compute the robot's joints configuration (if possible)
                            invkine_success = invkine_success_1 and invkine_success_2  # check if the inverse kinematics has succeeded
                            if not invkine_success:  # if the inverse kinematics has not succeeded
                                invkine_tolerance *= 10.0  # increase the tolerance of the inverse kinematics
                        if invkine_success:  # if the inverse kinematics has succeeded
                            # choose a start joints configuration for the robot, based on the distance of the joints values from their limits
                            invkine_max_tries = 100  # the maximum number of tries for the inverse kinematics
                            invkine_tries_start_configuration = []  # the tries for the inverse kinematics of the end-effector's start configuration on the workspace plane
                            for k in range(invkine_max_tries):  # loop through the maximum number of tries for the inverse kinematics
                                qr_joints_start_configuration, _ = kin.compute_inverse_kinematics(self.built_robotic_manipulator, end_effector_start_configuration_1, invkine_tolerance)  # compute the robot's joints configuration (if possible) for the start end-effector configuration
                                qr_joints_are_valid, joints_limits_distance = kin.check_joints_limits(self.built_robotic_manipulator, qr_joints_start_configuration)  # check the joints limits
                                if qr_joints_are_valid:  # if the joints values are within their limits
                                    invkine_tries_start_configuration.append([qr_joints_start_configuration, joints_limits_distance])  # append the tries for the inverse kinematics of the end-effector's start configuration on the workspace plane
                            invkine_tries_start_configuration.sort(key = lambda x: x[1], reverse = True)  # sort the tries of the inverse kinematics based on the distance of the joints values from their limits, in a descending order
                            # loop through all the inverse kinematics tries for the start configuration of the end-effector on the workspace plane, until a feasible trajectory is found for the robot to move to the target position
                            current_robot_joints_number = len(self.robot_joints_control_law_output)  # the current number of the robot's joints configurations in the control law output
                            reset_robot_configuration_joints_sequence = copy.deepcopy(self.robot_joints_control_law_output)  # the time sequence of the robot's joints to reset the robot's position
                            for k in range(len(invkine_tries_start_configuration)):  # loop through all the inverse kinematics tries
                                # move the robot to the start configuration above the workspace plane
                                go_to_start_1_success = True  # the flag to check if the robot can go to its start configuration above the workspace plane without hitting the obstacles plane
                                qr_start_1 = invkine_tries_start_configuration[k][0]  # the start configuration of the robot's joints above the workspace plane
                                qr_zero = np.zeros_like(qr_start_1)  # the zero configuration of the robot's joints
                                robot_joints_configurations.sort(key = lambda x: np.linalg.norm(x - self.robot_joints_control_law_output[-1]))  # sort (in ascending order) the robot's reachable configurations based on their distance from the current robot's joints configuration
                                for qr_middle in robot_joints_configurations:  # loop through the robot's reachable joints configurations
                                    go_to_start_1_success = True  # reset the flag to check if the robot can go to its start configuration above the workspace plane without hitting the obstacles plane
                                    for i in range(int(start_1_robot_steps / 2.0)):  # loop through the number of steps to navigate the robot to the start configuration above the workspace plane
                                        qr_new = qr_zero + (i + 1) * (qr_middle - qr_zero) / int(reset_robot_steps / 2.0)  # the new configuration of the robot's joints
                                        self.robot_joints_control_law_output.append(qr_new)  # append the new configuration of the robot's joints to the list of the robot joints control law output
                                    for j in range(int(start_1_robot_steps / 2.0)):  # loop through the number of steps to navigate the robot to the start configuration above the workspace plane
                                        qr_new = qr_middle + (j + 1) * (qr_start_1 - qr_middle) / int(reset_robot_steps / 2.0)  # the new configuration of the robot's joints
                                        self.robot_joints_control_law_output.append(qr_new)  # append the new configuration of the robot's joints to the list of the robot joints control law output
                                    for k in range(len(self.robot_joints_control_law_output[current_robot_joints_number:])):  # loop through the number of steps to check if the robot hits the obstacles plane
                                        qr = self.robot_joints_control_law_output[current_robot_joints_number + k]  # the current configuration of the robot's joints
                                        pos = np.array(self.built_robotic_manipulator.fkine(qr), dtype = float)[:, 3]  # the current position (in homogeneous coordinates) of the end-effector wrt the world frame
                                        collision_happening = (np.linalg.inv(obst_top_plane_wrt_world_transformation_matrix) @ pos)[2] <= -1e-3  # check if the end-effector is below the top plane of the obstacles
                                        if collision_happening:  # if the end-effector is below the top plane of the obstacles, meaning there is a collision
                                            go_to_start_1_success = False  # the robot cannot go to its start configuration above the workspace plane without hitting the obstacles plane
                                            self.robot_joints_control_law_output = copy.deepcopy(reset_robot_configuration_joints_sequence)  # reset the robot's joints to their initial configuration
                                            break  # break the loop
                                    if go_to_start_1_success:  # if the robot does not hit the obstacles plane
                                        break  # break the loop
                                if go_to_start_1_success:  # if the robot can go to its start configuration above the workspace plane without hitting the obstacles plane
                                    # move the robot from the start configuration above the workspace plane to the current start configuration on the workspace plane
                                    ve = np.concatenate((np.array([0.0, 0.0, ve_z]), np.zeros(3)), axis = 0)  # set the velocity of the end-effector wrt its own frame
                                    qr_old = self.robot_joints_control_law_output[-1]  # the previous configuration of the robot's joints
                                    for j in range(start_2_robot_steps):  # loop through the number of steps to move the robot from its start configuration above the workspace plane to the start configuration on the workspace plane
                                        qr_dot, _ = kin.compute_inverse_differential_kinematics(self.built_robotic_manipulator, self.robot_joints_control_law_output[-1], ve, "end-effector")  # compute the joints velocities
                                        qr_new = qr_old + qr_dot * self.move_robot_time_step_dt  # the new configuration of the robot's joints
                                        self.robot_joints_control_law_output.append(qr_new)  # append the new configuration of the robot's joints to the list of the robot joints control law output
                                        qr_old = qr_new  # the configuration of the robot's joints for the next step
                                    # move the robot from the start position to the target position on the workspace plane
                                    trajectory_success = True  # the flag to check if the trajectory has been computed successfully
                                    for k in range(len(self.realws_velocities_control_law_output)):  # loop through all the velocities of the real workspace path, generated by the control law
                                        p_dot = self.realws_velocities_control_law_output[k]  # the velocity of the real workspace path
                                        qr_old = self.robot_joints_control_law_output[-1]  # the previous configuration of the robot's joints
                                        pe_dot = R_plane @ np.array([p_dot[0], p_dot[1], 0.0])  # the velocity of the end-effector on the workspace plane wrt the world frame
                                        ve = np.concatenate((pe_dot, np.zeros(3)), axis = 0)  # the velocity of the end-effector wrt the world frame
                                        qr_dot, valid_qr_dot_found = kin.compute_inverse_differential_kinematics(self.built_robotic_manipulator, self.robot_joints_control_law_output[-1], ve, "world")  # compute the joints velocities
                                        qr_new = qr_old + qr_dot * self.solver_time_step_dt  # the new configuration of the robot's joints
                                        qr_joints_are_valid, _ = kin.check_joints_limits(self.built_robotic_manipulator, qr_new)  # check the joints limits
                                        if valid_qr_dot_found and qr_joints_are_valid:  # if valid joints velocities are found and the joints values are within their limits for the new configuration
                                            new_movement_divs_number = int(self.solver_time_step_dt / self.move_robot_time_step_dt)  # the number of movement divisions to reach the new configuration
                                            for i in range(new_movement_divs_number):  # divide the current movement to smaller steps
                                                qr_new_div = qr_old + (i + 1) * (qr_dot * self.solver_time_step_dt) / new_movement_divs_number  # the new configuration of the robot's joints
                                                self.robot_joints_control_law_output.append(qr_new_div)  # append the new configuration of the robot's joints to the list of the robot joints control law output
                                        else:  # if the Jacobian matrix is not full rank or the joints values are not within their limits
                                            trajectory_success = False  # set the flag to False
                                            break  # break the loop
                                    if trajectory_success:  # if a feasible trajectory is found for the robot to move to the target position
                                        break  # break the loop
                                    else:  # if the current trajectory is not feasible
                                        self.robot_joints_control_law_output = copy.deepcopy(reset_robot_configuration_joints_sequence)  # initialize the robot's joints to the reset time sequence
                                else:  # if the robot cannot go to its start configuration above the workspace plane without hitting the obstacles plane
                                    trajectory_success = False  # cannot find a feasible trajectory for the robot to move to the target position
                                    self.robot_joints_control_law_output = copy.deepcopy(reset_robot_configuration_joints_sequence)  # initialize the robot's joints to the reset time sequence
                            if trajectory_success:  # if a feasible trajectory is found for the robot to move to the target position
                                ms.showinfo("Robot trajectory success", "The robot's trajectory has been calculated successfully!", parent = self.menus_area)  # show an info message
                                self.plot_robot_joints_trajectories()  # plot the robot's joints trajectories for the obstacles avoidance
                            else:  # if no feasible trajectory is found for the robot to move to the target position
                                ms.showinfo("Robot trajectory failure", "Failed to find a feasible trajectory!", parent = self.menus_area)  # show an info message
                        else:  # if the inverse kinematics has not succeded
                            ms.showinfo("Robot trajectory failure", "The inverse kinematics has failed to find a feasible solution for the start configuration of the end-effector on the workspace plane!", parent = self.menus_area)  # show an info message
                    else: # if the robot cannot go to its zero configuration without hitting the obstacles plane
                        ms.showinfo("Robot trajectory failure", "The robot cannot go to its zero configuration without hitting the obstacles plane!", parent = self.menus_area)
            else:  # if no robotic manipulator is built yet
                ms.showerror("Error", "Please build a robotic manipulator first!", parent = self.menus_area)  # show an error message
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def plot_robot_joints_trajectories(self, event = None):  # plot the robot's joints trajectories for the obstacles avoidance
        if len(self.robot_joints_control_law_output) != 0:  # if the robot's joints trajectories have been computed
            revolute_joints_indexes = [k for k in range(len(self.joints_types)) if self.joints_types[k] == self.joints_types_list[0]]  # the indexes of the revolute joints
            prismatic_joints_indexes = [k for k in range(len(self.joints_types)) if self.joints_types[k] == self.joints_types_list[1]]  # the indexes of the prismatic joints
            trajectory_linewidth = 2; text_fontsize = 8; legend_fontsize = 7
            fig = plt.figure()
            if len(revolute_joints_indexes) != 0 and len(prismatic_joints_indexes) != 0:  # if there are both revolute and prismatic joints
                ax1 = fig.add_subplot(121); ax2 = fig.add_subplot(122)  # create two subplots
            else:  # if there is only one type of joints
                if len(revolute_joints_indexes) != 0: ax1 = fig.add_subplot(111)  # if there are only revolute joints create a subplot
                if len(prismatic_joints_indexes) != 0: ax2 = fig.add_subplot(111)  # if there are only prismatic joints create a subplot
            if len(revolute_joints_indexes) != 0:  # if there are prismatic joints
                for k in revolute_joints_indexes:  # loop through all the revolute joints
                    joint_trajectory = [np.rad2deg(self.robot_joints_control_law_output[j][k]) for j in range(len(self.robot_joints_control_law_output))]  # the trajectory of the k-th joint
                    ax1.plot(np.arange(0, len(joint_trajectory) * self.solver_time_step_dt, self.solver_time_step_dt), joint_trajectory, linewidth = trajectory_linewidth, label = f"q{k+1}")
                    ax1.text(len(joint_trajectory) * self.solver_time_step_dt, joint_trajectory[-1], f"q{k+1}", fontsize = text_fontsize)
                ax1.legend(fontsize = legend_fontsize, loc = "upper left")
                ax1.set_title("Robot's revolute joints trajectories")
                ax1.set_xlabel("Time (sec)"); ax1.set_ylabel("Joints angles (degrees)")
            if len(prismatic_joints_indexes) != 0:  # if there are prismatic joints
                for k in prismatic_joints_indexes:  # loop through all the prismatic joints
                    joint_trajectory = [self.robot_joints_control_law_output[j][k] for j in range(len(self.robot_joints_control_law_output))]  # the trajectory of the k-th joint
                    ax2.plot(np.arange(0, len(joint_trajectory) * self.solver_time_step_dt, self.solver_time_step_dt), joint_trajectory, linewidth = trajectory_linewidth, label = f"q{k+1}")
                    ax2.text(len(joint_trajectory) * self.solver_time_step_dt, joint_trajectory[-1], f"q{k+1}", fontsize = text_fontsize)
                ax2.legend(fontsize = legend_fontsize, loc = "upper left")
                ax2.set_title("Robot's prismatic joints trajectories")
                ax2.set_xlabel("Time (sec)"); ax2.set_ylabel("Joints displacements (m)")
            plt.show()
    def change_trajectory_speed(self, event = None):  # change the trajectory speed in relation to the obstacles avoidance solver time step
        self.trajectory_relative_speed = self.alternate_matrix_elements(self.trajectory_relative_speeds_list, self.trajectory_relative_speed)  # change the trajectory relative speed
        self.move_robot_time_step_dt = self.trajectory_relative_speed * self.solver_time_step_dt  # the time step of the robot's movement
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def move_simulated_robot_obstacles_avoidance(self, event = None):  # control the simulated robotic manipulator for the obstacles avoidance
        if not self.robot_control_thread_flag:  # if the robot control thread is not running
            if self.robotic_manipulator_is_built:  # if a robotic manipulator is built
                self.simulated_robot_is_moving = True  # set the flag that indicates that the simulated robot is moving
                self.control_or_kinematics_variables_visualization = self.control_or_kinematics_variables_visualization_list[0]  # choose the forward kinematics variables for visualization
                self.start_robot_control_thread()  # start the robot control thread
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def move_swift_robot_obstacles_avoidance(self, event = None):  # control the Swift robotic manipulator for the obstacles avoidance
        if not self.robot_control_thread_flag:  # if the robot control thread is not running
            if self.robotic_manipulator_is_built:  # if a robotic manipulator is built
                self.swift_robot_is_moving = True  # set the flag that indicates that the Swift robot is moving
                self.control_or_kinematics_variables_visualization = self.control_or_kinematics_variables_visualization_list[0]  # choose the forward kinematics variables for visualization
                self.start_robot_control_thread()  # start the robot control thread
    def move_real_robot_obstacles_avoidance(self, event = None):  # control the real robotic manipulator for the obstacles avoidance
        if not self.robot_control_thread_flag:  # if the robot control thread is not running
            if self.robotic_manipulator_is_built: # if a robotic manipulator is built
                self.real_robot_is_moving = True  # set the flag that indicates that the real robot is moving
                self.control_or_kinematics_variables_visualization = self.control_or_kinematics_variables_visualization_list[0]  # choose the control variables for visualization
                self.start_robot_control_thread()  # start the robot control thread
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def update_obstacles_avoidance_solver_indicators(self, event = None):  # update the indicators of the obstacles avoidance solver
        try:
            self.check_start_target_pos_on_plane()  # check the start and target positions of the robot's end-effector on the 2D plane
            self.check_workspace_transformations_built()  # check which workspace transformations have been built
            self.load_workspace_obstacles_objects_combobox.set(self.chosen_solver_workspace_image_name)  # set the value of the load workspace obstacles image combobox
            self.show_obstacles_infos_indicator.configure(text = f"{self.obstacles_infos_text}")  # change the text of the label that shows the obstacles information
            self.define_workspace_plane_button.configure(text = self.workspace_plane_creation_parameter)  # change the text of the button that allows the user to define the workspace plane
            if self.workspace_plane_creation_parameter == self.workspace_plane_creation_parameters_list[0]:  # if the user wants to define the workspace plane using the current workspace image
                self.obst_plane_wrt_world_transformation_matrix = np.array(self.workspace_image_plane_frame_wrt_world, dtype = np.float32)  # the transformation matrix of the workspace plane with respect to the world frame
            elif self.workspace_plane_creation_parameter == self.workspace_plane_creation_parameters_list[1]:  # if the user wants to define the workspace plane using the manual inputs
                self.obst_plane_wrt_world_transformation_matrix = gf.xy_plane_transformation(self.obstacles_2d_plane_normal_vector, self.obstacles_2d_plane_orientation, self.obstacles_2d_plane_translation)  # the transformation matrix of the workspace plane with respect to the world frame
            start_pos_workspace_plane = self.start_pos_workspace_plane + np.array([self.workspace_image_plane_x_length / 2.0, self.workspace_image_plane_y_length / 2.0])  # the start position of the robot's end-effector on the 2D workspace plane
            target_pos_workspace_plane =  self.target_pos_workspace_plane + np.array([self.workspace_image_plane_x_length / 2.0, self.workspace_image_plane_y_length / 2.0])  # the target position of the robot's end-effector on the 2D workspace plane
            self.start_pos_plane_button.configure(text = str([np.round(start_pos_workspace_plane[k], self.distances_precision) for k in range(len(start_pos_workspace_plane))]))  # change the text of the start position on the plane button
            self.target_pos_plane_button.configure(text = str([np.round(target_pos_workspace_plane[k], self.distances_precision) for k in range(len(target_pos_workspace_plane))]))  # change the text of the target position on the plane button
            self.check_start_target_pos_indicator.configure(text = self.check_positions_on_plane_text)  # change the text of the label that checks the start and target positions on the plane
            self.show_transformations_built_indicator.configure(text = self.check_transformations_built_text)  # change the text of the label that shows the built transformations
            self.choose_k_d_parameter_button.configure(text = f"{self.k_d:.2f}")  # change the text of the button that allows the user to choose the k_d parameter
            if len(self.k_i) != len(self.obstacles_boundaries_for_solver) - 1:  # if the number of the k_i parameters is not equal to the number of the inner boundaries
                self.k_i = [0.1 for k in range(len(self.obstacles_boundaries_for_solver) - 1)]  # set the k_i parameters to a default value
            if len(self.obstacles_boundaries_for_solver) != 0:  # if there are obstacles boundaries
                self.choose_k_i_parameters_combobox["values"] = [f"{k + 1}: {self.k_i[k]:.2f}" for k in range(len(self.obstacles_boundaries_for_solver) - 1)]  # update the values of the combobox that allows the user to choose the k_i parameters
                if self.k_i_chosen_num > len(self.obstacles_boundaries_for_solver) - 1:  # if the chosen number of the k_i parameters is greater than the number of the inner boundaries
                    self.k_i_chosen_num = 1  # set the chosen number of the k_i parameters to 1 (the first inner boundary)
            else:  # if there are no obstacles boundaries yet
                self.choose_k_i_parameters_combobox["values"] = []  # update the values of the combobox that allows the user to choose the k_i parameters
                self.k_i_chosen_num = 1  # set the chosen number of the k_i parameters to 1 (the first inner boundary)
            if len(self.k_i) != 0:  # if there are k_i parameters
                self.choose_k_i_parameters_combobox.set(f"{self.k_i[self.k_i_chosen_num - 1]:.2f}")  # set the value of the combobox that allows the user to choose the k_i parameters
            self.choose_w_phi_parameter_button.configure(text = f"{self.w_phi:.2f}")  # change the text of the button that allows the user to choose the w_phi parameter
            self.choose_dp_min_parameter_button.configure(text = f"{100.0 * self.dp_min:.1f}")  # change the text of the button that allows the user to choose the dp_min parameter
            self.choose_vp_max_parameter_button.configure(text = f"{100.0 * self.vp_max:.1f}")  # change the text of the button that allows the user to choose the vp_max parameter
            self.load_control_parameters_combobox["values"] = self.saved_control_law_parameters_list  # update the values of the combobox that allows the user to load the control law parameters
            self.load_control_parameters_combobox.set(self.chosen_control_law_parameters_name)  # set the value of the combobox that allows the user to load the control law parameters
            self.choose_solver_dt_button.configure(text = f"{self.solver_time_step_dt:.3f}")  # change the text of the button that allows the user to choose the time step for the control law
            self.choose_solver_max_iter_button.configure(text = f"{self.solver_maximum_iterations}")  # change the text of the button that allows the user to choose the maximum number of iterations for the control law
            self.choose_solver_error_tol_button.configure(text = f"{100.0 * self.solver_error_tolerance:.1f}")  # change the text of the button that allows the user to choose the error tolerance for the control law
            self.enable_error_correction_button.configure(text = ["no", "yes"][[False, True].index(self.solver_enable_error_correction)])  # change the text of the button that allows the user to enable/disable the error correction for the control law
            max_path_time = self.calculate_maximum_path_time()  # calculate the maximum path time
            self.show_max_path_time_indicator.configure(text = f"{max_path_time:.3f}")  # change the text of the label that shows the maximum path time
            max_path_length = self.calculate_maximum_path_length()  # calculate the maximum path length
            self.show_max_path_length_indicator.configure(text = f"{100.0 * max_path_length:.1f}")  # change the text of the label that shows the maximum path length
            self.navigation_field_plot_points_divs_button.configure(text = f"{self.navigation_field_plot_points_divs}")  # change the text of the button that allows the user to change the number of points divisions in the navigation field plot
            self.trajectory_speed_button.configure(text = f"x {self.trajectory_relative_speed}")  # change the text of the button that allows the user to change the trajectory relative speed
            self.refresh_obstacles_avoidance_solver_parameters()  # refresh the obstacles avoidance solver parameters
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_obstacles_avoidance_solver_indicators: {e}")

    # the functions used for the thread that causes the robotic manipulator controlled movement
    def start_robot_control_thread(self, event = None):  # create and start a robot control thread
        self.robot_control_thread_flag = True  # let the robot control thread run
        self.robot_control_thread = threading.Thread(target = self.robot_control_thread_run_function)  # create the thread for the robot control
        self.robot_control_thread.start()  # start the robot control thread
    def kill_robot_control_thread(self, event = None):  # kill the existed and running robot control thread
        self.robot_control_thread_flag = False  # stop the robot control thread
        self.simulated_robot_is_moving = False  # stop the simulated robot
        self.swift_robot_is_moving = False  # stop the Swift robot
        self.real_robot_is_moving = False  # stop the real robot
        self.update_obstacles_avoidance_solver_indicators()  # update the obstacles avoidance solver indicators
    def robot_control_thread_run_function(self):  # the run function, for the robot control thread
        control_or_kinematics_choose = self.control_or_kinematics_variables_visualization  # keep the choice of control or kinematics variables for visualization
        while True:  # while the thread is running
            for k in range(len(self.robot_joints_control_law_output)):  # loop through the joints variables time series for the robot control
                if not self.real_robot_is_moving:  # if the real robot is not moving
                    if control_or_kinematics_choose == self.control_or_kinematics_variables_visualization_list[0]:  # if the user wants to visualize the control variables
                        self.control_joints_variables = self.robot_joints_control_law_output[k]  # the joints variables for the robot control, for the current time step
                    elif control_or_kinematics_choose == self.control_or_kinematics_variables_visualization_list[1]:  # if the user wants to visualize the forward kinematics variables
                        self.forward_kinematics_variables = self.robot_joints_control_law_output[k]  # the joints variables for the robot control, for the current time step
                    if self.swift_robot_is_moving:  # if the Swift robot is moving
                        pass
                    time.sleep(self.solver_time_step_dt)  # wait for some time equal to the time step of the solver
                else:  # if the real robot is moving
                    if self.serial_connection.is_open:  # if the serial connection is open
                        self.control_joints_variables = self.robot_joints_control_law_output[k]  # the joints variables for the robot control, for the current time step
                        self.control_joints_variables[-1] = 0.0
                        self.control_end_effector_variable = 0.0  # the end-effector variable for the robot control, for the current time step
                        self.send_command_to_all_motors()  # send the command to all the motors
                        time.sleep(0.3)  # wait for some time
                    else:  # if the serial connection is not open yet
                        ms.showerror("Error", "The serial connection is not open yet! You have to establish a serial connection between the computer and the real robot, in order to control the real robot!", parent = self.menus_area)  # show an error message
                        break  # break the for loop
            self.kill_robot_control_thread()  # kill the existed and running robot control thread
            if not self.robot_control_thread_flag:  # if the thread is stopped
                break  # break the while loop
    

# the main function of the program
if __name__ == "__main__":
    # windows_number = int(input("How many windows (program instances) do you want to create? "))
    windows_number = 1
    roots_list = []
    guis_list = []
    for window in range(windows_number):
        roots_list.append(tk.Tk())
        guis_list.append(robotic_manipulators_playground_window(roots_list[window], window))
    for window in range(windows_number):
        roots_list[window].mainloop()