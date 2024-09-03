import numpy as np
from shapely.geometry import Polygon, Point
import cv2, glob
import general_functions as gf


# functions for camera operations

ArUco_dictionaries = {
	"4X4_50/20mm": cv2.aruco.DICT_4X4_50, "4X4_50/30mm": cv2.aruco.DICT_4X4_50, "4X4_50/40mm": cv2.aruco.DICT_4X4_50, "4X4_50/50mm": cv2.aruco.DICT_4X4_50, "4X4_50/100mm": cv2.aruco.DICT_4X4_50,
    "4X4_50/150mm": cv2.aruco.DICT_4X4_50, "4X4_50/200mm": cv2.aruco.DICT_4X4_50, "4X4_50/250mm": cv2.aruco.DICT_4X4_50, "4X4_50/300mm": cv2.aruco.DICT_4X4_50,
	"4X4_100/20mm": cv2.aruco.DICT_4X4_100, "4X4_100/30mm": cv2.aruco.DICT_4X4_100, "4X4_100/40mm": cv2.aruco.DICT_4X4_100, "4X4_100/50mm": cv2.aruco.DICT_4X4_100, "4X4_100/100mm": cv2.aruco.DICT_4X4_100,
    "4X4_100/150mm": cv2.aruco.DICT_4X4_100, "4X4_100/200mm": cv2.aruco.DICT_4X4_100, "4X4_100/250mm": cv2.aruco.DICT_4X4_100, "4X4_100/300mm": cv2.aruco.DICT_4X4_100,
    "4X4_250/20mm": cv2.aruco.DICT_4X4_250, "4X4_250/30mm": cv2.aruco.DICT_4X4_250, "4X4_250/40mm": cv2.aruco.DICT_4X4_250, "4X4_250/50mm": cv2.aruco.DICT_4X4_250, "4X4_250/100mm": cv2.aruco.DICT_4X4_250,
    "4X4_250/150mm": cv2.aruco.DICT_4X4_250, "4X4_250/200mm": cv2.aruco.DICT_4X4_250, "4X4_250/250mm": cv2.aruco.DICT_4X4_250, "4X4_250/300mm": cv2.aruco.DICT_4X4_250,
    "4X4_1000/20mm": cv2.aruco.DICT_4X4_1000, "4X4_1000/30mm": cv2.aruco.DICT_4X4_1000, "4X4_1000/40mm": cv2.aruco.DICT_4X4_1000, "4X4_1000/50mm": cv2.aruco.DICT_4X4_1000, "4X4_1000/100mm": cv2.aruco.DICT_4X4_1000,
    "4X4_1000/150mm": cv2.aruco.DICT_4X4_1000, "4X4_1000/200mm": cv2.aruco.DICT_4X4_1000, "4X4_1000/250mm": cv2.aruco.DICT_4X4_1000, "4X4_1000/300mm": cv2.aruco.DICT_4X4_1000,
	"5X5_50/20mm": cv2.aruco.DICT_5X5_50, "5X5_50/30mm": cv2.aruco.DICT_5X5_50, "5X5_50/40mm": cv2.aruco.DICT_5X5_50, "5X5_50/50mm": cv2.aruco.DICT_5X5_50, "5X5_50/100mm": cv2.aruco.DICT_5X5_50,
    "5X5_50/150mm": cv2.aruco.DICT_5X5_50, "5X5_50/200mm": cv2.aruco.DICT_5X5_50, "5X5_50/250mm": cv2.aruco.DICT_5X5_50, "5X5_50/300mm": cv2.aruco.DICT_5X5_50,
    "5X5_100/20mm": cv2.aruco.DICT_5X5_100, "5X5_100/30mm": cv2.aruco.DICT_5X5_100, "5X5_100/40mm": cv2.aruco.DICT_5X5_100, "5X5_100/50mm": cv2.aruco.DICT_5X5_100, "5X5_100/100mm": cv2.aruco.DICT_5X5_100,
    "5X5_100/150mm": cv2.aruco.DICT_5X5_100, "5X5_100/200mm": cv2.aruco.DICT_5X5_100, "5X5_100/250mm": cv2.aruco.DICT_5X5_100, "5X5_100/300mm": cv2.aruco.DICT_5X5_100,
    "5X5_250/20mm": cv2.aruco.DICT_5X5_250, "5X5_250/30mm": cv2.aruco.DICT_5X5_250, "5X5_250/40mm": cv2.aruco.DICT_5X5_250, "5X5_250/50mm": cv2.aruco.DICT_5X5_250, "5X5_250/100mm": cv2.aruco.DICT_5X5_250,
    "5X5_250/150mm": cv2.aruco.DICT_5X5_250, "5X5_250/200mm": cv2.aruco.DICT_5X5_250, "5X5_250/250mm": cv2.aruco.DICT_5X5_250, "5X5_250/300mm": cv2.aruco.DICT_5X5_250,
    "5X5_1000/20mm": cv2.aruco.DICT_5X5_1000, "5X5_1000/30mm": cv2.aruco.DICT_5X5_1000, "5X5_1000/40mm": cv2.aruco.DICT_5X5_1000, "5X5_1000/50mm": cv2.aruco.DICT_5X5_1000, "5X5_1000/100mm": cv2.aruco.DICT_5X5_1000,
    "5X5_1000/150mm": cv2.aruco.DICT_5X5_1000, "5X5_1000/200mm": cv2.aruco.DICT_5X5_1000, "5X5_1000/250mm": cv2.aruco.DICT_5X5_1000, "5X5_1000/300mm": cv2.aruco.DICT_5X5_1000,
    "6X6_50/20mm": cv2.aruco.DICT_6X6_50, "6X6_50/30mm": cv2.aruco.DICT_6X6_50, "6X6_50/40mm": cv2.aruco.DICT_6X6_50, "6X6_50/50mm": cv2.aruco.DICT_6X6_50, "6X6_50/100mm": cv2.aruco.DICT_6X6_50,
    "6X6_50/150mm": cv2.aruco.DICT_6X6_50, "6X6_50/200mm": cv2.aruco.DICT_6X6_50, "6X6_50/250mm": cv2.aruco.DICT_6X6_50, "6X6_50/300mm": cv2.aruco.DICT_6X6_50,
    "6X6_100/20mm": cv2.aruco.DICT_6X6_100, "6X6_100/30mm": cv2.aruco.DICT_6X6_100, "6X6_100/40mm": cv2.aruco.DICT_6X6_100, "6X6_100/50mm": cv2.aruco.DICT_6X6_100, "6X6_100/100mm": cv2.aruco.DICT_6X6_100,
    "6X6_100/150mm": cv2.aruco.DICT_6X6_100, "6X6_100/200mm": cv2.aruco.DICT_6X6_100, "6X6_100/250mm": cv2.aruco.DICT_6X6_100, "6X6_100/300mm": cv2.aruco.DICT_6X6_100,
    "6X6_250/20mm": cv2.aruco.DICT_6X6_250, "6X6_250/30mm": cv2.aruco.DICT_6X6_250, "6X6_250/40mm": cv2.aruco.DICT_6X6_250, "6X6_250/50mm": cv2.aruco.DICT_6X6_250, "6X6_250/100mm": cv2.aruco.DICT_6X6_250,
    "6X6_250/150mm": cv2.aruco.DICT_6X6_250, "6X6_250/200mm": cv2.aruco.DICT_6X6_250, "6X6_250/250mm": cv2.aruco.DICT_6X6_250, "6X6_250/300mm": cv2.aruco.DICT_6X6_250,
    "6X6_1000/20mm": cv2.aruco.DICT_6X6_1000, "6X6_1000/30mm": cv2.aruco.DICT_6X6_1000, "6X6_1000/40mm": cv2.aruco.DICT_6X6_1000, "6X6_1000/50mm": cv2.aruco.DICT_6X6_1000, "6X6_1000/100mm": cv2.aruco.DICT_6X6_1000,
    "6X6_1000/150mm": cv2.aruco.DICT_6X6_1000, "6X6_1000/200mm": cv2.aruco.DICT_6X6_1000, "6X6_1000/250mm": cv2.aruco.DICT_6X6_1000, "6X6_1000/300mm": cv2.aruco.DICT_6X6_1000,
    "7X7_50/20mm": cv2.aruco.DICT_7X7_50, "7X7_50/30mm": cv2.aruco.DICT_7X7_50, "7X7_50/40mm": cv2.aruco.DICT_7X7_50, "7X7_50/50mm": cv2.aruco.DICT_7X7_50, "7X7_50/100mm": cv2.aruco.DICT_7X7_50,
    "7X7_50/150mm": cv2.aruco.DICT_7X7_50, "7X7_50/200mm": cv2.aruco.DICT_7X7_50, "7X7_50/250mm": cv2.aruco.DICT_7X7_50, "7X7_50/300mm": cv2.aruco.DICT_7X7_50,
    "7X7_100/20mm": cv2.aruco.DICT_7X7_100, "7X7_100/30mm": cv2.aruco.DICT_7X7_100, "7X7_100/40mm": cv2.aruco.DICT_7X7_100, "7X7_100/50mm": cv2.aruco.DICT_7X7_100, "7X7_100/100mm": cv2.aruco.DICT_7X7_100,
    "7X7_100/150mm": cv2.aruco.DICT_7X7_100, "7X7_100/200mm": cv2.aruco.DICT_7X7_100, "7X7_100/250mm": cv2.aruco.DICT_7X7_100, "7X7_100/300mm": cv2.aruco.DICT_7X7_100,
    "7X7_250/20mm": cv2.aruco.DICT_7X7_250, "7X7_250/30mm": cv2.aruco.DICT_7X7_250, "7X7_250/40mm": cv2.aruco.DICT_7X7_250, "7X7_250/50mm": cv2.aruco.DICT_7X7_250, "7X7_250/100mm": cv2.aruco.DICT_7X7_250,
    "7X7_250/150mm": cv2.aruco.DICT_7X7_250, "7X7_250/200mm": cv2.aruco.DICT_7X7_250, "7X7_250/250mm": cv2.aruco.DICT_7X7_250, "7X7_250/300mm": cv2.aruco.DICT_7X7_250,
    "7X7_1000/20mm": cv2.aruco.DICT_7X7_1000, "7X7_1000/30mm": cv2.aruco.DICT_7X7_1000, "7X7_1000/40mm": cv2.aruco.DICT_7X7_1000, "7X7_1000/50mm": cv2.aruco.DICT_7X7_1000, "7X7_1000/100mm": cv2.aruco.DICT_7X7_1000,
    "7X7_1000/150mm": cv2.aruco.DICT_7X7_1000, "7X7_1000/200mm": cv2.aruco.DICT_7X7_1000, "7X7_1000/250mm": cv2.aruco.DICT_7X7_1000, "7X7_1000/300mm": cv2.aruco.DICT_7X7_1000,
    "ORIGINAL/20mm": cv2.aruco.DICT_ARUCO_ORIGINAL,	"ORIGINAL/30mm": cv2.aruco.DICT_ARUCO_ORIGINAL,	"ORIGINAL/40mm": cv2.aruco.DICT_ARUCO_ORIGINAL,	"ORIGINAL/50mm": cv2.aruco.DICT_ARUCO_ORIGINAL,	"ORIGINAL/100mm": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"ORIGINAL/150mm": cv2.aruco.DICT_ARUCO_ORIGINAL, "ORIGINAL/200mm": cv2.aruco.DICT_ARUCO_ORIGINAL, "ORIGINAL/250mm": cv2.aruco.DICT_ARUCO_ORIGINAL, "ORIGINAL/300mm": cv2.aruco.DICT_ARUCO_ORIGINAL,
}  # some typical ArUco markers dictionaries

def save_intrinsic_parameters(intrinsic_mat, dist_coeffs, reprojection_error, file_path):  # save the camera's intrinsic parameters to a file
    with open(file_path, "w") as file:  # open the file in write mode
        file.write("Camera intrinsic matrix:\n")  # write the camera's intrinsic matrix to the file
        np.savetxt(file, intrinsic_mat, fmt = "%.5f", delimiter = " ")  # write the camera's intrinsic matrix to the file
        file.write("\nCamera distortion coefficients:\n")  # write the camera's distortion coefficients to the file
        np.savetxt(file, dist_coeffs, fmt = "%.5f", delimiter = " ")  # write the camera's distortion coefficients to the file
        file.write("\nReprojection error:\n")  # write the reprojection error to the file
        np.savetxt(file, np.array([reprojection_error]), fmt = "%.5f", delimiter = " ")  # write the reprojection error to the file
        file.close()  # close the file

def load_intrinsic_parameters(file_path):  # load the camera's intrinsic parameters from a file
    with open(file_path, "r") as file:  # open the file in read mode
        lines = file.readlines()  # read the lines of the file
        intrinsic_mat = np.array([list(map(float, line.strip().split())) for line in lines[1:4]])  # read the camera's intrinsic matrix from the file
        dist_coeffs = np.array([list(map(float, line.strip().split())) for line in lines[6:11]]).flatten()  # read the camera's distortion coefficients from the file
        reprojection_error = float(lines[13].strip())  # read the reprojection error from the file
        file.close()  # close the file
    return intrinsic_mat, dist_coeffs, reprojection_error  # return the camera's intrinsic matrix, the distortion coefficients and the reprojection error (in pixels)

def min_distance_from_camera(plane_width, plane_height, frame_width, frame_height, dFoV):  # calculate the minimum allowed distance from the camera to a vertical (to the camera optical axis) plane
    if plane_width > plane_height * frame_width / frame_height:
        side_length_factor = plane_width / frame_width
    else:
        side_length_factor = plane_height / frame_height
    diag_length = side_length_factor * np.sqrt(frame_width ** 2 + frame_height ** 2)
    min_distance = diag_length / (2.0 * np.tan(np.deg2rad(dFoV) / 2.0))  # calculate the minimum allowed distance from the camera to the plane
    return min_distance  # return the minimum allowed distance from the camera to the plane (in meters)

def create_image_window(window_name, width, height):  # create the image window
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # create the window with the given name
    cv2.resizeWindow(window_name, int(width), int(height))  # resize the window to the given width and height

def show_images_on_screen(images, windows_names = [], size_factor = 2/3, show_images_flag = True, hold_images_flag = False, hold_images_time = 0):  # show the images with the given window names and size factor
    images = [np.array(image) for image in images]  # make sure the images are numpy arrays
    if len(windows_names) != len(images):  # if the number of window names is not equal to the number of images
        windows_names += [f"Image {k}" for k in range(abs(len(images) - len(windows_names)))]  # create window names for the images
    if show_images_flag:  # show the camera's original image
        for k in range(len(images)):  # for each image
            create_image_window(windows_names[k], size_factor * images[k].shape[1], size_factor * images[k].shape[0])
            cv2.imshow(windows_names[k], images[k])  # show the camera's original image
        if hold_images_flag: cv2.waitKey(int(hold_images_time))  # wait for some time
        else: pass  # continue

def convert_image_pixel_to_grayscale(pixel_luminance, luminance_thresholds):  # convert an image to grayscale
    luminance_thresholds = sorted(luminance_thresholds)  # sort the luminance thresholds in ascending order
    grayscale = np.linspace(0, 255, len(luminance_thresholds) + 1)  # create a list of grayscale values
    for k in range(len(luminance_thresholds)):
        if pixel_luminance <= luminance_thresholds[k]:  # if the pixel's luminance is less than or equal to the current luminance threshold
            return grayscale[k]  # return the grayscale value corresponding to the current luminance threshold
    return grayscale[-1]  # return the last grayscale value if the pixel's luminance is greater than the last (and biggest) luminance threshold

def calibrate_camera(calibration_images_path):  # calibrate the camera
    max_iterations = 100; epsilon_error = 1e-5  # the maximum number of iterations allowed and the desired accuracy
    termination_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iterations, epsilon_error)  # the termination criteria
    # sample some world space 3d points, based on the specific chessboard pattern used
    grid_rows = 9  # the grid rows, the number of inner corners in the chessboard pattern along the y-axis
    grid_columns = 6  # the grid columns, the number of inner corners in the chessboard pattern along the x-axis
    world_points_grid = np.zeros((grid_rows * grid_columns, 3), np.float32)  # initialize the world space 3d points grid
    world_points_grid[:, :2] = np.mgrid[0 : grid_rows, 0 : grid_columns].T.reshape(-1, 2)  # the world space 3d points grid (I assume the chessboard is on the xy plane, with z = 0)
    world_points_grid[:, 2] = 0  # set the z coordinate of the world space 3d points grid to 0
    world_points = []  # initialize the world space 3d points
    image_points = []  # initialize the image plane 2d points
    # calibrate the camera looping through all the images with the chessboard pattern
    images = glob.glob(calibration_images_path)  # load the images that contain the chessboard pattern in different poses for the calibration
    if len(images) != 0:  # if there are images to calibrate the camera
        print(f"Number of images for camera calibration: {len(images)}")  # print the number of images used for the camera calibration
        for img in images:  # loop through the images to calibrate the camera
            original_image_drawn_chessboard = cv2.imread(img)  # load the image
            gray_image = cv2.cvtColor(original_image_drawn_chessboard, cv2.COLOR_BGR2GRAY)  # convert the image to grayscale
            corners_found, corners = cv2.findChessboardCorners(gray_image, (grid_rows, grid_columns), None)  # find the chessboard corners
            if corners_found == True:  # if the corners are found
                world_points.append(world_points_grid)  # add the world space 3d points grid
                corners_refined = cv2.cornerSubPix(gray_image, corners, (11, 11), (-1, -1), termination_criteria)  # find more precise corner positions
                cv2.drawChessboardCorners(original_image_drawn_chessboard, (grid_rows, grid_columns), corners_refined, corners_found)  # draw the corners on the chessboard
                image_points.append(corners_refined)
                show_images_on_screen([original_image_drawn_chessboard], ["Original images with the drawn corners on the chessboard pattern"], 1, True, True, 1000)  # show the calibration images with the corners on the chessboard
        if len(image_points) != 0:  # if there are image points to calibrate the camera
            print(f"Number of images with chessboard corners found: {len(image_points)}")  # print the number of images with chessboard corners found
            # calibrate the camera and get the distortion coefficients and the intrinsic parameters of the intrinsic matrix
            reprojection_error, intrinsic_mat, dist_coeffs, _, _ = cv2.calibrateCamera(world_points, image_points, gray_image.T.shape[:2], None, None)  # calibrate the camera
            print(f"Reprojection error (in pixels): {reprojection_error:.5f}")
            print(f"Camera intrinsic matrix (in pixels): {np.round(intrinsic_mat, 3)}")
            print(f"Camera distortion coefficients (dimensionless): {dist_coeffs.flatten()}")
            return intrinsic_mat, dist_coeffs.flatten(), reprojection_error  # return the intrinsic matrix, the distortion coefficients and the reprojection error (in pixels)
        else:  # if there are no corners found in the images
            print("No corners found in the images!")  # print that there are no corners found in the images
            return None, None, None  # return None for the intrinsic matrix, the distortion coefficients and the reprojection error
    else:  # if there are no images to calibrate the camera
        print("No images found for camera calibration!")  # print that there are no images to calibrate the camera
        return None, None, None  # return None for the intrinsic matrix, the distortion coefficients and the reprojection error

def undistort_image_pixels(pixels, intrinsic_mat, dist_coeffs):  # undistort the pixels of a distorted image, based on the camera's intrinsic matrix and the distortion coefficients
    pixels = np.float32(pixels).reshape(-1, 1, 2)  # make sure the pixels are a numpy array with the right shape
    undistorted_pixels = np.int32(cv2.undistortPoints(pixels, intrinsic_mat, dist_coeffs, P = intrinsic_mat))  # undistort the pixels of the distorted image and convert them to integers
    return undistorted_pixels.reshape(-1, 2)  # return the undistorted pixels

def undistort_whole_image(image, intrinsic_mat, dist_coeffs):  # undistort the whole image, based on the camera's intrinsic matrix and the distortion coefficients
    image = np.array(image)  # make sure the image is a numpy array
    undistorted_image = cv2.undistort(image, intrinsic_mat, dist_coeffs, None, intrinsic_mat)  # undistort the image
    return undistorted_image  # return the undistorted image

def estimate_ArUco_marker_pose(image, ArUco_marker, intrinsic_mat, dist_coeffs):  # estimate the ArUco markers poses with respect to the camera
    image = np.array(image)  # make sure the image is a numpy array
    if ArUco_marker not in ArUco_dictionaries.keys():  # if the ArUco marker is not in the available ArUco markers dictionaries
        print(f"The ArUco marker {ArUco_marker} is not in the available ArUco markers dictionaries!")
        return np.eye(4), image  # return None for the ArUco marker transformation matrix and the image with the detected markers and their poses
    else:  # if the ArUco marker is in the available ArUco markers dictionaries
        ArUco_dict = cv2.aruco.Dictionary_get(ArUco_dictionaries[ArUco_marker])  # get the specific ArUco dictionary
        parameters = cv2.aruco.DetectorParameters_create()  # create the ArUco detector parameters
        corners, ids, _ = cv2.aruco.detectMarkers(image, ArUco_dict, parameters = parameters)  # detect the ArUco markers in the image
        ArUco_marker_side_length = float(ArUco_marker.split("/")[1].split("mm")[0]) / 1000.0  # get the ArUco marker side length in meters
        if ids is not None:  # if markers are detected
            for k in range(len(ids)):  # for each detected ArUco marker
                rotation_vec, translation_vec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[k], ArUco_marker_side_length, intrinsic_mat, dist_coeffs)  # estimate the pose of the ArUco marker
                if k == 0:  # if it is the first ArUco marker
                    rotation_matrix, _ = cv2.Rodrigues(rotation_vec)  # convert the rotation vector to a rotation matrix
                    ArUco_transformation_matrix = np.eye(4)  # initialize the transformation matrix
                    ArUco_transformation_matrix[:3, :3] = rotation_matrix.reshape(3, 3)  # set the rotation matrix in the transformation matrix
                    ArUco_transformation_matrix[:3, 3] = translation_vec.reshape(3,)  # set the translation vector in the transformation matrix
                cv2.drawFrameAxes(image, intrinsic_mat, dist_coeffs, rotation_vec, translation_vec, [0.1, 0.05][[True, False].index(k == 0)])  # draw the axis of each ArUco marker            
            cv2.aruco.drawDetectedMarkers(image, corners, ids)  # draw the detected ArUco markers
            print(f"ArUco markers detected: {ids.flatten()}")  # print the detected ArUco markers
            return ArUco_transformation_matrix, image  # return the ArUco marker transformation matrix and the image with the detected markers and their poses
        else:  # if no markers are detected
            print("No ArUco marker detected.")
            return np.eye(4), image  # return None for the ArUco marker transformation matrix and the image with the detected markers and their poses

def draw_shape_on_image_plane(image, shape_points, ArUco_wrt_camera_transformation_matrix, intrinsic_mat, dist_coeffs):  # draw a shape on the image plane
    # the shape points coordinates are with respect to the ArUco marker frame
    image = np.array(image)  # make sure the image is a numpy array
    shape_points_wrt_ArUco_frame = np.array(shape_points, dtype = np.float32)  # make the shape points a numpy array
    shape_points_homogeneous = np.hstack((shape_points_wrt_ArUco_frame, np.ones((len(shape_points), 1)))).T  # make the coordinates of the shape points homogeneous
    camera_shape_points_homogeneous = ArUco_wrt_camera_transformation_matrix @ shape_points_homogeneous  # transform the shape points from the ArUco marker frame to the camera frame
    camera_shape_points = camera_shape_points_homogeneous[:3, :].T  # get the camera coordinates of the shape points
    image_shape_points, _ = cv2.projectPoints(camera_shape_points, np.zeros(3), np.zeros(3), intrinsic_mat, dist_coeffs)  # project the camera frame coordinates to the image plane that is shown in the image
    # draw the shape on the image plane
    image_shape_points = np.int32(image_shape_points.reshape(-1, 2))  # convert the image points to integers
    for k in range(len(shape_points)):
        image_shape_points[k] = tuple(image_shape_points[k])  # convert the image points to tuples
        cv2.line(image, image_shape_points[k], image_shape_points[(k + 1) % len(shape_points)], (255, 0, 255), 2)  # draw the four sides of the shape on the image
    return image_shape_points, image  # return the image points of the shape drawn on the image plane and the image with the drawn shape

def detect_image_boundaries(grayscale_image, image_plane_points, removed_boundaries_indexes, boundaries_precision_parameter, boundaries_minimum_vertices):  # find the boundaries of the shapes in an image (in pixels)
    # grayscale_image is the binary image needed (in the form of a numpy array) for the Suzuki - Abe algorithm to work
    grayscale_image = np.array(grayscale_image)  # make sure the grayscale image is a numpy array
    boundaries, hierarchy = cv2.findContours(grayscale_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # find the boundaries of the shapes in the image (other options: cv2.RETR_TREE, cv2.RETR_EXTERNAL, cv2.RETR_CCOMP and cv2.CHAIN_APPROX_NONE)
    # approximate the boundaries of the shapes in the image using closed polygonal curves
    image_plane_points = np.array(image_plane_points)  # make sure the plane image points are a numpy array
    image_plane_polygon = Polygon(image_plane_points)  # create a polygon object from the plane image points
    detected_image_boundaries_list = []  # initialize the list of the approximations of the boundaries
    for cntr in boundaries:  # loop through all the boundaries of the shapes in the image
        epsilon = boundaries_precision_parameter * cv2.arcLength(cntr, True)  # calculate the epsilon parameter for the approximation of the boundaries
        polygonal_approximation = np.int32(cv2.approxPolyDP(cntr, epsilon, True).reshape(-1, 2))  # approximate the boundaries of the shapes in the image using closed polygonal curves
        # exclude the polygonal approximations that are (partially or totally) outside the obstacles plane or/and have less than the minimum number of vertices allowed
        polygonal_approximation = np.array([point for point in polygonal_approximation if image_plane_polygon.contains(Point(point))]).reshape(-1, 2)  # cut the polygonal approximation to the plane image polygon
        if polygonal_approximation.shape[0] >= 0 and polygonal_approximation.shape[0] >= boundaries_minimum_vertices:  # if the number of vertices of the approximation is greater than or equal to the minimum number of vertices allowed
            # close the open boundaries
            if np.linalg.norm(polygonal_approximation[0] - polygonal_approximation[-1]) != 0:  # if the first and last points of the approximation are not the same
                polygonal_approximation = np.vstack((polygonal_approximation, polygonal_approximation[0]))  # add the first point to the end of the approximation to close the boundary
            # ensure clockwise orientation of all the boundaries
            if cv2.contourArea(polygonal_approximation, oriented = True) > 0:  # if the signed area of the boundary is positive
                polygonal_approximation = polygonal_approximation[::-1]  # reverse the order of the points to make the orientation clockwise
            detected_image_boundaries_list.append(polygonal_approximation)  # add the approximations of the boundaries to the list
    # find the outer boundary among the non-removed boundaries and orient it anti-clockwise
    outer_boundary_index = -1  # initialize the index of the outer boundary
    for i in range(len(detected_image_boundaries_list)):  # loop through all the currently stored boundaries
        if i not in removed_boundaries_indexes:  # if the i boundary is not removed
            polygon_i = Polygon(np.int32(detected_image_boundaries_list[i]).reshape(-1, 2))  # create a polygon object from the i boundary
            inner_boundaries_counter = 0  # initialize the inner boundaries counter
            for j in range(len(detected_image_boundaries_list)):  # loop through all the currently stored boundaries
                if i != j and j not in removed_boundaries_indexes:  # if the i boundary is not the same as the j boundary and the j boundary is not removed
                    point_j_check = np.int32(detected_image_boundaries_list[j][0]).reshape(1, 2)  # get the first point of the j boundary
                    if polygon_i.contains(Point(point_j_check)):  # if the first point of the j boundary is inside the i boundary
                        inner_boundaries_counter += 1  # increase the inner boundaries counter
            if inner_boundaries_counter == len(detected_image_boundaries_list) - len(removed_boundaries_indexes) - 1:  # if the number of the boundaries inside the i boundary is correct 
                outer_boundary_index = i  # set the index of the outer boundary
                detected_image_boundaries_list[i] = detected_image_boundaries_list[i][::-1]  # reverse the order of the points to make the orientation of the outer boundary anti-clockwise
                break  # break the loop
    # draw the allowed detected boundaries
    boundaries_image = cv2.cvtColor(255 * np.ones_like(grayscale_image), cv2.COLOR_GRAY2BGR)  # create a bgr image with the same size as the grayscale image
    for k in range(len(detected_image_boundaries_list)):  # loop through all the detected boundaries
        if k not in removed_boundaries_indexes:  # if the boundary is not removed
            cv2.drawContours(boundaries_image, [detected_image_boundaries_list[k]], 0, [(255, 0, 0), (0, 0, 255)][[False, True].index(outer_boundary_index == k)], 1)  # draw the boundaries of the shapes in the image with the specified color and thickness
            cv2.putText(boundaries_image, f"{k + 1}", tuple(detected_image_boundaries_list[k][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)  # write the number of the boundary on the image
    cv2.putText(boundaries_image, f"Number of total boundaries: {len(detected_image_boundaries_list)},     Number of removed boundaries: {len(removed_boundaries_indexes)},     Precision: {boundaries_precision_parameter:.5f},     Minimum vertices: {boundaries_minimum_vertices}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)  # write the number of boundaries on the image
    return detected_image_boundaries_list, outer_boundary_index, boundaries_image  # return the boundaries of the shapes in the image (in pixels), the index of the outer boundary and the whole image with the detected boundaries drawn

def convert_boundaries_image_points_to_world_points(detected_image_boundaries_list, plane_frame_wrt_world, plane_normal_vec_displacement, camera_frame_wrt_world, intrinsic_mat, dist_coeffs):  # convert the detected image boundaries to the xy plane world frame coordinates (in meters)
    # find the plane frame that is tangent to the top of the obstacles, based on the initial plane frame and the height of the obstacles
    top_plane_frame_wrt_world = plane_frame_wrt_world.copy()  # initialize the top plane frame matrix wrt the world frame
    normal_vec, _, translation_vec = gf.get_components_from_xy_plane_transformation(plane_frame_wrt_world)  # get the normal vector and translation vector of the plane frame with respect to the world frame
    top_plane_position = translation_vec + plane_normal_vec_displacement * normal_vec  # calculate the position of the top plane
    top_plane_frame_wrt_world[:3, 3] = top_plane_position  # set the position of the top plane
    # find the boundaries of the shapes in the image converted to the xy plane world frame coordinates (in meters)
    intrinsic_mat_inverse = np.linalg.inv(intrinsic_mat)  # find the inverse of the intrinsic matrix
    camera_wrt_obstacles_frame_mat = np.linalg.inv(top_plane_frame_wrt_world) @ camera_frame_wrt_world  # find the camera frame wrt the obstacles frame matrix
    detected_image_boundaries_list = [undistort_image_pixels(boundary, intrinsic_mat, dist_coeffs) for boundary in detected_image_boundaries_list]  # undistort the image pixels of the detected boundaries
    boundaries_world_points_list = []  # initialize the list of the boundaries in the world frame
    for k in range(len(detected_image_boundaries_list)):  # loop through all the detected boundaries
        boundaries_world_points_list.append([])  # add an empty list for the current boundary
        boundary = detected_image_boundaries_list[k]  # get the current boundary
        for pixel_image_point in boundary:  # loop through all the points of the boundary
            pixel_image_point = np.vstack((np.array(pixel_image_point).reshape(-1, 1), np.ones((1, 1))))  # convert the pixel image point to homogeneous coordinates (in pixels)
            obst_point_normalized_wrt_camera = intrinsic_mat_inverse @ pixel_image_point  # find the pixel point normalized wrt the camera frame (in meters, in homogeneous coordinates)
            Z_obst_wrt_camera = float(-camera_wrt_obstacles_frame_mat[2, 3] / (camera_wrt_obstacles_frame_mat[2, :3] @ obst_point_normalized_wrt_camera[:3]))  # find the z coordinate of the obstacles wrt the camera frame
            obst_point_wrt_camera = np.array([[Z_obst_wrt_camera, 0, 0], [0, Z_obst_wrt_camera, 0], [0, 0, Z_obst_wrt_camera], [0, 0, 1]]) @ obst_point_normalized_wrt_camera  # find the pixel point wrt the camera frame (in meters, in homogeneous coordinates)
            obstacle_xy_world_point = camera_wrt_obstacles_frame_mat @ obst_point_wrt_camera  # find the obstacle xy world point
            boundaries_world_points_list[-1].append(np.array([obstacle_xy_world_point[0], obstacle_xy_world_point[1]]))  # add the obstacle xy world point to the current boundary
    return boundaries_world_points_list  # return the boundaries of the shapes in the image converted to the xy plane world frame coordinates (in meters)


# import os
# from PIL import Image

# intrinsic_mat, dist_coeffs, _ = load_intrinsic_parameters("camera_intrinsic_parameters.txt")
# dist_coeffs[0] = 1
# image = cv2.imread("distorted_image.jpg")
# height, width = image.shape[:2]
# undistorted_image1 = undistort_whole_image(image, intrinsic_mat, dist_coeffs)

# h, w = image.shape[:2]
# x, y = np.meshgrid(np.arange(w), np.arange(h))
# grid_pixels = np.stack((x.ravel(), y.ravel()), axis = -1)
# undistorted_pixels = undistort_image_pixels(grid_pixels, intrinsic_mat, dist_coeffs)
# undistorted_image2 = np.zeros_like(image)
# image = image.ravel()
# for i, (x, y) in enumerate(undistorted_pixels):
#     x, y = int(round(x)), int(round(y))
#     if 0 <= x < w and 0 <= y < h:
#         undistorted_image2[y, x] = image[i * 3:i * 3 + 3]  # Handle color images
# # undistorted_image2 = undistort_image2(image, intrinsic_mat, dist_coeffs)
# show_images_on_screen([undistorted_image1], ["Undistorted image 1"], 1, True, True, 0)
# show_images_on_screen([undistorted_image2], ["Undistorted image 2"], 1, True, True, 0)

# intrinsic_mat, dist_coeffs, _ = load_intrinsic_parameters("camera_intrinsic_parameters.txt")
# camera_capture = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # initialize the camera capture object, 0 for the default camera, 1 for the first external camera
# camera_aspect_ratios_frames_heights = {16/9: 720, 4/3: 480}  # define the aspect ratios and their corresponding frame heights
# aspect_ratio = 16/9  # define the desired aspect ratio of the captured frame
# frame_height = camera_aspect_ratios_frames_heights[aspect_ratio]  # get the desired height of the captured frame
# frame_width = int(aspect_ratio * frame_height)  # calculate the desired width of the captured frame
# camera_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)  # set the height of the captured frame
# camera_capture.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)  # set the width of the captured frame
# luminance_threshold = 127  # define the luminance threshold for the black and white conversion of the captured frame

# while True:  # infinite loop to continuously capture frames from the camera
#     confirm_capture, frame = camera_capture.read()  # capture a frame using the camera capture object and store it in the variable frame
#     if confirm_capture:  # if the frame was captured successfully
#         # show camera's captured frame
#         original_image_window_name = "Original frames"  # define the window name for the original frame
#         # show_images_on_screen([frame], [original_image_window_name], size_factor, True, False, 0)  # show the camera's original image

#         image = Image.fromarray(frame)  # convert the captured frame to an image
        
#         ArUco_wrt_camera_transformation_matrix, ArUco_marker_pose_image = estimate_ArUco_marker_pose(frame, "4X4_100/100mm", intrinsic_mat, dist_coeffs)  # estimate the pose of the ArUco markers in the image
#         # show_images_on_screen([ArUco_marker_pose_image], ["ArUco markers poses"], 2/3, True, False, 0)  # show the image with the detected ArUco markers and their poses
        
#         # print(ArUco_wrt_camera_transformation_matrix.round(3))
#         # Define rectangle corners in the world frame
#         x_length = 0.21
#         y_length = 0.15
#         world_corners = np.array([
#             [0, 0, 0],
#             [x_length, 0, 0],
#             [x_length, y_length, 0],
#             [0, y_length, 0]
#         ], dtype = np.float32)
#         image_shape_points, drawn_shape_image = draw_shape_on_image_plane(frame, world_corners, ArUco_wrt_camera_transformation_matrix, intrinsic_mat, dist_coeffs)
#         print(image_shape_points)
#         show_images_on_screen([drawn_shape_image], ["Drawn shape"], 2/3, True, False, 0)  # show the image with the drawn shape on the image plane
        
#         grayscale_image = image.convert("L").point(lambda pixel: convert_image_pixel_to_grayscale(pixel, [luminance_threshold]))  # convert the captured frame to black and white
#         show_images_on_screen([grayscale_image], ["Grayscale frames"], 2/3, True, False, 0)  # show the camera's original and grayscale images
#         boundaries, _, boundaries_image = detect_image_boundaries(np.array(grayscale_image), image_shape_points, 10**(-4), 10)  # find the boundaries of the shapes in the image
#         show_images_on_screen([boundaries_image], ["boundaries detected"], 1, True, False, 0)  # show the image with the detected shapes and boundaries
        
#         # undistorted_image = undistort_image(image, intrinsic_mat, dist_coeffs, 1)
#         # show_images_on_screen([undistorted_image], ["Undistorted image"], 2/3, True, True, 60000)

#     else:  # if the frame was not captured successfully
#         print("Error: Couldn't capture frame.")  # print an error message
#         break  # break the loop
    
#     pressed_key = cv2.waitKey(1)  # wait for 1 millisecond for a key to be pressed
#     if pressed_key == ord("a"): luminance_threshold -= 1
#     elif pressed_key == ord("w"): luminance_threshold += 1
#     elif pressed_key == ord("q"): break

# camera_capture.release()  # release/free the camera capture object
# cv2.destroyAllWindows()  # close all OpenCV windows that were opened
