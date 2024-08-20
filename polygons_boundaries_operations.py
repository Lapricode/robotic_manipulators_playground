import numpy as np
from scipy.ndimage import convolve
from shapely.geometry import Polygon, Point
from skimage import measure
import trimesh
import matplotlib.pyplot as plt


# functions for polygons boundaries operations

def points_in_polygon_detection(polygon_vertices, points):  # detect if the given points are inside the given polygon
    polygon_vertices = np.array(polygon_vertices).reshape(-1, 2)  # make sure the polygon vertices are a numpy array
    polygon = Polygon(polygon_vertices)  # create a polygon object from the x and y coordinates of the vertices of the polygon
    points = np.array(points).reshape(-1, 2)  # make sure the points are a numpy array
    return np.array([polygon.contains(Point(point)) for point in points]).reshape(-1, 2)  # for each point, check if it is inside the polygon

def convex_polygon_interior_detection(polygon_vertices, xy_limits, xy_resolutions, show_plots = False):  # detect the interior of a convex polygon
    # convert the given polygon to a convex polygon
    polygon_vertices = np.array(polygon_vertices).reshape(-1, 2)  # make sure the polygon vertices are a numpy array
    convex_hull = Polygon(polygon_vertices).convex_hull  # create the convex hull of the polygon in order to force it to be convex, in case it is originally concave
    x_hull, y_hull = convex_hull.exterior.coords.xy  # extract the x and y coordinates of the vertices of the convex hull
    polygon_vertices = np.vstack([x_hull, y_hull]).T  # create a numpy array of the vertices of the convex hull
    x_grid = np.linspace(xy_limits[0][0], xy_limits[0][1], xy_resolutions[0])  # create uniformly spaced x points
    y_grid = np.linspace(xy_limits[1][0], xy_limits[1][1], xy_resolutions[1])  # create uniformly spaced y points
    x_grid, y_grid = np.meshgrid(x_grid, y_grid, indexing = "xy")  # create a meshgrid of the x and y points
    grid_points = np.vstack([x_grid.flatten(), y_grid.flatten()]).T  # create the grid points
    # find the interior points of the convex polygon using shapely
    interior_mask = points_in_polygon_detection(polygon_vertices, grid_points)  # for each grid point, check if it is inside the polygon
    # create plots
    if show_plots:
        fig = plt.figure()
        ax1 = fig.add_subplot(121)
        ax1.fill(np.array(x_hull), np.array(y_hull), edgecolor = "black", fill = True, color = "skyblue")
        ax1.legend(["Convex hull of the polygon"])
        ax1.set_title(f"Convex polygon shape")
        ax1.set_xlabel("x (mm)"); ax1.set_ylabel("y (mm)"); ax1.set_aspect("equal")
        ax2 = fig.add_subplot(122)
        ax2.scatter(grid_points[interior_mask][:, 0], grid_points[interior_mask][:, 1], color = "green")
        ax2.scatter(grid_points[~interior_mask][:, 0], grid_points[~interior_mask][:, 1], color = "red")
        ax2.legend(["Interior points", "Exterior points"])
        ax2.set_title(f"Interior and exterior points\nof the polygon")
        ax2.set_xlabel("x (mm)"); ax2.set_ylabel("y (mm)"); ax2.set_aspect("equal")
        plt.show()
    return interior_mask.reshape(xy_resolutions[0], xy_resolutions[1]).T  # return the grid points that are inside the polygon

def convex_polygon_interior_detection_2(polygon_vertices, xy_limits, xy_resolutions, show_plots = False):  # second function to detect the interior of a convex polygon
    # convert the given polygon to a convex polygon
    polygon_vertices = np.array(polygon_vertices).reshape(-1, 2)  # make sure the polygon vertices are a numpy array
    polygon = Polygon(polygon_vertices)  # create a polygon object from the x and y coordinates of the vertices of the polygon
    convex_hull = polygon.convex_hull  # create the convex hull of the polygon in order to force it to be convex, in case it is originally concave
    x_hull, y_hull = convex_hull.exterior.coords.xy  # extract the x and y coordinates of the vertices of the convex hull
    polygon_vertices = np.vstack([x_hull, y_hull]).T  # create a numpy array of the vertices of the convex hull
    x_grid = np.linspace(xy_limits[0][0], xy_limits[0][1], xy_resolutions[0])  # create uniformly spaced x points
    y_grid = np.linspace(xy_limits[1][0], xy_limits[1][1], xy_resolutions[1])  # create uniformly spaced y points
    x_grid, y_grid = np.meshgrid(x_grid, y_grid, indexing = "xy")  # create a meshgrid of the x and y points
    grid_points = np.vstack([x_grid.flatten(), y_grid.flatten()]).T  # create the grid points
    # find the interior points of the convex polygon using the convex polygon property of being on the same half-plane with respect to every one of its edges
    interior_mask = np.zeros((len(grid_points),), dtype = bool)
    for k in range(len(grid_points)):
        x, y = grid_points[k]
        signs_list = []
        for i in range(len(polygon_vertices) - 1):
            x1, y1 = polygon_vertices[i]
            x2, y2 = polygon_vertices[(i + 1) % len(polygon_vertices)]
            sign = (x1 - x2) * y + (y2 - y1) * x + y1 * x2 - y2 * x1
            signs_list.append(sign)
        if all(sign > 0 for sign in signs_list) or all(sign < 0 for sign in signs_list):
            interior_mask[k] = True
        else:
            interior_mask[k] = False
    # create plots
    if show_plots:
        fig = plt.figure()
        ax1 = fig.add_subplot(121)
        ax1.fill(np.array(x_hull), np.array(y_hull), edgecolor = "black", fill = True, color = "skyblue")
        ax1.legend(["Convex hull of the polygon"])
        ax1.set_title(f"Convex polygon shape")
        ax1.set_xlabel("x (mm)"); ax1.set_ylabel("y (mm)"); ax1.set_aspect("equal")
        ax2 = fig.add_subplot(122)
        ax2.scatter(grid_points[interior_mask][:, 0], grid_points[interior_mask][:, 1], color = "green")
        ax2.scatter(grid_points[~interior_mask][:, 0], grid_points[~interior_mask][:, 1], color = "red")
        ax2.legend(["Interior points", "Exterior points"])
        ax2.set_title(f"Interior and exterior points\nof the polygon")
        ax2.set_xlabel("x (mm)"); ax2.set_ylabel("y (mm)"); ax2.set_aspect("equal")
        plt.show()
    return interior_mask.reshape(xy_resolutions[0], xy_resolutions[1]).T  # return the grid points that are inside the polygon

def any_polygon_interior_detection(polygon_vertices, xy_limits, xy_resolutions, show_plots = False):  # detect the interior of any polygon (convex or concave)
    # create the polygon object
    polygon_vertices = np.array(polygon_vertices).reshape(-1, 2)  # make sure the polygon vertices are a numpy array
    x_grid = np.linspace(xy_limits[0][0], xy_limits[0][1], xy_resolutions[0])  # create uniformly spaced x points
    y_grid = np.linspace(xy_limits[1][0], xy_limits[1][1], xy_resolutions[1])  # create uniformly spaced y points
    x_grid, y_grid = np.meshgrid(x_grid, y_grid, indexing = "xy")  # create a meshgrid of the x and y points
    grid_points = np.vstack([x_grid.flatten(), y_grid.flatten()]).T  # create the grid points
    # find the interior points of the polygon using shapely
    interior_mask = points_in_polygon_detection(polygon_vertices, grid_points)  # for each grid point, check if it is inside the polygon
    # create plots
    if show_plots:
        fig = plt.figure()
        ax1 = fig.add_subplot(121)
        ax1.fill(polygon_vertices[:, 0], polygon_vertices[:, 1], edgecolor = "black", fill = True, color = "skyblue")
        ax1.legend(["Polygon shape"])
        ax1.set_title(f"Convex or Concave polygon shape")
        ax1.set_xlabel("x (mm)"); ax1.set_ylabel("y (mm)"); ax1.set_aspect("equal")
        ax2 = fig.add_subplot(122)
        ax2.scatter(grid_points[interior_mask][:, 0], grid_points[interior_mask][:, 1], color = "green")
        ax2.scatter(grid_points[~interior_mask][:, 0], grid_points[~interior_mask][:, 1], color = "red")
        ax2.legend(["Interior points", "Exterior points"])
        ax2.set_title(f"Interior and exterior points\nof the polygon")
        ax2.set_xlabel("x (mm)"); ax2.set_ylabel("y (mm)"); ax2.set_aspect("equal")
        plt.show()
    return interior_mask.reshape(xy_resolutions[0], xy_resolutions[1]).T  # return the grid points that are inside the polygon

def convert_boundary_to_3D_mesh_stl(boundary_points, z_height, xy_limits, xy_resolutions, z_resolution, stl_file_path, show_mesh = False):  # convert a shape defined by a polygonal boundary to an stl file
    # create a 3d mask for the 3d shape
    boundary_name = stl_file_path.split("/")[-1].split(".")[0]  # get the stl file name
    boundary_points = np.array(boundary_points).reshape(-1, 2)  # make sure the boundary points are a numpy array
    mask_3d_shape = np.zeros((xy_resolutions[0], xy_resolutions[1], z_resolution), dtype = bool)  # create a mask to detect the interior of the 3d shape
    if "outer" in boundary_name:  # if the boundary is an outer boundary
        extend_factor = 1/5  # the factor to extend the xy limits to include the boundary
        extension = max(extend_factor * abs(xy_limits[0][0]), extend_factor * abs(xy_limits[0][1]), extend_factor * abs(xy_limits[1][0]), extend_factor * abs(xy_limits[1][1]))  # calculate the extension to include the boundary
        xy_limits = [[xy_limits[0][0] - extension, xy_limits[0][1] + extension], [xy_limits[1][0] - extension, xy_limits[1][1] + extension]]  # extend the xy limits to include the boundary
        mask_2d_interior = any_polygon_interior_detection(boundary_points, xy_limits, xy_resolutions, False)  # detect the interior of the polygon for an inner boundary
        kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])  # create a kernel to detect the boundary of the polygon
        mask_2d_interior_edge = np.array(mask_2d_interior, copy = True)  # create a copy of the 2d mask to detect the boundary
        for k in range(5):  # apply the kernel twice to detect the boundary
            mask_2d_interior_edge = convolve(mask_2d_interior_edge, kernel, mode = "constant", cval = 0.0)  # apply the kernel to detect the boundary of the polygon, keeping the interior
        mask_2d_interior = np.logical_and(~mask_2d_interior, mask_2d_interior_edge)  # keep only the boundary and exclude the interior of the polygon
    else:  # if the boundary is an inner boundary
        mask_2d_interior = any_polygon_interior_detection(boundary_points, xy_limits, xy_resolutions, False)  # detect the interior of the polygon for an inner boundary
    for k in range(z_resolution):  # for each z point, create the corresponding 2d mask
        mask_3d_shape[:, :, k] = mask_2d_interior  # reshape the 2d mask to a 3d mask
    mask_3d_shape[:, :, 0] = False; mask_3d_shape[:, :, -1] = False  # set the first and last z isosurfaces to False, in order to close the top and bottom of the shape
    # create the shape's 3d mesh and save it to an stl file
    vertices, faces, normals, _ = measure.marching_cubes(mask_3d_shape, 0.0)  # create the vertices and faces of the shape's 3d mesh
    vertices_x_range = max(vertices[:, 0]) - min(vertices[:, 0]); vertices_y_range = max(vertices[:, 1]) - min(vertices[:, 1]); vertices_z_range = max(vertices[:, 2]) - min(vertices[:, 2])  # calculate the x, y, and z ranges of the vertices of the shape
    real_x_range = max(boundary_points[:, 0]) - min(boundary_points[:, 0]); real_y_range = max(boundary_points[:, 1]) - min(boundary_points[:, 1]); real_z_range = z_height  # calculate the real x, y, and z ranges of the 3d shape
    translate_mesh_vec = np.array([min(vertices[:, 0]), min(vertices[:, 1]), min(vertices[:, 2])], dtype = np.float32)  # calculate the translation vector to center the 3d shape
    scale_mesh_vec = np.array([real_x_range / vertices_x_range, real_y_range / vertices_y_range, real_z_range / vertices_z_range], dtype = np.float32)  # calculate the scaling vector to scale the 3d shape
    translate_shape_vec = np.array([min(boundary_points[:, 0]), min(boundary_points[:, 1]), 0.0], dtype = np.float32)  # calculate the translation vector to shift the 3d shape to its original position
    vertices = (vertices - translate_mesh_vec) * scale_mesh_vec + translate_shape_vec  # translate and scale the vertices of the 3d shape to match the real shape and then shift them to their original position
    shape_mesh = trimesh.Trimesh(vertices = vertices, faces = faces, vertex_normals = normals)  # create the shape's 3d mesh
    shape_mesh.invert()  # invert the normals of the shape's 3d mesh
    shape_mesh.export(stl_file_path)  # save the shape's 3d mesh to the stl file path provided
    # show the 3d mesh of the shape
    if show_mesh:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = "3d")
        ax.plot_trisurf(shape_mesh.vertices[:, 0], shape_mesh.vertices[:, 1], shape_mesh.vertices[:, 2], triangles = shape_mesh.faces, color = "skyblue")
        ax.set_title(f"3D mesh of the shape created by the boundary \"{boundary_name}\"")
        ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)"); ax.set_aspect("equal"), ax.view_init(elev = 45, azim = -90)
        plt.show()


# import os
# x_points = 10.0 * np.array([0.0, 3.0, 2.0, 4.0, 3.0])  # the x points of the obstacle's polygon on the xy plane in millimeters
# y_points = 10.0 * np.array([0.0, 0.0, 1.0, 3.0, 4.0])  # the y points of the obstacle's polygon on the xy plane in millimeters
# z_height = 10.0 * 2.0  # the height of the obstacle in millimeters
# xy_limits = [[min(x_points), max(x_points)], [min(y_points), max(y_points)]]  # the limits of the grid in the x and y directions in millimeters
# stl_file_path = os.getcwd() + r"/saved_workspace_obstacles_infos/obstacle.stl"
# convex_polygon_interior_detection(np.vstack((x_points, y_points)).T, xy_limits, [300, 300], True)
# any_polygon_interior_detection(np.vstack((x_points, y_points)).T, xy_limits, [300, 300], True)
# convert_boundary_to_3D_mesh_stl(np.vstack((x_points, y_points)).T, z_height, xy_limits, [100, 100], 10, stl_file_path, True)
