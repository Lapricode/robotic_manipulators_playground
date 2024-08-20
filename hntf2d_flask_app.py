from flask import Flask, request
import hntf2d.map
import hntf2d.utils
import numpy as np


app = Flask(__name__)  # create a new Flask app object
hm = None  # create a global variable to store the HarmonicMap2D object

# create new HarmonicMap2D object
@app.route("/create_new_harmonic_map_object", methods = ["POST"])  # create a route to build a new HarmonicMap2D object
def create_new_harmonic_map_object():  # create a function to build a new HarmonicMap2D object
    global hm  # use the global variable hm
    hm = hntf2d.map.HarmonicMap2D()  # build a new HarmonicMap2D object
    print(f"New HarmonicMap2D object created successfully!\n{hm}")  # print a message to the console
    return {"message": "New HarmonicMap2D object created successfully!"}  # return a message to the client

# calculate the harmonic transformation from the real workspace to the sphere world (unit disk)
@app.route("/calculate_harmonic_transformation", methods = ["POST"])  # create a route to calculate the transformation
def calculate_harmonic_transformation():  # calculate the harmonic transformation
    global hm  # use the global variable hm
    if hm is None: return {"message": "HarmonicMap2D object has not been created!"}, 400  # return an error message if the HarmonicMap2D object has not been created yet
    input_data = request.get_json()  # get the input data from the client (host machine or user)
    outer_boundary = input_data["outer_boundary"]  # get the outer boundary from the input data
    inner_boundaries = input_data["inner_boundaries"]  # get the inner boundaries from the input data
    # append the boundaries (outer and inner) to the HarmonicMap2D object
    hm.boundary_append(np.array(outer_boundary))  # append the outer boundary to the HarmonicMap2D object
    for k in range(len(inner_boundaries)):  # loop through all the inner boundaries
        hm.boundary_append(np.array(inner_boundaries[k]))  # append each inner boundary to the HarmonicMap2D object
    # convert the outer boundary to a unit circle consisiting of points with coordinates u0, v0
    u0, v0 = hntf2d.utils.poly2circle(np.array(outer_boundary)).T  # convert the outer boundary to a unit circle consisiting of points with coordinates u0, v0
    unit_circle = np.array([u0, v0]).T  # create a unit circle array with coordinates u0, v0
    # compute/solve the harmonic transformation
    hm.solve(u0, v0)  # compute the harmonic transformation
    # calculate the punctures of the harmonic transformation
    punctures = hm.puncts()  # calculate the punctures of the harmonic transformation
    return {"message": "Harmonic transformation from the real workspace to unit disk computed successfully!", "unit_circle": unit_circle.tolist(), "punctures": punctures.tolist()}  # return a message to the client, along with the unit circle and punctures

# map a point from the real workspace to the sphere world (unit disk) using the harmonic transformation
@app.route("/map_point_to_unit_disk", methods = ["POST"])  # create a route to compute the mapping
def map_point_to_unit_disk():  # map a point from the real workspace to the sphere world (unit disk)
    global hm  # use the global variable hm
    if hm is None: return {"message": "HarmonicMap2D object has not been created!"}, 400  # return an error message if the HarmonicMap2D object has not been created yet
    input_data = request.get_json()  # get the input data from the client (host machine or user)
    p = np.array(input_data["point"])  # get the point coordinates from the input data
    mapped_point = hm.map(p)  # map the point p to the sphere world (unit disk)
    return {"message": "Point mapped to unit disk successfully!", "mapped_point": mapped_point.tolist()}  # return a message to the client, along with the mapped point

# compute the jacobian matrix of the harmonic transformation from the real workspace to the sphere world (unit disk) at a given point
@app.route("/compute_jacobian_at_point", methods = ["POST"])  # create a route to compute the jacobian matrix
def compute_jacobian_at_point():  # compute the jacobian matrix of the harmonic transformation at a given point
    global hm  # use the global variable hm
    if hm is None: return {"message": "HarmonicMap2D object has not been created!"}, 400  # return an error message if the HarmonicMap2D object has not been created yet
    input_data = request.get_json()  # get the input data from the client (host machine or user)
    p = np.array(input_data["point"])  # get the point coordinates from the input data
    jacobian_matrix = hm.jacob(p)  # compute the jacobian matrix at the point p
    return {"message": "Jacobian matrix computed successfully!", "jacobian_matrix": jacobian_matrix.tolist()}  # return a message to the client, along with the jacobian matrix


if __name__ == '__main__':
    app.run(host = "0.0.0.0", port = 5000)  # run the Flask app on the specified host and port