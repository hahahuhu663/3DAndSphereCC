import numpy as np
from scipy import integrate

# Vertices of the polyhedron
vertices = np.array([[-20., -17.30102041, -18.41326531],
                    [3.00681199, 10.18119891, -6.42915531],
                    [4.11111111, -2.90929705, 6.34693878],
                    [-1.56017192, -2.84813754, -14.48853868],
                    [-20., -3.93201133, 9.8101983],
                    [0.84322954, -7.37762491, -2.6564084],
                    [-20., 20., 13.4],
                    [9.01310044, 20., 9.04803493],
                    [-19.67225951, -17.33780761, -18.56375839],
                    [-20., 20., 9.5625],
                    [4.1969697, 20., 0.48863636],
                    [7.609375, 20., 3.90104167],
                    [3.97118848, -3.04921969, 6.34693878],
                    [0.55740528, -7.65958668, -2.62743972]])

# Face vertices of the polyhedron
faces = [[1, 3, 5, 2, 7, 11],
         [1, 10, 9, 0, 8, 3],
         [1, 11, 10],
         [2, 5, 13, 12],
         [2, 12, 4, 6, 7],
         [3, 8, 13, 5],
         [4, 0, 9, 6],
         [4, 12, 13, 8, 0],
         [6, 9, 10, 11, 7]]

# Gaussian density function parameters
mean = np.array([0, 0])      # Mean vector
covariance = np.eye(2)       # Covariance matrix

def density_function(x, y):
    exponent = -0.5 * np.array([x, y]).T @ np.linalg.inv(covariance) @ np.array([x, y])
    return np.exp(exponent) / (2 * np.pi * np.sqrt(np.linalg.det(covariance)))

def integrand(u, v):
    x, y = interpolate_vertex(u, v)
    return density_function(x, y)

def interpolate_vertex(u, v):
    num_faces = len(faces)
    num_vertices = vertices.shape[0]
    interpolated_vertices = np.zeros((num_faces, 3))

    # Interpolate vertex coordinates for each face
    for i, face in enumerate(faces):
        face_vertices = vertices[face]
        interpolated_vertices[i] = np.average(face_vertices, axis=2, weights=[u, v])

    # Interpolate face vertices to get final interpolated vertex
    interpolated_vertex = np.average(interpolated_vertices, axis=0)

    return interpolated_vertex

# Perform numerical integration
volume, _ = integrate.dblquad(integrand, 0, 1, lambda u: 0, lambda u: 1 - u)

print("Volume:", volume)