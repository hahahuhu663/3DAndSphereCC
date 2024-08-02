from scipy.spatial import Delaunay
import numpy as np
import time

poly_vertices = np.array([[-20.        , -17.30102041, -18.41326531],
       [  3.00681199,  10.18119891,  -6.42915531],
       [  4.11111111,  -2.90929705,   6.34693878],
       [ -1.56017192,  -2.84813754, -14.48853868],
       [-20.        ,  -3.93201133,   9.8101983 ],
       [  0.84322954,  -7.37762491,  -2.6564084 ],
       [-20.        ,  20.        ,  13.4       ],
       [  9.01310044,  20.        ,   9.04803493],
       [-19.67225951, -17.33780761, -18.56375839],
       [-20.        ,  20.        ,   9.5625    ],
       [  4.1969697 ,  20.        ,   0.48863636],
       [  7.609375  ,  20.        ,   3.90104167],
       [  3.97118848,  -3.04921969,   6.34693878],
       [  0.55740528,  -7.65958668,  -2.62743972]])

tri = Delaunay(vertices)
shapes = tri.simplices
points = tri.points

import numpy as np
from scipy import integrate

vertices = points[shapes[0]]
# Gaussian density function parameters
mean = np.array([mean_x, mean_y, mean_z])      # Mean vector
covariance = np.array([[var_x, cov_xy, cov_xz],
                       [cov_xy, var_y, cov_yz],
                       [cov_xz, cov_yz, var_z]])       # Covariance matrix

def density_function(x, y, z):
    position = np.array([x, y, z])
    exponent = -0.5 * (position - mean).T @ np.linalg.inv(covariance) @ (position - mean)
    return np.exp(exponent) / (np.sqrt((2 * np.pi) ** 3 * np.linalg.det(covariance)))

def integrand(u, v):
    x, y, z = interpolate_vertex(u, v)
    return density_function(x, y, z)

def interpolate_vertex(u, v):
    v1 = vertices[0]
    v2 = vertices[1]
    v3 = vertices[2]
    v4 = vertices[3]

    # Perform interpolation based on u and v to get the vertex coordinates
    x = (1 - u) * (1 - v) * v1[0] + u * (1 - v) * v2[0] + u * v * v3[0] + (1 - u) * v * v4[0]
    y = (1 - u) * (1 - v) * v1[1] + u * (1 - v) * v2[1] + u * v * v3[1] + (1 - u) * v * v4[1]
    z = (1 - u) * (1 - v) * v1[2] + u * (1 - v) * v2[2] + u * v * v3[2] + (1 - u) * v * v4[2]

    return x, y, z

# Perform numerical integration
mass, _ = integrate.dblquad(integrand, 0, 1, lambda v: 0, lambda v: 1 - v)

print("Mass:", mass)
