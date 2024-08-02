import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate as si
from scipy.spatial import Voronoi, voronoi_plot_2d

def density(q, pt, sigma, size):
    phi = 0
    for j in range(size):
        x = -( (np.power(q-pt[j],2))/ (2*np.power(sigma[j],2)))
        phi += np.exp(x)
    return phi

def error_integrand(t):
    return np.exp(-(np.power(t,2)))

def error_function(x):
    integ = si.quad(error_integrand, 0, x)[0]
    return (np.pi/2)*integ

def error_inner(s, X, xt, sigma, j, l, l_size):
    return ((((X[(l+1)%l_size] - X[l])*s) + X[l] - xt[j])/(np.sqrt(2)*sigma[j]))

def e_inner(s, X, xt, j, l, l_size):
    return -0.5 * np.power(((X[(l+1)%l_size] - X[l])*s) + X[l] - xt[j], 2)

def mass_integrand(s,X,xt,Y,yt,sigma,j,l,l_size):
    error_input = error_inner(s, X, xt, sigma, j, l, l_size)
    error = error_function(error_input)
    e_input = e_inner(s, Y, yt, j, l, l_size)
    exp = np.exp(e_input)
    leftover = Y[(l+1)%l_size] - Y[l]
    return (error * exp) * leftover


def mass_voronoi_i(X, xt, Y, yt, sigma, j_size, l_size):
    sum = 0
    for j in range(j_size):
        inner_integ = 0

        for l in range(l_size):
            inner_integ += si.quad(mass_integrand, 0, 1, args=(X,xt,Y,yt,sigma,j,l,l_size))[0]
        
        leftover_out = (np.sqrt(2*np.pi) * sigma[j])/2
        sum += leftover_out*inner_integ
    return sum

def centroid_x_integrand(s,X,xt,Y,yt,sigma,j,l,l_size):
    error = error_function(error_inner(s, Y, yt, sigma, j, l, l_size))
    exp = np.exp(e_inner(s, X, xt, j, l, l_size))
    leftover_inner = (X[(l+1)%l_size] - X[l])*s + X[l]

    return leftover_inner * exp * error


def centroid_x(X, xt, Y, yt, sigma, j_size, l_size):
    mass = mass_voronoi_i(X, xt, Y, yt, sigma, j_size, l_size)
    loop_result = 0
    for j in range(j_size):
        for l in range(l_size):
            integ_result = si.quad(centroid_x_integrand, 0, 1, args=(X,xt,Y,yt,sigma,j,l,l_size))[0]
            loop_result +=  (np.sqrt(2 * np.pi) * sigma[j] * 0.5 * (X[l] - X[(l+1)%l_size])) * integ_result
    return loop_result/mass

def centroid_y(X, xt, Y, yt, sigma, j_size, l_size):
    mass = mass_voronoi_i(X, xt, Y, yt, sigma, j_size, l_size)
    loop_result = 0
    for j in range(j_size):
        for l in range(l_size):
            integ_result = si.quad(centroid_x_integrand, 0, 1, args=(Y,yt,X,xt,sigma,j,l,l_size))[0]
            loop_result +=  (np.sqrt(2 * np.pi) * sigma[j] * 0.5 * (Y[l] - Y[(l+1)%l_size])) * integ_result
    return loop_result/mass



# Generate random points
points = np.random.rand(6, 2)  # Change the number of points as desired

# Compute Voronoi diagram
vor = Voronoi(points)

# Plot Voronoi diagram
voronoi_plot_2d(vor)
plt.show()

# Print vertices of each cell
for region in vor.regions:
    if len(region) > 0 and -1 not in region:
        vertices = vor.vertices[region]
        print("Vertices:", vertices)
        print()

