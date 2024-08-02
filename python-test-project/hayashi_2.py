import numpy as np
import scipy.integrate as si
from scipy.special import erf
import time
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from shapely import geometry
from scipy.stats import multivariate_normal


# def density(q, pt, sigma, size):
#     phi = 0
#     for j in range(size):
#         x = -( (np.power(q-pt[j],2))/ (2*np.power(sigma[j],2)))
#         phi += np.exp(x)
#     return phi

def error_integrand(x):
    return np.exp(-(np.power(x,2)))

def error_function(x):
    integ = si.quad(error_integrand, 0, x)[0]
    return (2/np.sqrt(np.pi))*integ

def error_inner(s, X, xt, sigma, j, l, l_size):
    return ((((X[(l+1)%l_size] - X[l])*s) + X[l] - xt[j])/(np.sqrt(2)*sigma[j]))

def e_inner(s, X, xt, sigma, j, l, l_size):
    return -0.5 * np.power((((X[(l+1)%l_size] - X[l])*s) + X[l] - xt[j])/sigma[j], 2)


#-----------------------------------------------------------------------------------------------------
# def mass_integrand(s,X,xt,Y,yt,sigma,j,l,l_size):
#     error_input = error_inner(s, X, xt, sigma, j, l, l_size)
#     error = error_function(error_input)
#     e_input = e_inner(s, Y, yt, sigma, j, l, l_size)
#     exp = np.exp(e_input)
#     return (error * exp)


# def mass_voronoi_i(X, xt, Y, yt, sigma, j_size, l_size):
#     result = 0
#     for j in range(j_size):
#         for l in range(l_size):
#             integ_result = si.quad(mass_integrand, 0, 1, args=(X,xt,Y,yt,sigma,j,l,l_size))[0]
#             leftover = Y[(l+1)%l_size] - Y[l]
#             result += ((np.sqrt(2*np.pi) * sigma[j])/2)*leftover*integ_result
#     return result
#-----------------------------------------------------------------------------------------------------
def mass_integrand(s,X,xt,Y,yt,sigma,j,l,l_size):
    error_input = error_inner(s, X, xt, sigma, j, l, l_size)
    # error = error_function(error_input)
    error = erf(error_input)
    e_input = e_inner(s, Y, yt, sigma, j, l, l_size)
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

# def alter_mass(X, xt, Y, yt, sigma):
#     for 
#     a = (1/2)*np.pi*np.power(sigma,2)
#     erf1 = erf(((xt-X)/(np.sqrt(2)*sigma)))
#     erf2 = erf(((yt-Y)/(np.sqrt(2)*sigma)))
#     return a*erf1*erf2

#-----------------------------------------------------------------------------------------------------
# def centroid_x_intergrand(s,X,xt,Y,yt,sigma,j,l,l_size):
#     error_input = (((Y[(l+1)%l_size] - Y[l])*s)+Y[l]-yt[j])/(np.sqrt(2)*sigma[j])
#     error = error_function(error_input)
#     e_input = (-0.5*np.power((((X[(l+1)%l_size] - X[l])*s)+X[l]-xt[j])/sigma[j],2))
#     e = np.exp(e_input)
#     leftover = ((X[(l+1)%l_size] - X[l])*s)+ X[l]
#     return (leftover*e*error)*(X[l]-X[(l+1)%l_size])
# def centroid_x(X,xt,Y,yt,sigma,j_size,l_size):
#     result = 0
#     for j in range(j_size):
#         l_loop_result = 0
#         for l in range(l_size):
#             integ = si.quad(centroid_x_intergrand, 0, 1, args=(X,xt,Y,yt,sigma,j,l,l_size))[0]
#             leftover = (np.sqrt(2*np.pi) * sigma[j])/2
#             l_loop_result += leftover * integ
#         result += l_loop_result
#     mass = mass_voronoi_i(X,xt,Y,yt,sigma,j_size,l_size)
#     return result/mass

# def centroid_y_intergrand(s,X,xt,Y,yt,sigma,j,l,l_size):
#     error_input = (((X[(l+1)%l_size] - X[l])*s)+X[l]-xt[j])/(np.sqrt(2)*sigma[j])
#     error = error_function(error_input)
#     e_input = (-0.5*np.power((((Y[(l+1)%l_size] - Y[l])*s)+Y[l]-yt[j])/sigma[j],2))
#     e = np.exp(e_input)
#     leftover = ((Y[(l+1)%l_size] - Y[l])*s)+ Y[l]
#     return (leftover*e*error)*(Y[l]-Y[(l+1)%l_size])
# def centroid_y(X,xt,Y,yt,sigma,j_size,l_size):
#     result = 0
#     for j in range(j_size):
#         l_loop_result = 0
#         for l in range(l_size):
#             integ = si.quad(centroid_y_intergrand, 0, 1, args=(X,xt,Y,yt,sigma,j,l,l_size))[0]
#             leftover = (np.sqrt(2*np.pi) * sigma[j])/2
#             l_loop_result += leftover * integ
#         result += l_loop_result
#     mass = mass_voronoi_i(X,xt,Y,yt,sigma,j_size,l_size)
#     return result/mass
#----------------------------------------------------------------------------------------



def centroid_x_integrand(s,X,xt,Y,yt,sigma,j,l,l_size):
    # error = error_function(error_inner(s, Y, yt, sigma, j, l, l_size))
    error = erf(error_inner(s, Y, yt, sigma, j, l, l_size))
    exp = np.exp(e_inner(s, X, xt, sigma, j, l, l_size))
    leftover_inner = (X[(l+1)%l_size] - X[l])*s + X[l]

    return leftover_inner * exp * error

def centroid_y_integrand(s,X,xt,Y,yt,sigma,j,l,l_size):
    # error = error_function(error_inner(s, X, xt, sigma, j, l, l_size))
    error = erf(error_inner(s, X, xt, sigma, j, l, l_size))
    exp = np.exp(e_inner(s, Y, yt, sigma, j, l, l_size))
    leftover_inner = (Y[(l+1)%l_size] - Y[l])*s + Y[l]

    return leftover_inner * exp * error


def centroid_x(X, xt, Y, yt, sigma, j_size, l_size, original_cx):
    mass = mass_voronoi_i(X, xt, Y, yt, sigma, j_size, l_size)
    # alter_mass = alter_mass()
    print("Mass:",mass)
    # if mass <= 0:
    #     return original_cx
    loop_result = 0
    for j in range(j_size):
        for l in range(l_size):
            integ_result = si.quad(centroid_x_integrand, 0, 1, args=(X,xt,Y,yt,sigma,j,l,l_size))[0]
            loop_result +=  (np.sqrt(2 * np.pi) * sigma[j] * 0.5 * (X[l] - X[(l+1)%l_size])) * integ_result
    return loop_result/mass

def centroid_y(X, xt, Y, yt, sigma, j_size, l_size, original_cy):
    mass = mass_voronoi_i(X, xt, Y, yt, sigma, j_size, l_size)
    # if mass <= 0:
    #     return original_cy
    loop_result = 0
    for j in range(j_size):
        for l in range(l_size):
            integ_result = si.quad(centroid_y_integrand, 0, 1, args=(X,xt,Y,yt,sigma,j,l,l_size))[0]
            loop_result +=  (np.sqrt(2 * np.pi) * sigma[j] * 0.5 * (Y[(l+1)%l_size] - Y[l])) * integ_result
    return loop_result/mass


start = time.time()

points = [[0.3, 0.9],[0.6,0.12],[0.15,0.3],[0.12,0.9],[0.6,0.4],[0.9,0.15]]
# points = [[0,0],[-10, 10],[0,10],[10,10],[-10,0],[10,0],[-10,-10],[0,-10],[10,-10]]
# points = [[-10,-10],[10,10],[-10,10],[10,-10]]
# points = [[0.24,0.06],[0.21,0.1],[0.22,0.21],[0.11,0.05],[0.22,0.26],[0.05,0.2],[0.15,0.23]]
# points = np.random.rand(6, 2)
coords = np.array(points)
rec_bound = [[0,0],[1,0],[1,1],[0,1]]
rec_poly = Polygon(rec_bound)
poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, rec_poly)
# print(poly_to_pt_assignments)

# fake_voronoi_x = np.array([[-5, -5, 0, 0],[0, 0, 5 ,5],[-5, -5, 0 ,0],[0, 0, 5 ,5]])
# fake_voronoi_y = np.array([[5, 0, 0, 5],[5, 0, 0, 5],[0, -5, -5, 0],[0, -5, -5, 0]])
# fake_voronoi_x = np.array([[-6, -6, -2, -2],[-2, -2, 2 ,2],[2, 2, 6 ,6],[-6, -6, -2, -2],[-2, -2, 2 ,2],[2, 2, 6 ,6],[-6, -6, -2, -2],[-2, -2, 2 ,2],[2, 2, 6 ,6]])
# fake_voronoi_y = np.array([[6, 2, 2, 6],[6, 2, 2, 6],[6, 2, 2, 6],[2,-2,-2,2],[2,-2,-2,2],[2,-2,-2,2],[-2,-6,-6,-2],[-2,-6,-6,-2],[-2,-6,-6,-2]])
# X1 = [-5, -5, 0, 0]
# Y1 = [5, 0, 0, 5]
# X2 = [0, 0, 5 ,5]
# Y2 = [5, 0, 0, 5]
# X3 = [-5, -5, 0 ,0]
# Y3 = [0, -5, -5, 0]
# X4 = [0, 0, 5 ,5]
# Y4 = [0, -5, -5, 0]

# xt = [2,-8]
# yt = [2,-8]
# sigma = [1,3]

xt = [0.1,0.6]
yt = [0.8,0.6]
sigma = [0.2,0.1]

# j_size = 1
# l_size = 9

shapes = []
fig = plt.figure()
for i in range(len(xt)):
    x_unit = np.random.normal(xt[i], sigma[i], 3000) 
    y_unit = np.random.normal(yt[i], sigma[i], 3000)
    plt.scatter(x_unit, y_unit, c='darkgrey', s=0.5)
# print(fake_voronoi_x)
# print(fake_voronoi_y)

color = ['b','g','r','c','m','y','k','pink','peru']

for i in range(len(poly_shapes)):
# for i in range(len(fake_voronoi_x)):
    x_clockwise,y_clockwise = poly_shapes[i].exterior.xy
    # x = x_clockwise[::-1]
    # y = y_clockwise[::-1]
    x = x_clockwise
    y = y_clockwise
    # print("shape :", str(i))
    # print(len(x))
    # print([x,y])
    # print(i)
    original_centroid = poly_shapes[i].centroid.coords[0]
    original_cx = original_centroid[0]
    original_cy = original_centroid[1]
    print([x,y])
    cx = centroid_x(x, xt, y, yt, sigma, len(xt), len(x), original_cx)
    cy = centroid_y(x, xt, y, yt, sigma, len(xt), len(x), original_cy)

    # point = geometry.Point([cx,cy]) # turn x_unit and y_unit into Point
    # if not point.within(poly_shapes[i]):
    #     cx = original_cx
    #     cy = original_cy

    # print([cx, cy])
    # cx = centroid_x(fake_voronoi_x[i], xt, fake_voronoi_y[i], yt, sigma, j_size, len(fake_voronoi_x[i]))
    # cy = centroid_y(fake_voronoi_x[i], xt, fake_voronoi_y[i], yt, sigma, j_size, len(fake_voronoi_x[i]))
    # print([cx,cy])
    plt.scatter(cx,cy,c='b',s=50, marker='s')
    # plt.scatter(cx, cy, c='r', s=40)
    # shape = np.array([fake_voronoi_x[i], fake_voronoi_y[i]]).T
    # print(shape)
    # polygon = Polygon(shape)
    # x,y = polygon.exterior.xy
    plt.plot(x,y,c='k')
    # print([original_cx,original_cy])
    print([cx,cy])

    

# polygon1 = Polygon(shape1)
# polygon2 = Polygon(shape2)
# polygon3 = Polygon(shape3)
# polygon4 = Polygon(shape4)

# x1,y1 = polygon1.exterior.xy
# x2,y2 = polygon2.exterior.xy
# x3,y3 = polygon3.exterior.xy
# x4,y4 = polygon4.exterior.xy
# plt.plot(x1,y1)
# plt.plot(x2,y2)
# plt.plot(x3,y3)
# plt.plot(x4,y4)



# cx = centroid_x(X, xt, Y, yt, sigma, j_size, l_size)
# cy = centroid_y(X, xt, Y, yt, sigma, j_size, l_size)


for p in points:
    plt.scatter(p[0], p[1], c='r', s=40)
# for i in range(len(xt)):
#     plt.scatter(xt[i], yt[i], c='#FFC0CB', s=20)
plt.xlim([0,1])
plt.ylim([0,1])

plt.show()
# print(mass_voronoi_i(X, xt, Y, yt, sigma, j_size, l_size))
# print([cx, cy])
print(time.time() - start)





