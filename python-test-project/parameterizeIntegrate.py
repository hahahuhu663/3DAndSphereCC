from scipy.spatial import Delaunay
from scipy import integrate
import matplotlib.pyplot as plt
import numpy as np
import time
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from tess import Container

# poly_vertices = np.array([[-20.        , -17.30102041, -18.41326531],
#        [  3.00681199,  10.18119891,  -6.42915531],
#        [  4.11111111,  -2.90929705,   6.34693878],
#        [ -1.56017192,  -2.84813754, -14.48853868],
#        [-20.        ,  -3.93201133,   9.8101983 ],
#        [  0.84322954,  -7.37762491,  -2.6564084 ],
#        [-20.        ,  20.        ,  13.4       ],
#        [  9.01310044,  20.        ,   9.04803493],
#        [-19.67225951, -17.33780761, -18.56375839],
#        [-20.        ,  20.        ,   9.5625    ],
#        [  4.1969697 ,  20.        ,   0.48863636],
#        [  7.609375  ,  20.        ,   3.90104167],
#        [  3.97118848,  -3.04921969,   6.34693878],
#        [  0.55740528,  -7.65958668,  -2.62743972]])
# poly_vertices = [np.array([(0.0, 0.0, 0.0), (6.0, 0.0, 0.0), (0.0, 6.0, 0.0), (6.0, 6.0, 0.0), (0.0, 0.0, 6.0), (6.0, 0.0, 6.0), (0.0, 6.0, 6.0), (6.0, 6.0, 6.0)])]
# poly_vertices = [np.array([(0.0, 6.0, 0.5), (5.5, 6.0, 6.0), (0.0, 6.0, 6.0), (0.0, 0.5, 6.0)]),np.array([(0.0, 0.0, 0.0), (6.0, 6.0, 3.833333333333333), (0.0, 6.0, 0.0), (6.0, 0.0, 5.833333333333333), (0.0, 0.0, 6.0), (6.0, 0.0, 6.0), (0.0, 6.0, 0.5), (6.0, 6.0, 6.0), (0.0, 0.5, 6.0), (5.5, 6.0, 6.0), (0.16666666666666674, 0.0, 0.0), (2.166666666666667, 6.0, 0.0)]), np.array([(6.0, 6.0, 3.833333333333333), (6.0, 0.0, 0.0), (6.0, 0.0, 5.833333333333333), (6.0, 6.0, 0.0), (0.16666666666666696, 0.0, 0.0), (2.166666666666667, 6.0, 0.0)])]
# poly_faces = [[[1, 0, 3], [1, 2, 0], [1, 3, 2], [2, 3, 0]],[[1, 11, 10, 3], [1, 7, 9, 6, 2, 11], [1, 3, 5, 7], [2, 0, 10, 11], [2, 6, 8, 4, 0], [3, 10, 0, 4, 5], [4, 8, 9, 7, 5], [6, 9, 8]],[[1, 3, 5, 4], [1, 2, 0, 3], [1, 4, 2], [2, 4, 5, 0], [3, 0, 5]]]
# poly_faces = [[[1, 3, 2, 0], [1, 5, 7, 3], [1, 0, 4, 5], [2, 6, 4, 0], [2, 3, 7, 6], [4, 6, 7, 5]]]

positions = [[2,2,2],[1,3,5],[4,4,4]]
tessellation = Container(positions, limits=[(0,0,0),(6,6,6)], periodic=False)

def density_function(x, y, z):
    exponent = ((x - mean_x)**2 + (y - mean_y)**2 + (z - mean_z)**2)/(-2* sigma**2)
    return np.exp(exponent)

def integrand(u,v, w):
    x, y, z = interpolate_vertex(u,v, w)
    return density_function(x, y, z)

def interpolate_vertex(u,v, w):

    # Perform interpolation based on u and v to get the vertex coordinates
    # x = (1 - u) * (1 - v) * v1[0] + u * (1 - v) * v2[0] + u * v * v3[0] + (1 - u) * v * v4[0]
    # y = (1 - u) * (1 - v) * v1[1] + u * (1 - v) * v2[1] + u * v * v3[1] + (1 - u) * v * v4[1]
    # z = (1 - u) * (1 - v) * v1[2] + u * (1 - v) * v2[2] + u * v * v3[2] + (1 - u) * v * v4[2]

    x = u * v1[0] + v * v2[0] + w * v3[0] + (1 - u - v - w) * v4[0]
    y = u * v1[1] + v * v2[1] + w * v3[1] + (1 - u - v - w) * v4[1]
    z = u * v1[2] + v * v2[2] + w * v3[2] + (1 - u - v - w) * v4[2]
    return x, y, z

def centroid_x_integrand(u,v, w):
    x, y, z = interpolate_vertex(u, v, w)
    # exponent = ((x - mean_x)**2 + (y - mean_y)**2 + (z - mean_z)**2)/(-2* (sigma**2))
    return x*density_function(x, y, z)

def centroid_y_integrand(u,v, w):
    x, y, z = interpolate_vertex(u,v, w)
    # exponent = ((x - mean_x)**2 + (y - mean_y)**2 + (z - mean_z)**2)/(-2* (sigma**2))
    return y*density_function(x, y, z)

def centroid_z_integrand(u,v, w):
    x, y, z = interpolate_vertex(u,v, w)
    # exponent = ((x - mean_x)**2 + (y - mean_y)**2 + (z - mean_z)**2)/(-2* (sigma**2))
    return z*density_function(x, y, z)

# Gaussian density function parameters
mean_x = 0
mean_y = 0
mean_z = 0
sigma = 1    # Mean vector      # Covariance matrix

v1 = []
v2 = []
v3 = []
v4 = []

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_zlim(0, 6)
ax.set_xlim(0, 6)
ax.set_ylim(0, 6)
colorList = ['y','r','g','b']
start = time.time()
indexColor=0
for cell in tessellation:
    cell_vertices = cell.vertices()
    faces = cell.face_vertices()
    tri = Delaunay(cell_vertices)
    shapes = tri.simplices
    points = tri.points
    cxtotal = 0
    cytotal = 0
    cztotal = 0
    masstotal = 0
    for shape in shapes:
        # start = time.time()
        vertices = points[shape]
        # hull = ConvexHull(vertices)
        v1 = vertices[0]
        v2 = vertices[1]
        v3 = vertices[2]
        v4 = vertices[3]
        mass = integrate.tplquad(integrand, 0, 1, lambda u: 0, lambda u: 1 - u, lambda u, v: 0, lambda u, v: 1 - u - v)[0]
        # centroid_x = integrate.dblquad(centroid_x_integrand, 0, 1, lambda v: 0, lambda v: 1 - v)[0]
        # centroid_y = integrate.dblquad(centroid_y_integrand, 0, 1, lambda v: 0, lambda v: 1 - v)[0]
        # centroid_z = integrate.dblquad(centroid_z_integrand, 0, 1, lambda v: 0, lambda v: 1 - v)[0]
        centroid_x = integrate.tplquad(centroid_x_integrand, 0, 1, lambda u: 0, lambda u: 1 - u, lambda u, v: 0, lambda u, v: 1 - u - v)[0]
        centroid_y = integrate.tplquad(centroid_y_integrand, 0, 1, lambda u: 0, lambda u: 1 - u, lambda u, v: 0, lambda u, v: 1 - u - v)[0]
        centroid_z = integrate.tplquad(centroid_z_integrand, 0, 1, lambda u: 0, lambda u: 1 - u, lambda u, v: 0, lambda u, v: 1 - u - v)[0]
        # print("Shape:", i)
        # i+=1
        # print("Mass:", mass)
        # print("Centroid:",[centroid_x, centroid_y, centroid_z])
        cxtotal += centroid_x
        cytotal += centroid_y
        cztotal += centroid_z
        masstotal += mass
        # print("X Y Z:", [centroid_x, centroid_y, centroid_z])
        # print("Hull Volume:", hull.volume)
        # print("Time used:", time.time()-start)
    # print([cxtotal,cytotal,cztotal])
    # print(masstotal)
    cxav = cxtotal/masstotal
    cyav = cytotal/masstotal
    czav = cztotal/masstotal
    print("Centroid", [cxav, cyav, czav])
    poly3d = [[cell_vertices[faces[ix][iy]] for iy in range(len(faces[ix]))] for ix in range(len(faces))]
    ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=colorList[indexColor],linewidths=1, alpha=0.2))
    
    ax.scatter(cxav, cyav, czav, c='k', s=30)
    indexColor+=1
print(time.time()-start)
# def plot_tetra(tetra, pts, color="green", alpha=0.1, lc="k", lw=1):
#     combs = combinations(tetra, 3)
#     for comb in combs:
#         X = pts[comb, 0]
#         Y = pts[comb, 1]
#         Z = pts[comb, 2]
#         verts = [X,Y,Z]
#         triangle = Poly3DCollection(verts, facecolors=color, alpha=0.1)
#         lines = Line3DCollection(verts, colors=lc, linewidths=lw)
#         ax.add_collection3d(triangle)
#         ax.add_collection3d(lines)
 

# for k, tetra in enumerate(tri.simplices):
#     color = plt.cm.Accent(k/(tri.nsimplex - 1))
#     plot_tetra(tetra, poly_vertices, color=color, alpha=0.1, lw=0.5, lc="k")
# ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c='k')
# plt.savefig("Delaunay.png", dpi=600)a
plt.show()
    