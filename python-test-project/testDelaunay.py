import numpy as np
from scipy.spatial import ConvexHull, Delaunay

def in_poly_hull_single(poly, point):
    hull = ConvexHull(poly)
    new_hull = ConvexHull(np.concatenate((poly, [point])))
    return np.array_equal(new_hull.vertices, hull.vertices)

poly = np.random.rand(65, 3)
point = np.random.rand(3)

poly = np.array([[0,0,0],[0,0,2],[0,2,2],[2,2,2],[2,2,0],[2,0,0],[0,2,0],[2,0,2]])
print(point)
# %timeit in_poly_hull_single(poly, point)
print(Delaunay(poly).find_simplex(point)>=0)

x_unit = [0,1,2,3]
y_unit = [0,1,2,3]
z_unit = [0,1,2,3]

point_value = np.vstack((np.array(x_unit), np.array(y_unit), np.array(z_unit))).T
print(point_value)