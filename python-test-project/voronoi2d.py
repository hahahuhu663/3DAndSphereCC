import numpy as np
import time
from shapely import geometry
from scipy.spatial import ConvexHull, Delaunay
# from shapely.geometry import Polygon
import matplotlib.pyplot as plt
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords

def in_poly_hull_single(poly, point):
    hull = ConvexHull(poly)
    new_hull = ConvexHull(np.concatenate((poly, [point])))
    return np.array_equal(new_hull.vertices, hull.vertices)

def is_point_inside_polygon(point, polygon):
    """
    Check if a point is inside a convex polygon.

    Args:
        point: Tuple (x, y) representing the coordinates of the point to be tested.
        polygon: List of tuples [(x1, y1), (x2, y2), ..., (xn, yn)] representing the vertices of the convex polygon.

    Returns:
        True if the point is inside the polygon, False otherwise.
    """

    # Extract x and y coordinates of the point
    x, y = point

    # Initialize counters
    cross_product_sign = None
    n = len(polygon)

    for i in range(n):
        # Get consecutive vertices of the polygon
        vertex_1 = polygon[i]
        vertex_2 = polygon[(i + 1) % n]

        # Calculate the cross product
        cross_product = (vertex_2[0] - vertex_1[0]) * (y - vertex_1[1]) - (x - vertex_1[0]) * (vertex_2[1] - vertex_1[1])

        # Check the sign of the cross product
        if cross_product == 0:
            # The point is on an edge of the polygon
            return True
        elif cross_product_sign is None:
            # Set the initial sign of the cross product
            cross_product_sign = cross_product
        elif cross_product_sign * cross_product < 0:
            # The signs of the cross products differ, point is outside the polygon
            return False

    # The signs of all cross products are the same, point is inside the polygon
    return True



start = time.time()

# points = [[1, 3],[2,-4],[5,1],[-4,3],[0,0],[-3,-3],[-2,0],[3,-4]]
points = [[0,0],[-10, 10],[0,10],[10,10],[-10,0],[10,0],[-10,-10],[0,-10],[10,-10]]
coords = np.array(points)
rec_bound = [[-20,20],[-20,-20],[20,-20],[20,20]]
rec_poly = geometry.Polygon(rec_bound)
poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, rec_poly)

xt = [-8]
yt = [-8]
sigma = [2]
j_size = 1
l_size = 9

shapes = []
# fig = plt.figure()
x_unit = np.random.normal(xt[0], sigma[0], 3000) 
y_unit = np.random.normal(yt[0], sigma[0], 3000)
plt.scatter(x_unit, y_unit, c='grey', s=0.5)

randomPoints = np.vstack((np.array(x_unit), np.array(y_unit))).T 

region_value =[[] for i in range(len(poly_shapes))]
for i, p in enumerate(randomPoints):
            for j, poly in enumerate(poly_shapes):
                point = geometry.Point(p) # turn x_unit and y_unit into Point
                if point.within(poly): # turn x_unit and y_unit (Point form) is within poly_shapes
                # x,y = poly.exterior.xy
                # re_shaped = np.array([x,y]).T
                # if is_point_inside_polygon(p, re_shaped):
                # if in_poly_hull_single(re_shaped, p):
                # if Delaunay(re_shaped).find_simplex(p) >= 0:
                    region_value[j].append(p)


centroids =[]
for i in range(len(poly_shapes)):
    cx = 0
    cy = 0
    size = len(region_value[i])
    if size != 0:
        for j in region_value[i]:
            cx += j[0]
            cy += j[1]
        cx = cx/size
        cy = cy/size
    else:
         centroid = poly_shapes[i].centroid.coords[0]
         cx = centroid[0]
         cy = centroid[1]
    centroids.append([cx,cy])
    print([cx,cy])
    x,y = poly_shapes[i].exterior.xy  
    plt.scatter(cx,cy,c='r',s=30)
    plt.plot(x,y)
# print(centroids)
print(points)
# for i in range(len(xt)):
#     plt.scatter(xt[i], yt[i], c='#FFC0CB', s=20)
# for p in points:
#     plt.scatter(p[0], p[1], c='b', s=10)

plt.xlim([-20,20])
plt.ylim([-20,20])
plt.show()
print(time.time() - start)