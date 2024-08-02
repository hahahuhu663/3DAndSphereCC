import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import Polygon

# Define the vertices of the convex polyhedron
# vertices = np.array([
#     [0, 0, 0],
#     [1, 0, 0],
#     [1, 1, 0],
#     [0, 1, 0],
#     [0.5, 0.5, 1]
# ])
vertices = np.array([[-20.        , -17.30102041, -18.41326531],
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

# Define the faces of the convex polyhedron
# faces = np.array([
#     [0, 1, 2, 3],
#     [0, 1, 4],
#     [1, 2, 4],
#     [2, 3, 4],
#     [3, 0, 4]
# ])
faces = np.array([[1, 3, 5, 2, 7, 11], [1, 10, 9, 0, 8, 3], [1, 11, 10], [2, 5, 13, 12], [2, 12, 4, 6, 7], [3, 8, 13, 5], [4, 0, 9, 6], [4, 12, 13, 8, 0], [6, 9, 10, 11, 7]])

# Create a 3D plot for the polyhedron
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# Plot the polyhedron in 3D
vertices_3d = []
for face in faces:
    vertices_3d.append(vertices[face])
# ax.add_collection3d(Poly3DCollection(vertices_3d, facecolor='lightblue'))

# # Set the limits of the 3D plot
# ax.set_xlim([-20, 20])
# ax.set_ylim([-20, 20])
# ax.set_zlim([-20, 20])

# # Save the figure with the 3D view
# plt.savefig('3d_view.png')
# plt.close()

# Create a 2D plot for the top view
result_polygon_top = None
fig, ax = plt.subplots()
for face in faces:
    vertices_2d = vertices[face][:, :2]

    shape_polygon = Polygon(vertices_2d)
    if result_polygon_top is None:
        result_polygon_top = shape_polygon
    else:
        result_polygon_top = result_polygon_top.union(shape_polygon)
   
    # vertices_2d_top.append(vertices_2d)
    # ax.add_patch(plt.Polygon(vertices_2d, facecolor='lightblue'))

# Set the limits and aspect ratio of the top view
result_vertices_top = list(result_polygon_top.exterior.coords)[::-1]
# print(result_vertices)
# ax.add_patch(plt.Polygon(result_vertices, facecolor='lightblue'))
# ax.set_xlim([-20, 20])
# ax.set_ylim([-20, 20])
# ax.set_aspect('equal')

# Save the figure with the top view
# plt.savefig('top_view.png')
# plt.close()

# Create a 2D plot for the front view
fig, ax = plt.subplots()
for face in faces:
    vertices_2d = vertices[face][:, [0, 2]]
    ax.add_patch(plt.Polygon(vertices_2d, facecolor='lightblue'))

# Set the limits and aspect ratio of the front view
# ax.set_xlim([-20, 20])
# ax.set_ylim([-20, 20])
# ax.set_aspect('equal')

# # Save the figure with the front view
# plt.savefig('front_view.png')
# plt.close()

# Create a 2D plot for the side view
fig, ax = plt.subplots()
for face in faces:
    vertices_2d = vertices[face][:, [1, 2]]
    ax.add_patch(plt.Polygon(vertices_2d, facecolor='lightblue'))

# Set the limits and aspect ratio of the side view
ax.set_xlim([-20, 20])
ax.set_ylim([-20, 20])
ax.set_aspect('equal')

# Save the figure with the side view
plt.savefig('side_view.png')
plt.close()