import freud
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# points = np.array([[-0.5, -0.5, -0.5], [0.5, -0.5, 0.5],[0,0,0] , [-0.5, 0.5, -0.5], [0.5, 0.5, 0.5]])
# points = np.array([[0, 1, 1], [0, 2, 0], [1, 0, 1], [1, 1, 0], [1, 2, 1]])
points = np.array([[-1,1,1],[1,-1,1],[1,1,-1],[-1,-1,1],[-1,1,-1],[1,-1,-1],[-1,-1,-1],[1,1,1]])

plt.scatter(points[:, 0], points[:, 1])
plt.title("Points")
plt.xlim((-1, 1))
plt.ylim((-1, 1))
plt.gca().set_aspect("equal")
# plt.savefig('/home/robolab/python-test-project/1.png')


L = 4
box = freud.box.Box.cube(L)
voro = freud.locality.Voronoi()
cells = voro.compute((box, points)).polytopes
# print(cells)
# plt.figure()
# ax = plt.gca()
# voro.plot(ax=ax)
# ax.scatter(wpoints[:, 0], points[:, 1], s=10, c="k")

# plt.savefig('/home/robolab/python-test-project/2.png')
axes = [4, 4, 4]
# Create Data
data = np.ones(axes, dtype=np.bool)
# Control Transparency
alpha = 0.5
# Control colour
colors = np.empty(axes + [4], dtype=np.float32)
colors[:] = [1, 0, 0, alpha]  # red
# Plot figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_ylim(-2,2)
ax.set_xlim(-2,2)
ax.set_zlim(-2,2)
# Voxels is used to customizations of the
# sizes, positions and colors.
print(data)
ax.voxels(data, facecolors=colors)
plt.show()
