import freud
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

points = np.array([[-0.5, -0.5, 0], [0.5, -0.5, 0], [-0.5, 0.5, 0], [0.5, 0.5, 0]])
plt.scatter(points[:, 0], points[:, 1])
plt.title("Points")
plt.xlim((-1, 1))
plt.ylim((-1, 1))
plt.gca().set_aspect("equal")
plt.show()

L = 2
box = freud.box.Box.square(L)
voro = freud.locality.Voronoi()

cells = voro.compute((box, points)).polytopes

plt.figure()
ax = plt.gca()
voro.plot(ax=ax)
ax.scatter(points[:, 0], points[:, 1], s=10, c="k")
plt.show()
