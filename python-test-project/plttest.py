import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection

A = np.random.random((100,3))
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
# ax.scatter(A[:,0], A[:,1], A[:,2])
# ax.autoscale()
plt.show()


# import matplotlib.pyplot as plt
# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')
# plt.show()
