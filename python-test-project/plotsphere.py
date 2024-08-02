import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def sphere(o, r):
    u,v=np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]
    x=np.cos(u)*np.sin(v)*r + o[0]
    y=np.sin(u)*np.sin(v)*r + o[1]
    z=np.cos(v)*r + o[2]
    ax.plot_wireframe(x, y, z, color='lightskyblue', linewidth=0.5)

fig = plt.figure()
ax = Axes3D(fig)
ax.set_zlim(0, 20)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)

sphere([5,5,5], 3)
plt.show()
