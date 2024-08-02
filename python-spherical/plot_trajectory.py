import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import time
import pandas as pd
from tess import Container
from scipy.spatial import ConvexHull

folder_name = "/home/robolab/python-spherical/15Jul_rand_im3"
d1 = pd.read_csv(folder_name +'/trajectories.csv', header = None)
# d2 = pd.read_csv('d2.csv', header = None)
# d3 = pd.read_csv('d3.csv', header = None)
# d4 = pd.read_csv('d4.csv', header = None)
# d5 = pd.read_csv('d5.csv', header = None)
# d6 = pd.read_csv('d6.csv', header = None)
# d7 = pd.read_csv('d7.csv', header = None)
# d8 = pd.read_csv('d8.csv', header = None)

frame = d1.shape[0]
# voronoiLimit = [(-20,-20, 0),(20,20,40)]
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_zlim(-20, 20)
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_xlabel('x',fontsize=18)
ax.set_ylabel('y',fontsize=18)
ax.set_zlabel('z',fontsize=18)
radius = 20
center = [0,0,0]

u,v=np.mgrid[0:2*np.pi:20j, 0:np.pi:20j]
x=np.cos(u)*np.sin(v)*(radius) + center[0]
y=np.sin(u)*np.sin(v)*(radius) + center[1]
z=np.cos(v)*(radius) + center[2]
ax.plot_wireframe(x, y, z, color='lightskyblue', linewidth=0.5)

# d1Len = len(d1)
ax.plot(d1[0],d1[1], d1[2], c='b', label='drone01')
# for x in range(d1Len):
#     ax.scatter(d1[0][x], d1[1][x], d1[2][x], c='k', label='drone1')
ax.plot(d1[6],d1[7], d1[8], c='y', label='drone02')
ax.plot(d1[12],d1[13], d1[14], c='g', label='drone03')
ax.plot(d1[18],d1[19], d1[20], c='c', label='drone04')
ax.plot(d1[24],d1[25], d1[26], c='b', label='drone05')
ax.plot(d1[30],d1[31], d1[32], c='m', label='drone06')
ax.plot(d1[36],d1[37], d1[38], c='r', label='drone07')
ax.plot(d1[42],d1[43], d1[44], c='#da8200', label='drone08')



plt.legend(loc='upper right', fontsize="8")
# plt.show()
plt.savefig(folder_name+'/trajectories.png')

def resetAxes():
    ax.clear()
    ax.set_zlim(-20, 20)
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)

colorList = ['k','y','g','c','b','m','r','w']


# obstacles = np.array([
#      pipesCombined, cratesStack, topCrate, nearPipeCrate, lonelyCrate
#     ])

# collection = []

# for prism in obstacles:
#         hull = ConvexHull(prism)
#         # draw the polygons of the convex hull
#         for s in hull.simplices:
#             tri = Poly3DCollection([prism[s]])
#             tri.set_color('b')
#             tri.set_alpha(0.5)
#             collection.append(tri)

