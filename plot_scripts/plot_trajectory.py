import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import time
import pandas as pd
from tess import Container
from scipy.spatial import ConvexHull

folder_name = 'sym1'
d1 = pd.read_csv(folder_name +'/trajectories.csv', header = None)
# d2 = pd.read_csv('d2.csv', header = None)
# d3 = pd.read_csv('d3.csv', header = None)
# d4 = pd.read_csv('d4.csv', header = None)
# d5 = pd.read_csv('d5.csv', header = None)
# d6 = pd.read_csv('d6.csv', header = None)
# d7 = pd.read_csv('d7.csv', header = None)
# d8 = pd.read_csv('d8.csv', header = None)

frame = d1.shape[0]
voronoiLimit = [(-20,-20, 0),(20,20,40)]
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_zlim(0, 40)
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_xlabel('x',fontsize=18)
ax.set_ylabel('y',fontsize=18)
ax.set_zlabel('z',fontsize=18)

# d1Len = len(d1)
ax.plot(d1[0],d1[1], d1[2], c='k', label='drone1')
# for x in range(d1Len):
#     ax.scatter(d1[0][x], d1[1][x], d1[2][x], c='k', label='drone1')
ax.plot(d1[6],d1[7], d1[8], c='y', label='drone2')
ax.plot(d1[12],d1[13], d1[14], c='g', label='drone3')
ax.plot(d1[18],d1[19], d1[20], c='c', label='drone4')
ax.plot(d1[24],d1[25], d1[26], c='b', label='drone5')
ax.plot(d1[30],d1[31], d1[32], c='m', label='drone6')
ax.plot(d1[36],d1[37], d1[38], c='r', label='drone7')
ax.plot(d1[42],d1[43], d1[44], c='#da8200', label='drone8')


plt.legend()
# plt.show()
# plt.savefig('/home/robolab/plot_result/'+folder_name+'/trajectories.png')

def resetAxes():
    ax.clear()
    ax.set_zlim(0, 40)
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

def plotVoronoi(ax, position, colorIndex, destination, vertices=[], faceVertices=[]):
    # poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
    # ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=color,linewidths=1, alpha=0.2))
    poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
    ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=colorList[colorIndex],linewidths=1, alpha=0.2))
    # ax.scatter(centroid[0], centroid[1], centroid[2], c='r', s=30)

    # for tri in collection:
    #         ax.add_collection3d(tri)

    ax.scatter(destination[0], destination[1], destination[2], c='g', s=30)
    ax.scatter(position[0], position[1], position[2], c='r', s=30)
    ax.plot([position[0],destination[0]],[position[1],destination[1]],[position[2],destination[2]],color = 'k', linestyle='dashed')

o1 = [d1[0][0], d1[1][0], d1[2][0]]
o2 = [d1[6][0], d1[7][0], d1[8][0]]
o3 = [d1[12][0], d1[13][0], d1[14][0]]
o4 = [d1[18][0], d1[19][0], d1[20][0]]
o5 = [d1[24][0], d1[25][0], d1[26][0]]
o6 = [d1[30][0], d1[31][0], d1[32][0]]
o7 = [d1[36][0], d1[37][0], d1[38][0]]
o8 = [d1[42][0], d1[43][0], d1[44][0]]
ax.set_zlim(0, 40)
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
for i in range(frame):
    if i == 0:
        break
    u1 = [d1[0][i], d1[1][i], d1[2][i]]
    u2 = [d1[6][i], d1[7][i], d1[8][i]]
    u3 = [d1[12][i], d1[13][i], d1[14][i]]
    u4 = [d1[18][i], d1[19][i], d1[20][i]]
    u5 = [d1[24][i], d1[25][i], d1[26][i]]
    u6 = [d1[30][i], d1[31][i], d1[32][i]]
    u7 = [d1[36][i], d1[37][i], d1[38][i]]
    u8 = [d1[42][i], d1[43][i], d1[44][i]]
    
    # container = Container(positions, limits=voronoiLimit, periodic=False)
    # j = 0
    # for cell in container:
        
    #     plotVoronoi(ax, positions[j], j, destinations[j], cell.vertices(), cell.face_vertices())
    #     j +=1
    # for j in range(len(positions)):
    #     plotVoronoi(ax, positions[j], j, destinations[j])
        


plt.savefig('/home/robolab/plot_result/cbf_cvt_cell/trajectory.png')






# print(d1[0])
